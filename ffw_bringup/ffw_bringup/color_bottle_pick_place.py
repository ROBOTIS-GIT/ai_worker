#!/usr/bin/env python3

import threading
import time

from cv_bridge import CvBridgeError
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ffw_bringup.vision_pick_place import VisionPickPlace


class ColorBottlePickPlace(VisionPickPlace):
    """Pick red, blue, and green bottles into their matching table boxes."""

    def __init__(
        self,
        node_name='color_bottle_pick_place',
        place_target_name='box',
        default_place_positions=None,
    ):
        super().__init__(node_name)

        if default_place_positions is None:
            default_place_positions = {
                'red': [0.90, 0.35, 0.84],
                'blue': [0.90, 0.0, 0.84],
                'green': [0.90, -0.35, 0.84],
            }
        self.place_target_name = place_target_name

        self.declare_parameter('input_image_topic', '/zedm/rgbd_camera/image')
        self.declare_parameter('color_order', 'red,blue,green')
        self.declare_parameter('red_place_position', default_place_positions['red'])
        self.declare_parameter('blue_place_position', default_place_positions['blue'])
        self.declare_parameter('green_place_position', default_place_positions['green'])
        self.declare_parameter('return_home_between_places', True)
        self.declare_parameter('arm_center_deadband_ratio', 0.15)

        self.input_image_topic = self.get_parameter('input_image_topic').value
        self.color_order = self.get_color_order()
        self.color_place_positions = {
            'red': self.get_list_parameter('red_place_position'),
            'blue': self.get_list_parameter('blue_place_position'),
            'green': self.get_list_parameter('green_place_position'),
        }
        self.return_home_between_places = self.get_bool_parameter('return_home_between_places')
        self.arm_center_deadband_ratio = float(
            self.get_parameter('arm_center_deadband_ratio').value
        )

        self.latest_color_image = None
        self.captured_color_poses = {}

        self.create_subscription(Image, self.input_image_topic, self.image_callback, 10)
        self.create_service(Trigger, '~/capture_all', self.capture_all_callback)
        self.create_service(Trigger, '~/execute_all', self.execute_all_callback)

        self.get_logger().info('Color bottle pick/place node ready')
        self.get_logger().info(f'  color order: {self.color_order}')
        self.get_logger().info(f'  image topic: {self.input_image_topic}')
        self.get_logger().info(f'  left gripper topic: {self.arm_joint_trajectory_topic}')
        self.get_logger().info(f'  right gripper topic: {self.right_arm_joint_trajectory_topic}')
        for color in self.color_order:
            self.get_logger().info(
                f'  {color} place position: {self.color_place_positions[color]}'
            )

    def image_callback(self, msg):
        try:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8',
            )
        except CvBridgeError as exc:
            self.get_logger().warn(
                f'Failed to convert color image: {exc}',
                throttle_duration_sec=2.0,
            )

    def detections_callback(self, msg):
        self.latest_detections = msg

    def capture_callback(self, request, response):
        return self.capture_all_callback(request, response)

    def execute_callback(self, request, response):
        return self.execute_all_callback(request, response)

    def capture_all_callback(self, request, response):
        del request
        color_poses = self.capture_color_poses(log_detection=True)
        self.captured_color_poses = color_poses

        missing = [color for color in self.color_order if color not in color_poses]
        response.success = not missing
        if missing:
            response.message = 'missing bottle detections: ' + ', '.join(missing)
            return response

        response.message = 'captured bottles: ' + ', '.join(self.color_order)
        return response

    def execute_all_callback(self, request, response):
        del request
        if not self.execute_motion:
            response.success = False
            response.message = 'set execute_motion:=true to enable motion execution'
            return response
        if self.execution_lock.locked():
            response.success = False
            response.message = (
                f'pick/place already running at "{self.execution_step}"; '
                f'call /{self.get_name()}/cancel to stop it'
            )
            return response

        self.cancel_event.clear()
        thread = threading.Thread(
            target=self.execute_color_sequence,
            daemon=True,
        )
        thread.start()

        response.success = True
        response.message = f'color bottle to {self.place_target_name} sequence started'
        return response

    def capture_color_poses(self, log_detection):
        color_poses = {}
        for color in self.color_order:
            capture = self.process_color_detection(color, log_detection)
            if capture is not None:
                color_poses[color] = capture
        return color_poses

    def process_color_detection(self, color, log_detection):
        if self.latest_depth_image is None or self.latest_depth_msg is None:
            self.get_logger().warn('No depth image received yet', throttle_duration_sec=2.0)
            return None
        if self.latest_camera_info is None:
            self.get_logger().warn('No camera info received yet', throttle_duration_sec=2.0)
            return None
        if self.latest_color_image is None:
            self.get_logger().warn('No color image received yet', throttle_duration_sec=2.0)
            return None

        if log_detection:
            self.log_current_detections()

        detection = None
        if self.latest_detections is None:
            self.get_logger().warn('No YOLO detections received yet', throttle_duration_sec=2.0)
        else:
            detection = self.select_detection_by_color(self.latest_detections, color)
        if detection is None:
            self.get_logger().warn(f'No matching {color} bottle detection', throttle_duration_sec=2.0)
            return self.process_color_image_fallback(color, log_detection)

        pose = self.detection_to_pose(detection)
        if pose is None:
            return None
        arm = self.select_arm_for_detection(detection, pose)

        self.object_pose_pub.publish(pose)
        self.grasp_pose_pub.publish(self.make_grasp_pose(pose))
        self.place_pose_pub.publish(self.make_color_place_pose(color, pose.header.stamp))

        if log_detection:
            p = pose.pose.position
            self.get_logger().info(
                f'{color} {detection.class_name} score={detection.score:.2f} '
                f'using {arm} arm in {pose.header.frame_id}: '
                f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f})'
            )
        return {'pose': pose, 'arm': arm}

    def process_color_image_fallback(self, color, log_detection):
        pixel = self.find_color_blob_center(color)
        if pixel is None:
            self.get_logger().warn(f'No {color} color blob found in image', throttle_duration_sec=2.0)
            return None

        u, v = pixel
        depth = self.sample_depth(u, v)
        if depth is None:
            self.get_logger().warn(
                f'Invalid depth around {color} color blob pixel ({u}, {v})',
                throttle_duration_sec=1.0,
            )
            return None

        camera_info = self.latest_camera_info
        x = (u - camera_info.k[2]) * depth / camera_info.k[0]
        y = (v - camera_info.k[5]) * depth / camera_info.k[4]
        projection_frame = self.projection_frame or camera_info.header.frame_id
        pose = self.transform_camera_point(projection_frame, x, y, depth)
        if pose is None:
            return None

        arm = self.select_arm_for_pixel(u, pose)
        self.object_pose_pub.publish(pose)
        self.grasp_pose_pub.publish(self.make_grasp_pose(pose))
        self.place_pose_pub.publish(self.make_color_place_pose(color, pose.header.stamp))

        if log_detection:
            p = pose.pose.position
            self.get_logger().info(
                f'{color} image fallback using {arm} arm in {pose.header.frame_id}: '
                f'pixel=({u}, {v}) pose=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})'
            )
        return {'pose': pose, 'arm': arm}

    def find_color_blob_center(self, color):
        image = self.latest_color_image
        if image is None:
            return None

        b = image[:, :, 0].astype(np.float32)
        g = image[:, :, 1].astype(np.float32)
        r = image[:, :, 2].astype(np.float32)

        if color == 'red':
            mask = (r > 70.0) & (r > g * 1.25) & (r > b * 1.25)
        elif color == 'blue':
            mask = (b > 70.0) & (b > g * 1.25) & (b > r * 1.25)
        elif color == 'green':
            mask = (g > 55.0) & (g > r * 1.15) & (g > b * 1.15)
        else:
            return None

        ys, xs = np.where(mask)
        if xs.size < 20:
            return None

        return int(round(float(np.median(xs)))), int(round(float(np.median(ys))))

    def log_current_detections(self):
        detection_summaries = []
        for detection in self.latest_detections.detections:
            detection_summaries.append(
                f'{detection.class_name}:{detection.score:.2f}'
                f'@({detection.bbox.center.position.x:.0f},'
                f'{detection.bbox.center.position.y:.0f})'
            )
        if detection_summaries:
            self.get_logger().info('Current detections: ' + ', '.join(detection_summaries))
        else:
            self.get_logger().warn('Current detections: none above min_score')

    def select_arm_for_detection(self, detection, pose):
        center_x = float(detection.bbox.center.position.x)
        return self.select_arm_for_pixel(center_x, pose)

    def select_arm_for_pixel(self, center_x, pose):
        if self.latest_color_image is None:
            return 'left'
        image_width = self.latest_color_image.shape[1]
        image_center_x = image_width * 0.5
        deadband_half_width = image_width * self.arm_center_deadband_ratio * 0.5

        if center_x > image_center_x + deadband_half_width:
            return 'right'
        if center_x < image_center_x - deadband_half_width:
            return 'left'

        if pose.pose.position.y < 0.0:
            return 'right'
        return 'left'

    def select_detection_by_color(self, detections_msg, target_color):
        candidates = []
        for detection in detections_msg.detections:
            if detection.score < self.min_score:
                continue
            color = self.detection_color(detection)
            if color == target_color:
                candidates.append(detection)

        if not candidates:
            return None
        return max(candidates, key=lambda detection: detection.score)

    def detection_color(self, detection):
        class_name = detection.class_name.strip().lower()
        for color in ('red', 'blue', 'green'):
            if color in class_name:
                return color
        class_names = {name.strip().lower() for name in self.class_names}
        if class_names and not any(name in class_name for name in class_names):
            return None
        return self.classify_detection_color(detection)

    def classify_detection_color(self, detection):
        image = self.latest_color_image
        if image is None:
            return None

        height, width = image.shape[:2]
        cx = float(detection.bbox.center.position.x)
        cy = float(detection.bbox.center.position.y)
        bw = max(1.0, float(detection.bbox.size.x))
        bh = max(1.0, float(detection.bbox.size.y))

        # Use the central/lower body area to avoid white caps and table background.
        x_min = int(max(0, round(cx - bw * 0.25)))
        x_max = int(min(width, round(cx + bw * 0.25)))
        y_min = int(max(0, round(cy - bh * 0.10)))
        y_max = int(min(height, round(cy + bh * 0.35)))
        if x_max <= x_min or y_max <= y_min:
            return None

        patch = image[y_min:y_max, x_min:x_max]
        if patch.size == 0:
            return None

        bgr = np.mean(patch.reshape(-1, 3), axis=0)
        blue, green, red = [float(value) for value in bgr]
        channels = {'blue': blue, 'green': green, 'red': red}
        color = max(channels, key=channels.get)

        sorted_values = sorted(channels.values(), reverse=True)
        if sorted_values[0] < sorted_values[1] * 1.15:
            return None
        return color

    def make_color_place_pose(self, color, stamp):
        pose = self.make_place_pose(stamp)
        place_position = self.color_place_positions[color]
        pose.pose.position.x = float(place_position[0])
        pose.pose.position.y = float(place_position[1])
        pose.pose.position.z = float(place_position[2])
        return pose

    def execute_color_sequence(self):
        with self.execution_lock:
            success = False
            moved_colors = []
            returned_home = False
            self.execution_step = 'starting color sequence'
            try:
                for color in self.color_order:
                    capture = self.process_color_detection(color, log_detection=True)
                    if capture is None:
                        self.get_logger().warn(f'Skipping {color}; no current detection')
                        continue
                    object_pose = capture['pose']
                    arm = capture['arm']
                    place_pose = self.make_color_place_pose(color, object_pose.header.stamp)
                    if not self.execute_single_pick_place(color, object_pose, place_pose, arm):
                        if self.cancel_event.is_set():
                            return
                        self.get_logger().error(
                            f'{color} pick/place failed; continuing with next detected bottle'
                        )
                        continue
                    moved_colors.append(color)
                    returned_home = False
                    if self.return_home_between_places:
                        self.execution_step = f'return home after {color}'
                        self.get_logger().info(f'Returning both arms home after {color}')
                        if not self.move_home():
                            self.get_logger().error(f'Failed to return home after {color}')
                            if self.cancel_event.is_set():
                                return
                        else:
                            returned_home = True
                success = True
                if moved_colors:
                    self.get_logger().info(
                        f'Color bottle to {self.place_target_name} sequence finished for: '
                        + ', '.join(moved_colors)
                    )
                else:
                    self.get_logger().warn(
                        f'Color bottle to {self.place_target_name} sequence finished with no moves'
                    )
            finally:
                if self.cancel_event.is_set():
                    self.get_logger().warn('Pick/place cancelled; skipping home return')
                elif self.return_home_after_place and not returned_home:
                    self.execution_step = 'return home'
                    self.get_logger().info('Returning to table initial pose')
                    if not self.move_home():
                        self.get_logger().error('Failed to return to table initial pose')
                    elif not success:
                        self.get_logger().info('Returned home after interrupted sequence')
                self.execution_step = 'idle'

    def execute_single_pick_place(self, color, object_pose, place_pose, arm):
        grasp_pose = self.make_grasp_pose(object_pose)
        above_grasp_pose = self.make_above_pose(grasp_pose)
        above_place_pose = self.make_above_pose(place_pose)

        steps = [
            (f'open {arm} gripper for {color}', lambda: self.move_gripper_for_arm(arm, self.gripper_open_position)),
            (f'move {arm} arm above {color} bottle', lambda: self.move_to_pose_for_arm(arm, above_grasp_pose)),
            (f'move {arm} arm down to {color} bottle', lambda: self.move_to_pose_for_arm(arm, grasp_pose)),
            (f'close {arm} gripper on {color} bottle', lambda: self.move_gripper_for_arm(arm, self.gripper_closed_position)),
            (f'lift {color} bottle with {arm} arm', lambda: self.move_to_pose_for_arm(arm, above_grasp_pose)),
            (
                f'move {arm} arm above {color} {self.place_target_name}',
                lambda: self.move_to_pose_for_arm(arm, above_place_pose),
            ),
            (
                f'move {arm} arm down to {color} {self.place_target_name}',
                lambda: self.move_to_pose_for_arm(arm, place_pose),
            ),
            (
                f'open {arm} gripper at {color} {self.place_target_name}',
                lambda: self.move_gripper_for_arm(arm, self.gripper_open_position),
            ),
            (
                f'retreat {arm} arm above {color} {self.place_target_name}',
                lambda: self.move_to_pose_for_arm(arm, above_place_pose),
            ),
        ]

        for name, command in steps:
            if self.cancel_event.is_set():
                self.get_logger().warn(f'Pick/place cancelled before {name}')
                return False
            self.execution_step = name
            self.get_logger().info(f'Executing {name}')
            if not command():
                self.get_logger().error(f'Pick/place stopped at {name}')
                return False
        return True

    def move_to_pose_for_arm(self, arm, pose):
        if arm == 'right':
            return self.publish_movel(self.right_movel_pub, self.right_movel_topic, pose)
        return self.move_to_pose(pose)

    def move_gripper_for_arm(self, arm, position):
        if arm == 'right':
            return self.move_right_gripper(position)
        return self.move_gripper(position)

    def move_home(self):
        left_home_pose = self.make_pose_from_values(self.left_home_pose_values)
        right_home_pose = self.make_pose_from_values(self.right_home_pose_values)

        if not self.publish_movel(self.right_movel_pub, self.right_movel_topic, right_home_pose):
            return False
        if not self.publish_movel(self.movel_pub, self.movel_topic, left_home_pose):
            return False
        if not self.move_right_gripper(self.gripper_open_position):
            return False
        return self.move_gripper(self.gripper_open_position)

    def move_right_gripper(self, position):
        joint_names = [str(name) for name in self.right_home_joint_names]
        positions = self.get_current_joint_positions(joint_names)
        if positions is None:
            return False
        if self.right_gripper_joint not in joint_names:
            self.get_logger().error(f'{self.right_gripper_joint} is not in right_home_joint_names')
            return False

        positions[joint_names.index(self.right_gripper_joint)] = float(position)
        self.get_logger().info(f'Gripper target {self.right_gripper_joint}: {float(position):.3f}')
        if not self.publish_joint_positions_to(
            self.right_arm_joint_trajectory_pub,
            self.right_arm_joint_trajectory_topic,
            joint_names,
            positions,
            self.gripper_duration,
        ):
            return False
        return self.cancelable_sleep(self.gripper_duration + self.gripper_settle_time)

    def publish_joint_positions_to(self, publisher, topic_name, joint_names, positions, duration):
        if len(joint_names) != len(positions):
            self.get_logger().error('joint_names and positions length mismatch')
            return False
        if not self.wait_for_joint_trajectory_subscriber_on(publisher):
            self.get_logger().error(f'No joint trajectory subscriber on {topic_name}')
            return False

        trajectory = JointTrajectory()
        trajectory.header.stamp.sec = 0
        trajectory.header.stamp.nanosec = 0
        trajectory.joint_names = [str(name) for name in joint_names]

        point = JointTrajectoryPoint()
        point.positions = [float(position) for position in positions]
        point.time_from_start = self.seconds_to_duration(duration)
        trajectory.points.append(point)

        self.get_logger().info(
            f'Publishing joint trajectory to {topic_name}: '
            + ', '.join(
                f'{name}={position:.3f}'
                for name, position in zip(trajectory.joint_names, point.positions)
            )
        )
        publisher.publish(trajectory)
        return True

    def wait_for_joint_trajectory_subscriber_on(self, publisher):
        timeout_at = time.monotonic() + self.joint_trajectory_subscriber_timeout
        while time.monotonic() < timeout_at:
            if self.cancel_event.is_set():
                return False
            if publisher.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        return publisher.get_subscription_count() > 0

    def get_color_order(self):
        value = self.get_parameter('color_order').value
        if isinstance(value, str):
            colors = [color.strip() for color in value.split(',') if color.strip()]
        else:
            colors = [str(color) for color in value if str(color)]

        valid_colors = {'red', 'blue', 'green'}
        for color in colors:
            if color not in valid_colors:
                raise ValueError(f'unsupported color "{color}" in color_order')
        return colors


def main(args=None):
    rclpy.init(args=args)
    node = ColorBottlePickPlace()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
