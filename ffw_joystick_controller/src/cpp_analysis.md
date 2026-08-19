# Joystick Controller Topic Analysis

기준 파일:

```text
ffw_joystick_controller/src/joystick_controller.cpp
```

## 1. 요약

현재 `joystick_controller.cpp` 기준 토픽 입출력은 아래 구조다.

```text
subscribe:
  params_.joint_states_topic
  /leader/foot_switch/middle_pedal

publish:
  ~/sensorxel_joy_values
  sensor_joint_trajectory_topic_[sensor_name]
  /leader/joystick_controller/tact_trigger
  /leader/left_command
  /leader/right_command
  /cmd_vel
```

`/robot/head_leader/joint_states`, `/robot/lift_leader/joint_states` 발행 코드는 제거되었다.

## 2. 구독 토픽

| 토픽 | 메시지 타입 | 위치 | 용도 |
|---|---|---|---|
| `params_.joint_states_topic` | `sensor_msgs/msg/JointState` | `on_configure()` | leader의 현재 joint position을 읽는다. 기본 parameter 값은 `/joint_states`. |
| `/leader/foot_switch/middle_pedal` | `std_msgs/msg/Bool` | `on_configure()` | middle pedal 눌림 상태를 읽는다. tact switch 동작 분기에 사용된다. |

코드:

```cpp
joint_states_subscriber_ =
  get_node()->create_subscription<sensor_msgs::msg::JointState>(
    params_.joint_states_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&JoystickController::joint_states_callback, this, std::placeholders::_1));
```

```cpp
middle_pedal_sub_ =
  get_node()->create_subscription<std_msgs::msg::Bool>(
    "/leader/foot_switch/middle_pedal",
    10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      middle_pedal_held_ = msg->data;
    });
```

## 3. 발행 토픽

| 토픽 | 메시지 타입 | 위치 | 용도 |
|---|---|---|---|
| `~/sensorxel_joy_values` | `std_msgs/msg/Float64MultiArray` | `on_configure()` | 정규화된 joystick 값을 발행한다. |
| `sensor_joint_trajectory_topic_[sensor_name]` | `trajectory_msgs/msg/JointTrajectory` | `on_configure()` | 센서별 joint trajectory를 발행한다. 실제 토픽명은 parameter에서 온다. |
| `/leader/joystick_controller/tact_trigger` | `std_msgs/msg/String` | `on_configure()` | tact switch trigger 문자열을 발행한다. |
| `/leader/left_command` | `std_msgs/msg/UInt8` | `on_configure()` | left leader command를 발행한다. |
| `/leader/right_command` | `std_msgs/msg/UInt8` | `on_configure()` | right leader command를 발행한다. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `on_configure()` | base velocity command를 발행한다. |

## 4. Trajectory 토픽 이름을 가져오는 방식

센서 이름 뒤에 `"_joint_trajectory_topic"`을 붙여 parameter 이름을 만든다.

```cpp
std::string topic_param = sensor_name + "_joint_trajectory_topic";
```

예:

```text
sensor_name = "sensorxel_l_joy"
topic_param = "sensorxel_l_joy_joint_trajectory_topic"

sensor_name = "sensorxel_r_joy"
topic_param = "sensorxel_r_joy_joint_trajectory_topic"
```

그 parameter 값을 읽어서 map에 저장한다.

```cpp
sensor_joint_trajectory_topic_[sensor_name] =
  get_node()->get_parameter(topic_param).as_string();
```

그 값을 publisher 생성에 사용한다.

```cpp
sensor_joint_trajectory_publisher_[sensor_name] =
  get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    sensor_joint_trajectory_topic_[sensor_name],
    rclcpp::SystemDefaultsQoS());
```

## 5. Trajectory 발행 위치

`update()`에서 조건이 맞으면 `publish_joint_trajectory()`를 호출한다.

```cpp
if (!current_joint_states_.name.empty() && !controlled_joints.empty()) {
  ...
  publish_joint_trajectory(controlled_joints, positions, sensor_name);
}
```

실제 publish는 `publish_joint_trajectory()` 안에서 한다.

```cpp
auto joint_trajectory_publisher =
  sensor_joint_trajectory_publisher_[sensor_name];

if (joint_trajectory_publisher) {
  joint_trajectory_publisher->publish(trajectory_msg);
}
```

## 6. Trajectory 메시지 형태

```cpp
trajectory_msg.header.stamp = rclcpp::Time(0);
trajectory_msg.joint_names = controlled_joints;

point.time_from_start = rclcpp::Duration(0, 0);
point.positions = positions;
point.velocities.resize(positions.size(), 0.0);
point.accelerations.resize(positions.size(), 0.0);

trajectory_msg.points.push_back(point);
```

## 7. ROS 토픽이 아닌 joystick 입력

joystick ADC 값은 ROS topic으로 구독하지 않는다.  
ros2_control state interface에서 직접 읽는다.

| 입력 |
|---|
| `sensorxel_l_joy/JOYSTICK X VALUE` |
| `sensorxel_l_joy/JOYSTICK Y VALUE` |
| `sensorxel_l_joy/JOYSTICK TACT SWITCH` |
| `sensorxel_r_joy/JOYSTICK X VALUE` |
| `sensorxel_r_joy/JOYSTICK Y VALUE` |
| `sensorxel_r_joy/JOYSTICK TACT SWITCH` |

코드:

```cpp
auto opt_value =
  joint_state_interface_[j][sensor_idx].get().get_optional();
```

## 8. 더 이상 발행하지 않는 토픽

아래 토픽을 발행하던 `sensor_joint_state_stamped_publisher_` 경로는 삭제되었다.

```text
/robot/head_leader/joint_states
/robot/lift_leader/joint_states
```

