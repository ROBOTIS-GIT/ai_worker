"""Run in a sourced ROS workspace; constructs actions without starting cameras."""
from pathlib import Path
import runpy
import socket
import tempfile
from types import SimpleNamespace
from unittest.mock import patch

from ffw_bringup import camera_serial_configurator

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
import yaml

parent = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'launch/camera.launch.py'))
handler = next(a.event_handler for a in parent['generate_launch_description']().entities
               if isinstance(a, RegisterEventHandler))
callback = handler.handle
context = LaunchContext()
assert callback(SimpleNamespace(returncode=1), context) is None
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory) / 'serials.yaml'
    path.write_text(yaml.safe_dump({'hosts': {socket.gethostname().split('.')[0]: {
        'camera1_serial': "'111'", 'camera2_serial': "'222'", 'camera3_serial': "'333'"}}}))
    for head, expected in (('zed', 2), ('realsense', 3)):
        context = LaunchContext()
        context.launch_configurations.update(head_camera_type=head)
        parent_actions = callback(SimpleNamespace(returncode=0), context)
        assert len(parent_actions) == 2 and isinstance(parent_actions[1], TimerAction)
        assert parent_actions[0].condition.evaluate(context) == (head == 'zed')
        with patch.object(camera_serial_configurator, 'DEFAULT_SERIALS_PATH', str(path)):
            module = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'launch/camera_realsense.launch.py'))
        actions = module['generate_launch_description']().entities
        for action in actions:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        cameras = [a for a in actions if isinstance(a, OpaqueFunction)
                   and (a.condition is None or a.condition.evaluate(context))]
        assert len(cameras) == expected
        for i in (1, 2, 3):
            assert context.launch_configurations[f'serial_no{i}'] == f"'{str(i) * 3}'"
print('Parent registration gate, RealSense serial defaults: PASS')
