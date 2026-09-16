"""Run with python3; no cameras or ROS runtime required."""
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffw_bringup import camera_serial_configurator as camera


def device(serial, model, port):
    return dict(serial=serial, name=model, usb_port=port)


devices = [device('right', 'D405', '1-4.6-8'),
           device('head', 'D455', '2-3.1-4'),
           device('left', 'D405', '1-4.5-7')]
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory) / 'rs_serial.yaml'
    original = {'default': {'camera1_serial': ''},
                'hosts': {'other': {'camera1_serial': "'keep'"}}}
    path.write_text(yaml.safe_dump(original))
    with patch.object(camera, 'discover_realsense_devices', return_value=devices), \
            patch('builtins.input', side_effect=AssertionError('Must not prompt')):
        camera.configure(path, 'robot', 'realsense')
    saved = yaml.safe_load(path.read_text())
    assert saved['hosts']['robot'] == dict(camera1_serial="'left'", camera2_serial="'right'", camera3_serial="'head'")
    assert saved['default'] == original['default']
    assert saved['hosts']['other'] == original['hosts']['other']
    before = path.read_bytes()
    with patch.object(camera, 'discover_realsense_devices', side_effect=AssertionError('Must not discover')):
        camera.configure(path, 'robot', 'realsense')
    assert path.read_bytes() == before
    with patch.object(camera, 'discover_realsense_devices', return_value=devices[:1]):
        try:
            camera.configure(path, 'missing', 'zed')
        except ValueError:
            pass
        else:
            raise AssertionError('Missing camera accepted')
    assert path.read_bytes() == before

assert camera.assign_cameras(devices, 2, {'camera1_serial': "'right'"}) == {
    'camera1_serial': "'right'", 'camera2_serial': "'left'"}
assert camera.serial_value("''") == ''
print('Camera registration, no-discovery exit, no-prompt and missing-device checks: PASS')
