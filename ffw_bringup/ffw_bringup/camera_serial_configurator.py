# Copyright 2023 Intel Corporation. All Rights Reserved.
# Copyright 2026 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0
"""Register RealSense serials once; discovery adapted from main camera_realsense.launch.py."""
import argparse
import os
from pathlib import Path
import re
import socket
import subprocess
import tempfile

import yaml

DEFAULT_SERIALS_PATH = '~/ros2_ws/src/ai_worker/ffw_bringup/config/common/rs_serial.yaml'


def get_device_info(device, camera_info):
    try:
        if device.supports(camera_info):
            return device.get_info(camera_info)
    except RuntimeError:
        pass
    return ''


def normalize_usb_port(physical_port):
    usb_ports = re.findall(r'(?<!\w)(\d+(?:-\d+(?:\.\d+)*)+)(?![\w.])', physical_port or '')
    return usb_ports[-1] if usb_ports else physical_port


def devices_from_realsense_context():
    try:
        import pyrealsense2 as rs
    except ImportError:
        return []

    try:
        context = rs.context()
    except RuntimeError:
        return []

    devices = []
    for device in context.query_devices():
        serial = get_device_info(device, rs.camera_info.serial_number)
        if not serial:
            continue
        physical_port = get_device_info(device, rs.camera_info.physical_port)
        devices.append({
            'serial': serial,
            'name': get_device_info(device, rs.camera_info.name),
            'product_line': get_device_info(device, rs.camera_info.product_line),
            'usb_port': normalize_usb_port(physical_port),
            'physical_port': physical_port,
        })
    return devices


def device_from_rs_enumerate_block(block):
    serial_match = re.search(r'Serial Number\s*:\s*([0-9]+)', block)
    if not serial_match:
        return {}
    physical_port_match = re.search(r'Physical Port\s*:\s*(.+)', block)
    name_match = re.search(r'Name\s*:\s*(.+)', block)
    product_line_match = re.search(r'Product Line\s*:\s*(.+)', block)
    physical_port = physical_port_match.group(1).strip() if physical_port_match else ''
    return {
        'serial': serial_match.group(1),
        'name': name_match.group(1).strip() if name_match else '',
        'product_line': product_line_match.group(1).strip() if product_line_match else '',
        'usb_port': normalize_usb_port(physical_port),
        'physical_port': physical_port,
    }


def devices_from_rs_enumerate_devices():
    try:
        result = subprocess.run(
            ['rs-enumerate-devices'],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return []

    if result.returncode != 0:
        return []

    devices = []
    for block in re.split(r'\n\s*\n', result.stdout):
        device = device_from_rs_enumerate_block(block)
        if device:
            devices.append(device)
    return devices


def discover_realsense_devices():
    seen = set()
    devices = []
    for device in devices_from_realsense_context() or devices_from_rs_enumerate_devices():
        serial = device.get('serial')
        if serial and serial not in seen:
            seen.add(serial)
            devices.append(device)
    return devices


def serial_value(value):
    return str(value or '').strip().strip("\"'")


def assign_cameras(devices, count, existing):
    """Keep existing roles, then prefer main's known ports, then propose port order."""
    keys = [f'camera{i}_serial' for i in range(1, count + 1)]
    assigned = {key: serial_value(existing.get(key)) for key in keys}
    by_serial = {d['serial']: d for d in devices}
    used = [value for value in assigned.values() if value]
    if len(set(used)) != len(used) or any(value not in by_serial for value in used):
        raise ValueError('Existing partial mapping has duplicate or disconnected serials.')
    ports = ['1-4.5-7', '1-4.6-8', '2-3.1-4']
    for index, key in enumerate(keys):
        model = 'D455' if index == 2 else 'D405'
        candidates = sorted(
            [d for d in devices if d['serial'] not in assigned.values()
             and model in d.get('name', '').upper()],
            key=lambda d: (d.get('usb_port') != ports[index],
                           d.get('usb_port') or d['serial']))
        if not assigned[key]:
            if not candidates:
                raise ValueError(f'No available {model} for {key}; nothing saved.')
            assigned[key] = candidates[0]['serial']
        if model not in by_serial[assigned[key]].get('name', '').upper():
            raise ValueError(f'{key} requires {model}; nothing saved.')
    return {key: f"'{value}'" for key, value in assigned.items()}


def configure(path, hostname, head_camera_type):
    path = Path(path).expanduser().resolve(strict=True)
    original = path.read_text()
    config = yaml.safe_load(original)
    if not isinstance(config, dict) or not isinstance(config.get('hosts', {}), dict):
        raise ValueError('Expected a YAML mapping with a hosts mapping.')
    hosts = config.setdefault('hosts', {})
    existing = hosts.get(hostname) or {}
    if not isinstance(existing, dict):
        raise ValueError(f'Invalid host entry: {hostname}')
    count = 3 if head_camera_type == 'realsense' else 2
    if all(serial_value(existing.get(f'camera{i}_serial')) for i in range(1, count + 1)):
        print(f'{hostname}: serials already registered; skipping discovery.')
        return

    devices = discover_realsense_devices()
    mapping = assign_cameras(devices, count, existing)
    print(f'Host: {hostname}\nSave to: {path}')
    for i, role in enumerate(('left', 'right', 'head')[:count], 1):
        serial = serial_value(mapping[f'camera{i}_serial'])
        device = next(d for d in devices if d['serial'] == serial)
        print(f"{role}: {serial}  {device['name']}  USB {device['usb_port']}")

    hosts[hostname] = {**existing, **mapping}
    if path.read_text() != original:
        raise ValueError('YAML changed during discovery; retry instead of overwriting it.')
    # Replace the resolved source file, not an install symlink.
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            yaml.safe_dump(config, stream, sort_keys=False)
        temporary.chmod(path.stat().st_mode & 0o777)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    print(f'Saved {hostname} to {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serials-path', default=DEFAULT_SERIALS_PATH)
    parser.add_argument('--head-camera-type', choices=('zed', 'realsense'), default='zed')
    args = parser.parse_args()
    try:
        configure(args.serials_path, socket.gethostname().split('.')[0], args.head_camera_type)
    except (OSError, ValueError, RuntimeError, yaml.YAMLError) as error:
        parser.exit(1, f'{error}\n')


if __name__ == '__main__':
    main()
