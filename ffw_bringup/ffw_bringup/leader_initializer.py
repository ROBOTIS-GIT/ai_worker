#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Sungho Woo, Woojin Wie, Wonho Yun

from pathlib import Path


def detect_leader_port():
    candidates = {}
    for path in sorted(Path('/dev/serial/by-id').glob('*')):
        if path.exists() and any(name in path.name.upper() for name in (
            'STM32_VIRTUAL_COMPORT', 'ROBOTIS_AVATAR_CONTROLLER'
        )):
            candidates.setdefault(path.resolve(), path)
    if not candidates:
        raise RuntimeError('No leader port found. Connect the leader.')
    if len(candidates) > 1:
        found = ', '.join(str(path) for path in candidates.values())
        raise RuntimeError(
            f'Multiple leader ports found ({len(candidates)}). Connect only one leader.\n'
            f'Detected ports: {found}'
        )
    path = next(iter(candidates.values()))
    if 'STM32_VIRTUAL_COMPORT' in path.name.upper():
        return str(path)

    port = None
    try:
        from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

        port = PortHandler(str(path))
        packet = PacketHandler(2.0)
        if not port.openPort():
            raise RuntimeError('ROBOTIS Avatar Controller: could not open port')
        if not port.setBaudRate(4_000_000):
            raise RuntimeError('ROBOTIS Avatar Controller: could not set baud rate to 4000000')
        data, result, error = packet.readTxRx(port, 200, 10001, 12)
        if result != COMM_SUCCESS:
            raise RuntimeError(f'ROBOTIS Avatar Controller: {packet.getTxRxResult(result)}')
        if error:
            raise RuntimeError(f'ROBOTIS Avatar Controller: {packet.getRxPacketError(error)}')
        if len(data) != 12:
            raise RuntimeError(
                f'ROBOTIS Avatar Controller: expected 12 name bytes, received {len(data)}'
            )
        name = bytes(data).split(b'\x00', 1)[0].decode('ascii')
        if name != 'LEADER_A2':
            raise RuntimeError(f'ROBOTIS Avatar Controller: expected LEADER_A2, received {name!r}')
    except Exception as exc:
        raise RuntimeError(
            f'ROBOTIS Avatar Controller identification failed on {path}: {exc}'
        ) from exc
    finally:
        if port is not None and port.is_open:
            port.closePort()
    return str(path)


def detect_foot_switch():
    pass


def read_follower_urdf():
    pass


def read_follower_joint_states():
    pass
