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
# Authors: Sungho Woo, Wonho Yun

import os
import subprocess


def main():
    print('')
    print('This script copies the udev rule to /etc/udev/rules.d/')
    print('to configure the U2D2 device for the ffw follower.')
    print('')

    try:
        pkg_prefix = subprocess.check_output(
            ['ros2', 'pkg', 'prefix', 'ffw_bringup'],
            universal_newlines=True
        ).strip()
    except subprocess.CalledProcessError:
        print('Error: Failed to find package prefix for ffw_bringup.')
        return

    src_path = os.path.join(pkg_prefix, 'share', 'ffw_bringup', 'ffw.rules')
    dst_path = '/etc/udev/rules.d/ffw.rules'

    print(f'Copying {src_path} to {dst_path}')
    try:
        subprocess.run(['sudo', 'cp', src_path, dst_path], check=True)
    except subprocess.CalledProcessError:
        print('Error: Failed to copy udev rules file.')
        return

    print('')
    print('Reloading udev rules...')
    try:
        subprocess.run(['sudo', 'udevadm', 'control', '--reload-rules'], check=True)
        subprocess.run(['sudo', 'udevadm', 'trigger'], check=True)
    except subprocess.CalledProcessError:
        print('Error: Failed to reload udev rules.')
        return

    print('')
    print('Done!')


if __name__ == '__main__':
    main()
