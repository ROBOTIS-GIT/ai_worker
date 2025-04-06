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
# Authors: Wonho Yun

from pymodbus.client import ModbusSerialClient
import time

PORT = '/dev/left_hand'
BAUDRATE = 115200


def scan_ids():
    print("Modbus ID Scannings ...")

    client = ModbusSerialClient(port=PORT, baudrate=BAUDRATE, timeout=0.2)
    if not client.connect():
        print(f"Serial port {PORT} connection failed")
        return

    found_ids = []

    for slave_id in range(1, 10):
        try:
            result = client.read_holding_registers(address=0x0006, count=1, slave=slave_id)
            if not result.isError():
                found_ids.append(slave_id)
        except:
            pass
        time.sleep(0.01)

    client.close()

    if found_ids:
        print(f"\nAvailable ID List: {found_ids}")
    else:
        print("No connected devices found.")

if __name__ == '__main__':
    scan_ids()
