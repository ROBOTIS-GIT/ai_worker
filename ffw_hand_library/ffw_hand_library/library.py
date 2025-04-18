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

import serial


class InspireHand:

    def __init__(self, port, hand_id=1):
        self.hand_id = hand_id
        self.ser = serial.Serial(port, 115200, timeout=1)
        if not self.ser.is_open:
            self.ser.open()

    def data2bytes(self, data):
        return [0xFF, 0xFF] if data == -1 else [data & 0xFF, (data >> 8) & 0xFF]

    def num2str(self, num):
        return bytes([num & 0xFF])

    def checknum(self, data, length):
        return sum(data[2:length]) & 0xFF

    def send_and_receive(self, b, datanum, read_len):
        putdata = b''
        for i in range(1, datanum + 6):
            putdata += self.num2str(b[i-1])
        self.ser.write(putdata)
        res = self.ser.read(read_len)
        return res

    def set_6val(self, addr, values, label='set_6val'):
        if len(values) != 6 or any(v < -1 or v > 2000 for v in values):
            return
        datanum = 0x0F
        b = [0] * (datanum + 5)
        b[0] = 0xEB
        b[1] = 0x90
        b[2] = self.hand_id
        b[3] = datanum
        b[4] = 0x12
        b[5] = addr & 0xFF
        b[6] = (addr >> 8) & 0xFF
        b[7:19] = sum([self.data2bytes(v) for v in values], [])
        b[19] = self.checknum(b, datanum + 4)
        return self.send_and_receive(b, datanum, 9)

    def get_6val(self, addr, label='get_6val'):
        datanum = 0x04
        b = [0] * (datanum + 5)
        b[0] = 0xEB
        b[1] = 0x90
        b[2] = self.hand_id
        b[3] = datanum
        b[4] = 0x11
        b[5] = addr & 0xFF
        b[6] = (addr >> 8) & 0xFF
        b[7] = 0x0C
        b[8] = self.checknum(b, datanum + 4)
        res = self.send_and_receive(b, datanum, 20)
        values = []
        for i in range(6):
            low_byte = res[6 + i * 2] if len(res) > 6 + i * 2 else 0xFF
            high_byte = res[6 + i * 2 + 1] if len(res) > 6 + i * 2 + 1 else 0xFF
            values.append(-1 if (low_byte == 0xFF and high_byte == 0xFF)
                          else low_byte + (high_byte << 8))
        return values

    def setpos(self, *vals): return self.set_6val(0x05C2, vals, 'setpos')
    def setangle(self, *vals): return self.set_6val(0x05CE, vals, 'setangle')
    def setspeed(self, *vals): return self.set_6val(0x05F2, vals, 'setspeed')
    def setpower(self, *vals): return self.set_6val(0x05DA, vals, 'setpower')
    def setdefaultspeed(self, *vals): return self.set_6val(0x0408, vals, 'setdefaultspeed')
    def setdefaultpower(self, *vals): return self.set_6val(0x0414, vals, 'setdefaultpower')

    def get_setpos(self): return self.get_6val(0x05C2, 'get_setpos')
    def get_setangle(self): return self.get_6val(0x05CE, 'get_setangle')
    def get_actpos(self): return self.get_6val(0x05FE, 'get_actpos')
    def get_actangle(self): return self.get_6val(0x060A, 'get_actangle')
    def get_setpower(self): return self.get_6val(0x05DA, 'get_setpower')
    def get_actforce(self): return self.get_6val(0x062E, 'get_actforce')
    def get_current(self): return self.get_6val(0x063A, 'get_current')
    def get_error(self): return self.get_6val(0x0646, 'get_error')
    def get_status(self): return self.get_6val(0x064C, 'get_status')
    def get_temp(self): return self.get_6val(0x0652, 'get_temp')

    def set_clear_error(self): return self._set_simple(0x03EC, 'set_clear_error')
    def set_save_flash(self): return self._set_simple(0x03ED, 'set_save_flash')
    def gesture_force_clb(self): return self._set_simple(0x03F1, 'gesture_force_clb')

    def _set_simple(self, addr, label):
        datanum = 0x04
        b = [0] * (datanum + 5)
        b[0] = 0xEB
        b[1] = 0x90
        b[2] = self.hand_id
        b[3] = datanum
        b[4] = 0x12
        b[5] = addr & 0xFF
        b[6] = (addr >> 8) & 0xFF
        b[7] = 0x01
        b[8] = self.checknum(b, datanum + 4)
        readlen = 18 if label == 'set_save_flash' else 9
        return self.send_and_receive(b, datanum, readlen)
