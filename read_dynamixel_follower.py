#!/usr/bin/env python3
"""Read Current Limit and Homing Offset from X-series Dynamixels
on /dev/follower at 4Mbps using raw Protocol 2.0 packets via pyserial.

No dynamixel-sdk required.
"""
import serial
import struct
import sys
import time

PORT = "/dev/follower"
BAUD = 4_000_000

ADDR_CURRENT_LIMIT = 38
LEN_CURRENT_LIMIT = 2

ADDR_HOMING_OFFSET = 52
LEN_HOMING_OFFSET = 4

ADDR_GEAR_RATIO_NUM = 96
LEN_GEAR_RATIO_NUM = 4

ADDR_GEAR_RATIO_DEN = 100
LEN_GEAR_RATIO_DEN = 4

# Robotis CRC-16/IBM table (poly 0x8005)
CRC_TABLE = [
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202,
]

def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        idx = ((crc >> 8) ^ b) & 0xFF
        crc = ((crc << 8) ^ CRC_TABLE[idx]) & 0xFFFF
    return crc

def build_read(dxl_id: int, addr: int, length: int) -> bytes:
    # Header(4) + ID(1) + Length(2) + Instr(1) + Params(4) + CRC(2)
    pkt = bytes([0xFF, 0xFF, 0xFD, 0x00, dxl_id])
    body_len = 1 + 4 + 2  # instr + params + crc
    pkt += struct.pack("<H", body_len)
    pkt += bytes([0x02])  # READ
    pkt += struct.pack("<H", addr)
    pkt += struct.pack("<H", length)
    pkt += struct.pack("<H", crc16(pkt))
    return pkt

def parse_status(buf: bytes, expected_data_len: int):
    """Return (error_byte, data_bytes) or raise."""
    if len(buf) < 11 + expected_data_len:
        raise ValueError(f"short response ({len(buf)} bytes): {buf.hex()}")
    if buf[:4] != b"\xFF\xFF\xFD\x00":
        raise ValueError(f"bad header: {buf[:4].hex()}")
    pkt_id = buf[4]
    body_len = struct.unpack("<H", buf[5:7])[0]
    instr = buf[7]
    if instr != 0x55:
        raise ValueError(f"not a status packet (instr=0x{instr:02X})")
    err = buf[8]
    data = buf[9:9 + expected_data_len]
    return pkt_id, err, data

def read_register(ser, dxl_id, addr, length, timeout=0.05):
    pkt = build_read(dxl_id, addr, length)
    ser.reset_input_buffer()
    ser.write(pkt)
    ser.flush()
    time.sleep(0.005)
    # Expected status length: 4 + 1 + 2 + 1 + 1 + length + 2
    expected = 11 + length
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline and len(buf) < expected:
        chunk = ser.read(expected - len(buf))
        if not chunk:
            time.sleep(0.001)
            continue
        buf += chunk
    if len(buf) < expected:
        raise TimeoutError(f"id={dxl_id} no/short reply ({len(buf)}B)")
    _, err, data = parse_status(buf, length)
    if err & 0x7F:
        raise RuntimeError(f"id={dxl_id} hw err 0x{err:02X}")
    return int.from_bytes(data, "little")

def to_signed(value: int, byte_length: int) -> int:
    """Convert unsigned int to signed (two's complement)."""
    bits = byte_length * 8
    if value >= (1 << (bits - 1)):
        value -= (1 << bits)
    return value

def main():
    ids = [1, 2, 3, 4, 5, 6, 7, 8, 31, 32, 33, 34, 35, 36, 37, 38, 61, 62, 81]
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    print(f"Port: {PORT}  Baud: {BAUD}")
    print(f"Reading Current Limit (addr {ADDR_CURRENT_LIMIT}, {LEN_CURRENT_LIMIT}B), "
          f"Homing Offset (addr {ADDR_HOMING_OFFSET}, {LEN_HOMING_OFFSET}B), "
          f"Gear Ratio Num (addr {ADDR_GEAR_RATIO_NUM}, {LEN_GEAR_RATIO_NUM}B), "
          f"Gear Ratio Den (addr {ADDR_GEAR_RATIO_DEN}, {LEN_GEAR_RATIO_DEN}B) "
          f"from {len(ids)} IDs\n")
    print(f"{'ID':>4}  {'Current Limit':>14}  {'Homing Offset':>15}  "
          f"{'Gear Num':>12}  {'Gear Den':>12}  {'Ratio':>10}")
    print("-" * 80)
    for did in ids:
        # Current Limit
        try:
            cl = read_register(ser, did, ADDR_CURRENT_LIMIT, LEN_CURRENT_LIMIT)
            cl_str = f"{cl}"
        except Exception as e:
            cl_str = f"-- {e}"

        # Homing Offset (signed 4 bytes)
        try:
            ho_raw = read_register(ser, did, ADDR_HOMING_OFFSET, LEN_HOMING_OFFSET)
            ho = to_signed(ho_raw, LEN_HOMING_OFFSET)
            ho_str = f"{ho}"
        except Exception as e:
            ho_str = f"-- {e}"

        # Electronic Gear Ratio Numerator (unsigned 4 bytes)
        gn = None
        try:
            gn = read_register(ser, did, ADDR_GEAR_RATIO_NUM, LEN_GEAR_RATIO_NUM)
            gn_str = f"{gn}"
        except Exception as e:
            gn_str = f"-- {e}"

        # Electronic Gear Ratio Denominator (unsigned 4 bytes)
        gd = None
        try:
            gd = read_register(ser, did, ADDR_GEAR_RATIO_DEN, LEN_GEAR_RATIO_DEN)
            gd_str = f"{gd}"
        except Exception as e:
            gd_str = f"-- {e}"

        if gn is not None and gd is not None and gd != 0:
            ratio_str = f"{gn / gd:.4f}"
        else:
            ratio_str = "--"

        print(f"{did:>4}  {cl_str:>14}  {ho_str:>15}  "
              f"{gn_str:>12}  {gd_str:>12}  {ratio_str:>10}")
    ser.close()

if __name__ == "__main__":
    main()
