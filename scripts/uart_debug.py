#!/usr/bin/env python3
"""最小化 UART 调试：发送 handshake 帧，打印原始收发数据"""
import sys
import time
import serial

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

print(f"Opening {port} @ {baud}")
ser = serial.Serial(port, baud, timeout=3)
ser.reset_input_buffer()
time.sleep(0.1)

# Build handshake frame: A5 01 12 34 [CRC16_LE]
header = bytes([0xA5, 0x01, 0x12, 0x34])
crc = crc16_modbus(header)
frame = header + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
print(f"TX ({len(frame)} bytes): {frame.hex(' ')}")

ser.write(frame)
ser.flush()

# Wait and read whatever comes back
time.sleep(2)
rx = ser.read(ser.in_waiting or 64)
print(f"RX ({len(rx)} bytes): {rx.hex(' ') if rx else '(empty)'}")

if len(rx) >= 6:
    print(f"  Header: 0x{rx[0]:02X}")
    print(f"  Cmd:    0x{rx[1]:02X}")
    print(f"  Param:  0x{rx[2]:02X}{rx[3]:02X}")
    print(f"  CRC:    0x{rx[5]:02X}{rx[4]:02X}")
    crc_check = crc16_modbus(rx[:4])
    print(f"  CRC calc: 0x{crc_check:04X} {'OK' if crc_check == (rx[4] | rx[5]<<8) else 'MISMATCH'}")

ser.close()
