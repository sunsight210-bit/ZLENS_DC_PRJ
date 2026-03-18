#!/usr/bin/env python3
"""UART 监听器：持续监听串口，每 3 秒发一帧 handshake，记录所有收发数据
用法: python3 scripts/uart_monitor.py /dev/ttyUSB0
按 Ctrl+C 退出
"""
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

print(f"=== UART Monitor: {port} @ {baud} ===")
print("Every 3s sends a handshake frame. Ctrl+C to quit.\n")

ser = serial.Serial(port, baud, timeout=0.5)
ser.reset_input_buffer()

# Handshake frame
header = bytes([0xA5, 0x01, 0x12, 0x34])
crc = crc16_modbus(header)
frame = header + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

last_tx = 0
count = 0

try:
    while True:
        now = time.time()

        # Send every 3 seconds
        if now - last_tx >= 3.0:
            count += 1
            ser.write(frame)
            ser.flush()
            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] TX #{count}: {frame.hex(' ')}")
            last_tx = now

        # Read any available data
        if ser.in_waiting > 0:
            rx = ser.read(ser.in_waiting)
            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] RX ({len(rx)}): {rx.hex(' ')}")

        time.sleep(0.05)

except KeyboardInterrupt:
    print(f"\nDone. Sent {count} frames.")

ser.close()
