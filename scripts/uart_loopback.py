#!/usr/bin/env python3
"""USB 串口模块回环测试 + 端口诊断
用法：
  1. 将 USB 串口模块的 TX 和 RX 短接（不连板子）
  2. python3 scripts/uart_loopback.py /dev/ttyUSB0
"""
import sys
import time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"

print(f"=== 串口诊断: {port} ===\n")

ser = serial.Serial(port, 115200, timeout=2)
print(f"Port settings: {ser.baudrate} {ser.bytesize}{ser.parity}{ser.stopbits}")
print(f"  RTS/CTS: {ser.rtscts}, DSR/DTR: {ser.dsrdtr}")
print(f"  CTS={ser.cts}, DSR={ser.dsr}, CD={ser.cd}, RI={ser.ri}")
print()

# Loopback test
ser.reset_input_buffer()
test_data = b'\xA5\x01\x12\x34\x7E\x5F'
ser.write(test_data)
ser.flush()
time.sleep(0.5)
rx = ser.read(ser.in_waiting or 64)

if rx == test_data:
    print(f"[PASS] Loopback OK: TX={test_data.hex(' ')} RX={rx.hex(' ')}")
    print("  -> USB 串口模块正常，请检查与板子的接线")
elif len(rx) > 0:
    print(f"[WARN] Partial/garbled: TX={test_data.hex(' ')} RX={rx.hex(' ')}")
    print("  -> 可能波特率不匹配或接线干扰")
else:
    print(f"[INFO] Loopback RX empty (TX/RX 未短接 或 模块有问题)")
    print("  -> 请短接 TX-RX 引脚后重试，确认模块本身正常")

ser.close()
