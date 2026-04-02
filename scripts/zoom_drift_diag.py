#!/usr/bin/env python3
"""
Zoom drift diagnostic: repeat 1X<->2X cycles and collect SWO position data.

Usage:
    # Terminal 1: start SWO capture (路径B, sleep=120000)
    # Terminal 2: python3 scripts/zoom_drift_diag.py --port /dev/ttyUSB0
"""
import argparse
import struct
import time
import serial


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def build_frame(cmd: int, param: int = 0) -> bytes:
    hdr = struct.pack('>BBH', 0xA5, cmd, param)
    return hdr + struct.pack('<H', crc16(hdr))


def read_frames(ser: serial.Serial, timeout: float = 15.0):
    """Generator: yield (cmd, param) frames until timeout."""
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            buf.extend(chunk)
        while len(buf) >= 6:
            if buf[0] != 0xA5:
                buf.pop(0)
                continue
            frame = bytes(buf[:6])
            crc_calc = crc16(frame[:4])
            crc_recv = struct.unpack('<H', frame[4:6])[0]
            if crc_calc != crc_recv:
                buf.pop(0)
                continue
            cmd = frame[1]
            param = struct.unpack('>H', frame[2:4])[0]
            del buf[:6]
            yield (cmd, param)


def send_and_wait(ser: serial.Serial, cmd: int, param: int,
                  expected_cmd: int, expected_param: int = None,
                  timeout: float = 15.0) -> tuple:
    """Send command, skip echo, wait for expected response. Returns (cmd, param)."""
    ser.write(build_frame(cmd, param))
    echo_skipped = False
    for (rcmd, rparam) in read_frames(ser, timeout):
        # Skip echo: same cmd and param as sent
        if not echo_skipped and rcmd == cmd and rparam == param:
            echo_skipped = True
            continue
        if rcmd == expected_cmd:
            if expected_param is not None and rparam != expected_param:
                print(f"  [RSP] cmd=0x{rcmd:02X} param=0x{rparam:04X} (unexpected param)")
                continue
            return (rcmd, rparam)
        print(f"  [RSP] cmd=0x{rcmd:02X} param=0x{rparam:04X}")
    raise TimeoutError(f"Timeout waiting for 0x{expected_cmd:02X}")


CMD_HOMING = 0x01
CMD_SET_ZOOM = 0x10
RSP_HOMING_DONE = 0x01
RSP_ARRIVED = 0x02
RSP_ZOOM = 0x10

ZOOM_1X = 10   # 1.0x
ZOOM_2X = 20   # 2.0x
NUM_CYCLES = 10


def main():
    parser = argparse.ArgumentParser(description='Zoom drift diagnostic')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--cycles', type=int, default=NUM_CYCLES)
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    ser.reset_input_buffer()
    print(f"=== Zoom Drift Diagnostic ===")
    print(f"Port: {args.port}, Cycles: {args.cycles}")

    # Step 1: Homing
    print("\n[1] Sending HOMING...")
    try:
        _, param = send_and_wait(ser, CMD_HOMING, 0,
                                 expected_cmd=RSP_HOMING_DONE,
                                 expected_param=0x000F,
                                 timeout=30.0)
        print(f"    HOMING_DONE param=0x{param:04X}")
    except TimeoutError as e:
        print(f"    FAIL: {e}")
        ser.close()
        return

    time.sleep(1.0)

    # Step 2: Cycle 1X <-> 2X
    print(f"\n[2] Starting {args.cycles} cycles: 1X -> 2X -> 1X ...")
    for i in range(1, args.cycles + 1):
        # Move to 2X
        try:
            send_and_wait(ser, CMD_SET_ZOOM, ZOOM_2X,
                          expected_cmd=RSP_ARRIVED, timeout=15.0)
            print(f"  Cycle {i:2d}: 1X->2X ARRIVED")
        except TimeoutError as e:
            print(f"  Cycle {i:2d}: 1X->2X FAIL: {e}")
            break

        time.sleep(0.5)

        # Move to 1X
        try:
            send_and_wait(ser, CMD_SET_ZOOM, ZOOM_1X,
                          expected_cmd=RSP_ARRIVED, timeout=15.0)
            print(f"  Cycle {i:2d}: 2X->1X ARRIVED")
        except TimeoutError as e:
            print(f"  Cycle {i:2d}: 2X->1X FAIL: {e}")
            break

        time.sleep(0.5)

    print(f"\n[3] Done. Check SWO log for [MOVE] entries.")
    print("    Run: python3 scripts/parse_swo.py build/swo/swo.bin")
    print("    Then grep for [MOVE] to see position drift.")
    ser.close()


if __name__ == '__main__':
    main()
