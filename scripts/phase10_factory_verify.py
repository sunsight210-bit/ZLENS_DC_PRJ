#!/usr/bin/env python3
"""
phase10_factory_verify.py — Factory mode calibration verification

Tests the full factory calibration flow:
    1. Enter factory mode (0xFA param=0xFAE5)
    2. Erase all zoom entries (0xF0 + magic)
    3. Set 5 zoom entries (0xF1)
    4. Exit factory mode (0xFA + magic) → saves to Flash
    5. Query zoom range to verify table loaded
    6. SET_ZOOM to calibrated entries → verify ARRIVED
    7. HOMING → verify still works after table change

Usage:
    python3 scripts/phase10_factory_verify.py [--port PORT] [--baud BAUD]
"""

import argparse
import struct
import sys
import time


# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------
FRAME_HEADER      = 0xA5
WORK_FRAME_SIZE   = 6
FACTORY_FRAME_SIZE = 8

# Work mode commands
CMD_HOMING         = 0x01
CMD_SET_ZOOM       = 0x10
CMD_QUERY_ZOOM     = 0x20
CMD_QUERY_RANGE    = 0x24
CMD_SWITCH_FACTORY = 0xFA

# Factory mode commands
FCMD_ERASE_ALL     = 0xF0
FCMD_SET_ENTRY     = 0xF1
FCMD_SWITCH_TO_WORK = 0xFA

# Magic keys
FACTORY_MAGIC_PARAM = 0xFAE5
FACTORY_MAGIC_HIGH  = 0xFAE5
FACTORY_MAGIC_LOW   = 0x00FA

# Response commands
RSP_HOMING_DONE = 0x01
RSP_ARRIVED     = 0x02
RSP_ZOOM        = 0x10
RSP_MIN_ZOOM    = 0x14
RSP_MAX_ZOOM    = 0x15

# Test calibration table (5 entries)
TEST_ZOOM_TABLE = [
    (6,      0),     # 0.6x = 0.00 deg
    (10,  7200),     # 1.0x = 72.00 deg
    (20, 14800),     # 2.0x = 148.00 deg
    (40, 25750),     # 4.0x = 257.50 deg
    (70, 34700),     # 7.0x = 347.00 deg
]


# ---------------------------------------------------------------------------
# CRC16 Modbus
# ---------------------------------------------------------------------------
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_work_frame(cmd: int, param: int) -> bytes:
    hdr = struct.pack('>BBH', FRAME_HEADER, cmd, param)
    crc = crc16_modbus(hdr)
    return hdr + struct.pack('<H', crc)


def build_factory_frame(cmd: int, param_h: int, param_l: int) -> bytes:
    hdr = struct.pack('>BBHH', FRAME_HEADER, cmd, param_h, param_l)
    crc = crc16_modbus(hdr)
    return hdr + struct.pack('<H', crc)


def parse_work_response(data: bytes):
    """Parse a 6-byte work frame. Returns (cmd, param) or None."""
    if len(data) < WORK_FRAME_SIZE:
        return None
    if data[0] != FRAME_HEADER:
        return None
    crc_calc = crc16_modbus(data[:4])
    crc_recv = data[4] | (data[5] << 8)
    if crc_calc != crc_recv:
        return None
    cmd   = data[1]
    param = (data[2] << 8) | data[3]
    return (cmd, param)


# ---------------------------------------------------------------------------
# UART helpers
# ---------------------------------------------------------------------------
def collect_responses(ser, timeout: float = 3.0):
    """Collect all responses within timeout. Returns list of (cmd, param)."""
    responses = []
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            buf += chunk
            while len(buf) >= WORK_FRAME_SIZE:
                result = parse_work_response(buf[:WORK_FRAME_SIZE])
                if result is not None:
                    responses.append(result)
                    buf = buf[WORK_FRAME_SIZE:]
                else:
                    buf = buf[1:]  # skip bad byte
        else:
            time.sleep(0.01)
    return responses


def send_work_cmd(ser, cmd: int, param: int, timeout: float = 3.0):
    """Send work frame, collect responses."""
    frame = build_work_frame(cmd, param)
    ser.reset_input_buffer()
    ser.write(frame)
    return collect_responses(ser, timeout)


def send_factory_cmd(ser, cmd: int, param_h: int, param_l: int, timeout: float = 3.0):
    """Send factory frame, collect responses."""
    frame = build_factory_frame(cmd, param_h, param_l)
    ser.reset_input_buffer()
    ser.write(frame)
    return collect_responses(ser, timeout)


def wait_for_response(ser, expected_cmd: int, timeout: float = 20.0):
    """Wait for a specific response cmd. Returns (True, param) or (False, None)."""
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            buf += chunk
            while len(buf) >= WORK_FRAME_SIZE:
                result = parse_work_response(buf[:WORK_FRAME_SIZE])
                if result is not None:
                    if result[0] == expected_cmd:
                        return True, result[1]
                    buf = buf[WORK_FRAME_SIZE:]
                else:
                    buf = buf[1:]
        else:
            time.sleep(0.01)
    return False, None


# ---------------------------------------------------------------------------
# Test runner
# ---------------------------------------------------------------------------
def run_tests(port: str, baud: int) -> bool:
    try:
        import serial
    except ImportError:
        print("ERROR: pyserial not installed. Run: pip3 install pyserial")
        sys.exit(1)

    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR opening {port}: {e}")
        sys.exit(1)

    results = []

    def check(name: str, passed: bool, detail: str = ""):
        status = "PASS" if passed else "FAIL"
        msg = f"  [{status}] {name}"
        if detail:
            msg += f"  ({detail})"
        print(msg)
        results.append((name, passed))
        return passed

    print("=" * 60)
    print("ZLENS_DC Factory Mode Verification")
    print(f"  port={port}  baud={baud}")
    print("=" * 60)

    # Clear boot residual
    ser.reset_input_buffer()
    time.sleep(0.5)
    ser.reset_input_buffer()

    # --- Test 1: HOMING first (ensure known state) ---
    print("\n[TEST 1] HOMING (establish baseline)")
    frame = build_work_frame(CMD_HOMING, 0)
    ser.reset_input_buffer()
    ser.write(frame)
    ok, _ = wait_for_response(ser, RSP_HOMING_DONE, timeout=25.0)
    check("HOMING -> HOMING_DONE", ok)
    if not ok:
        print("  CRITICAL: Homing failed, aborting.")
        ser.close()
        return False
    time.sleep(2.0)
    ser.reset_input_buffer()

    # --- Test 2: Enter factory mode ---
    print("\n[TEST 2] Enter factory mode (0xFA param=0xFAE5)")
    responses = send_work_cmd(ser, CMD_SWITCH_FACTORY, FACTORY_MAGIC_PARAM)
    # Expect echo + response with param=0x0001
    entered = any(cmd == CMD_SWITCH_FACTORY and param == 0x0001
                  for cmd, param in responses)
    check("Enter factory mode -> response 0x0001", entered,
          f"responses={[(hex(c), hex(p)) for c, p in responses]}")

    # --- Test 3: Erase all ---
    print("\n[TEST 3] Erase all zoom entries (0xF0 + magic)")
    send_factory_cmd(ser, FCMD_ERASE_ALL, FACTORY_MAGIC_HIGH, FACTORY_MAGIC_LOW)
    time.sleep(0.2)
    # No specific response expected (echo only), just proceed
    check("Erase command sent", True)

    # --- Test 4: Set 5 entries ---
    print("\n[TEST 4] Set zoom entries (0xF1)")
    for zoom_x10, angle_x100 in TEST_ZOOM_TABLE:
        send_factory_cmd(ser, FCMD_SET_ENTRY, zoom_x10, angle_x100)
        time.sleep(0.1)
        print(f"    Set: {zoom_x10/10:.1f}x -> angle={angle_x100/100:.2f} deg")
    check(f"Set {len(TEST_ZOOM_TABLE)} entries", True)

    # --- Test 5: Exit factory mode ---
    print("\n[TEST 5] Exit factory mode (0xFA + magic -> saves to Flash)")
    responses = send_factory_cmd(ser, FCMD_SWITCH_TO_WORK,
                                 FACTORY_MAGIC_HIGH, FACTORY_MAGIC_LOW,
                                 timeout=5.0)
    exited = any(cmd == CMD_SWITCH_FACTORY and param == 0x0000
                 for cmd, param in responses)
    check("Exit factory mode -> response 0x0000", exited,
          f"responses={[(hex(c), hex(p)) for c, p in responses]}")
    time.sleep(0.5)
    ser.reset_input_buffer()

    # --- Test 6: Query zoom range (verify table loaded) ---
    print("\n[TEST 6] Query zoom range (verify calibrated table)")
    responses = send_work_cmd(ser, CMD_QUERY_RANGE, 0, timeout=3.0)
    min_zoom = None
    max_zoom = None
    for cmd, param in responses:
        if cmd == RSP_MIN_ZOOM:
            min_zoom = param
        elif cmd == RSP_MAX_ZOOM:
            max_zoom = param
    expected_min = TEST_ZOOM_TABLE[0][0]   # 6
    expected_max = TEST_ZOOM_TABLE[-1][0]  # 70
    check("Min zoom matches", min_zoom == expected_min,
          f"expected={expected_min}, got={min_zoom}")
    check("Max zoom matches", max_zoom == expected_max,
          f"expected={expected_max}, got={max_zoom}")

    # --- Test 7: HOMING after table change ---
    print("\n[TEST 7] HOMING (after table change)")
    frame = build_work_frame(CMD_HOMING, 0)
    ser.reset_input_buffer()
    ser.write(frame)
    ok, _ = wait_for_response(ser, RSP_HOMING_DONE, timeout=25.0)
    check("HOMING -> HOMING_DONE", ok)
    if not ok:
        print("  WARNING: Homing failed after table change")
    time.sleep(2.0)
    ser.reset_input_buffer()

    # --- Test 8: SET_ZOOM to calibrated entries ---
    print("\n[TEST 8] SET_ZOOM to calibrated entries")
    for zoom_x10, _ in TEST_ZOOM_TABLE[1:]:  # skip 0.6x (already at home)
        label = f"{zoom_x10/10:.1f}x"
        print(f"    Moving to {label}...")
        frame = build_work_frame(CMD_SET_ZOOM, zoom_x10)
        ser.reset_input_buffer()
        ser.write(frame)
        ok, _ = wait_for_response(ser, RSP_ARRIVED, timeout=20.0)
        check(f"SET_ZOOM({label}) -> ARRIVED", ok)
        time.sleep(0.5)
        ser.reset_input_buffer()

    # Return to 0.6x
    print("    Moving to 0.6x...")
    frame = build_work_frame(CMD_SET_ZOOM, 6)
    ser.reset_input_buffer()
    ser.write(frame)
    ok, _ = wait_for_response(ser, RSP_ARRIVED, timeout=20.0)
    check("SET_ZOOM(0.6x) -> ARRIVED", ok)

    ser.close()

    # --- Summary ---
    print()
    print("=" * 60)
    print("SUMMARY")
    print("-" * 60)
    all_pass = True
    for name, passed in results:
        status = "PASS" if passed else "FAIL"
        print(f"  [{status}] {name}")
        if not passed:
            all_pass = False
    print("-" * 60)
    n_pass = sum(1 for _, p in results if p)
    n_total = len(results)
    overall = "ALL PASS" if all_pass else f"FAILED ({n_total - n_pass}/{n_total} failed)"
    print(f"RESULT: {overall}  ({n_pass}/{n_total})")
    print("=" * 60)
    return all_pass


def main():
    parser = argparse.ArgumentParser(
        description="ZLENS_DC factory mode calibration verification"
    )
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    args = parser.parse_args()

    passed = run_tests(args.port, args.baud)
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
