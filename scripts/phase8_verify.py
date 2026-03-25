#!/usr/bin/env python3
"""
phase8_verify.py — Phase 8 board verification script

Sends UART commands to verify ZLENS_DC firmware position accuracy
after AS5311 encoder replacement (Phase 8).

Usage:
    python3 scripts/phase8_verify.py [--port PORT] [--baud BAUD]

Interface:
    --port  Serial port for UART (default: /dev/ttyUSB0)
    --baud  Baud rate (default: 115200)

Tests:
    1. HOMING → HOMING_DONE
    2. SET_ZOOM(1.0X=10) → ARRIVED, query position ≈ 13235 ± DEADZONE
    3. SET_ZOOM(2.0X=20) → ARRIVED, query position ≈ 27070 ± DEADZONE
    4. SET_ZOOM(1.0X=10) → ARRIVED (reverse direction test)
    5. SET_ZOOM(7.0X=70) → ARRIVED, query position ≈ 63297 ± DEADZONE
    6. SET_ZOOM(0.6X=6)  → ARRIVED, query position ≈ 128   ± DEADZONE

Protocol:
    Frame: [0xA5] [CMD] [PARAM_H] [PARAM_L] [CRC16_L] [CRC16_H]
    CRC16: Modbus CRC over first 4 bytes

See docs/build-flash-swo-workflow.md for hardware setup details.
"""

import argparse
import struct
import sys
import time

# ---------------------------------------------------------------------------
# Protocol constants (from comm_protocol.hpp, task_config.hpp)
# ---------------------------------------------------------------------------
FRAME_HEADER    = 0xA5
WORK_FRAME_SIZE = 6

CMD_HOMING      = 0x01
CMD_SET_ZOOM    = 0x10
CMD_QUERY_ZOOM  = 0x20   # returns current position in counts

RSP_HOMING_DONE = 0x01
RSP_ARRIVED     = 0x02
RSP_ZOOM        = 0x10   # query zoom response

DEADZONE = 25            # counts, from motor_ctrl.hpp MotorCtrl::DEADZONE

# Expected positions (encoder counts) from zoom_table (phase 8 calibration)
EXPECTED_POSITIONS = {
    6:  128,    # 0.6X
    10: 13235,  # 1.0X
    20: 27070,  # 2.0X
    70: 63297,  # 7.0X
}


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


def build_frame(cmd: int, param: int) -> bytes:
    hdr = struct.pack('>BBH', FRAME_HEADER, cmd, param)
    crc = crc16_modbus(hdr)
    return hdr + struct.pack('<H', crc)


def parse_response(data: bytes):
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
def send_and_wait(ser, cmd: int, param: int, expected_rsp: int,
                  timeout: float = 20.0, label: str = ""):
    """
    Send a command frame and wait for a specific response byte.
    Returns (True, rsp_param) on success, (False, None) on timeout.
    """
    frame = build_frame(cmd, param)
    ser.reset_input_buffer()
    ser.write(frame)
    tag = label or f"CMD=0x{cmd:02X} param={param}"

    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(WORK_FRAME_SIZE)
        if chunk:
            buf += chunk
            while len(buf) >= WORK_FRAME_SIZE:
                result = parse_response(buf[:WORK_FRAME_SIZE])
                buf = buf[WORK_FRAME_SIZE:]
                if result is not None:
                    rsp_cmd, rsp_param = result
                    if rsp_cmd == expected_rsp:
                        return True, rsp_param
    return False, None


def query_position(ser, timeout: float = 3.0):
    """Send QUERY_ZOOM and return position counts, or None on failure."""
    ok, param = send_and_wait(ser, CMD_QUERY_ZOOM, 0, RSP_ZOOM,
                              timeout=timeout, label="QUERY_ZOOM")
    if ok:
        return param
    return None


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
        ser = serial.Serial(port, baud, timeout=2)
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
        results.append((name, passed, detail))
        return passed

    print("=" * 60)
    print("ZLENS_DC Phase 8 Board Verification")
    print(f"  port={port}  baud={baud}")
    print("=" * 60)

    # --- Test 1: HOMING ---
    print("\n[TEST 1] HOMING")
    ok, _ = send_and_wait(ser, CMD_HOMING, 0, RSP_HOMING_DONE,
                          timeout=25.0, label="HOMING")
    check("HOMING → HOMING_DONE", ok)
    if not ok:
        print("  CRITICAL: Homing failed, aborting remaining tests.")
        ser.close()
        return False
    time.sleep(0.5)

    # --- Test 2: SET_ZOOM 1.0X ---
    print("\n[TEST 2] SET_ZOOM 1.0X (forward from home)")
    ok, _ = send_and_wait(ser, CMD_SET_ZOOM, 10, RSP_ARRIVED,
                          timeout=20.0, label="SET_ZOOM(1.0X)")
    if ok:
        pos = query_position(ser)
        expected = EXPECTED_POSITIONS[10]
        if pos is not None:
            err = abs(pos - expected)
            detail = f"pos={pos} expected={expected} err={err} DEADZONE={DEADZONE}"
            check("SET_ZOOM(1.0X) position", err <= DEADZONE, detail)
        else:
            check("SET_ZOOM(1.0X) position", False, "query failed")
    else:
        check("SET_ZOOM(1.0X) ARRIVED", False, "timeout")
    time.sleep(0.3)

    # --- Test 3: SET_ZOOM 2.0X ---
    print("\n[TEST 3] SET_ZOOM 2.0X (forward)")
    ok, _ = send_and_wait(ser, CMD_SET_ZOOM, 20, RSP_ARRIVED,
                          timeout=20.0, label="SET_ZOOM(2.0X)")
    if ok:
        pos = query_position(ser)
        expected = EXPECTED_POSITIONS[20]
        if pos is not None:
            err = abs(pos - expected)
            detail = f"pos={pos} expected={expected} err={err} DEADZONE={DEADZONE}"
            check("SET_ZOOM(2.0X) position", err <= DEADZONE, detail)
        else:
            check("SET_ZOOM(2.0X) position", False, "query failed")
    else:
        check("SET_ZOOM(2.0X) ARRIVED", False, "timeout")
    time.sleep(0.3)

    # --- Test 4: SET_ZOOM 1.0X (reverse) ---
    print("\n[TEST 4] SET_ZOOM 1.0X (reverse direction)")
    ok, _ = send_and_wait(ser, CMD_SET_ZOOM, 10, RSP_ARRIVED,
                          timeout=20.0, label="SET_ZOOM(1.0X) reverse")
    if ok:
        pos = query_position(ser)
        expected = EXPECTED_POSITIONS[10]
        if pos is not None:
            err = abs(pos - expected)
            detail = f"pos={pos} expected={expected} err={err} DEADZONE={DEADZONE}"
            check("SET_ZOOM(1.0X) reverse position", err <= DEADZONE, detail)
        else:
            check("SET_ZOOM(1.0X) reverse position", False, "query failed")
    else:
        check("SET_ZOOM(1.0X) reverse ARRIVED", False, "timeout")
    time.sleep(0.3)

    # --- Test 5: SET_ZOOM 7.0X ---
    print("\n[TEST 5] SET_ZOOM 7.0X (long forward move)")
    ok, _ = send_and_wait(ser, CMD_SET_ZOOM, 70, RSP_ARRIVED,
                          timeout=30.0, label="SET_ZOOM(7.0X)")
    if ok:
        pos = query_position(ser)
        expected = EXPECTED_POSITIONS[70]
        if pos is not None:
            err = abs(pos - expected)
            detail = f"pos={pos} expected={expected} err={err} DEADZONE={DEADZONE}"
            check("SET_ZOOM(7.0X) position", err <= DEADZONE, detail)
        else:
            check("SET_ZOOM(7.0X) position", False, "query failed")
    else:
        check("SET_ZOOM(7.0X) ARRIVED", False, "timeout")
    time.sleep(0.3)

    # --- Test 6: SET_ZOOM 0.6X ---
    print("\n[TEST 6] SET_ZOOM 0.6X (reverse to near-home)")
    ok, _ = send_and_wait(ser, CMD_SET_ZOOM, 6, RSP_ARRIVED,
                          timeout=30.0, label="SET_ZOOM(0.6X)")
    if ok:
        pos = query_position(ser)
        expected = EXPECTED_POSITIONS[6]
        if pos is not None:
            err = abs(pos - expected)
            detail = f"pos={pos} expected={expected} err={err} DEADZONE={DEADZONE}"
            check("SET_ZOOM(0.6X) position", err <= DEADZONE, detail)
        else:
            check("SET_ZOOM(0.6X) position", False, "query failed")
    else:
        check("SET_ZOOM(0.6X) ARRIVED", False, "timeout")

    ser.close()

    # --- Summary ---
    print()
    print("=" * 60)
    print("SUMMARY")
    print("-" * 60)
    all_pass = True
    for name, passed, detail in results:
        status = "PASS" if passed else "FAIL"
        print(f"  [{status}] {name}")
        if not passed:
            all_pass = False
    print("-" * 60)
    n_pass = sum(1 for _, p, _ in results if p)
    n_total = len(results)
    overall = "ALL PASS" if all_pass else f"FAILED ({n_total - n_pass}/{n_total} failed)"
    print(f"RESULT: {overall}")
    print("=" * 60)
    return all_pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="ZLENS_DC Phase 8 board verification"
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
