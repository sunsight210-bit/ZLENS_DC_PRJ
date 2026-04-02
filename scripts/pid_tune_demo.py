#!/usr/bin/env python3
"""
pid_tune_demo.py — Phase 8 PID tuning automation script

Automates step-response testing for ZLENS_DC PID controller tuning.
Sends SET_ZOOM commands via UART, captures SWO log, parses [PID] lines,
and produces a PASS/FAIL report with tuning suggestions.

Usage:
    python3 scripts/pid_tune_demo.py [--port PORT] [--baud BAUD] [--swo-time SECS]

Interface:
    --port      Serial port for UART (default: /dev/ttyUSB0)
    --baud      Baud rate (default: 115200)
    --swo-time  SWO capture duration in seconds (default: 60)

Protocol:
    Frame format: [0xA5] [CMD] [PARAM_H] [PARAM_L] [CRC16_L] [CRC16_H]
    CRC16: Modbus CRC over first 4 bytes

Command IDs (from task_config.hpp):
    HOMING   = 0x01  param=0
    SET_ZOOM = 0x10  param=zoom_x10 (10=1.0X, 20=2.0X)

Response codes:
    HOMING_DONE = 0x01
    ARRIVED     = 0x02

PASS criteria:
    overshoot    < 5%
    steady-state error < DEADZONE (25 counts)
    oscillations <= 1
    settling time < 2000 ms

See docs/build-flash-swo-workflow.md for OpenOCD SWO capture details.
"""

import argparse
import os
import re
import shutil
import struct
import subprocess
import sys
import time

# ---------------------------------------------------------------------------
# Paths (from docs/build-flash-swo-workflow.md)
# ---------------------------------------------------------------------------
OPENOCD = (
    "/home/cbn/st/stm32cubeide_2.1.0/plugins/"
    "com.st.stm32cube.ide.mcu.externaltools.openocd.linux64_2.4.400.202601091506/"
    "tools/bin/openocd"
)
OCD_SCRIPTS = (
    "/home/cbn/st/stm32cubeide_2.1.0/plugins/"
    "com.st.stm32cube.ide.mcu.debug.openocd_2.3.300.202602021527/"
    "resources/openocd/st_scripts/"
)
SWO_BIN = "build/swo/swo.bin"
SWO_DIR = "build/swo"
PARSE_SWO = "scripts/parse_swo.py"

# ---------------------------------------------------------------------------
# Protocol constants (from comm_protocol.hpp / build-flash-swo-workflow.md)
# ---------------------------------------------------------------------------
FRAME_HEADER = 0xA5
WORK_FRAME_SIZE = 6

CMD_HOMING   = 0x01
CMD_SET_ZOOM = 0x10

RSP_HOMING_DONE = 0x01
RSP_ARRIVED     = 0x02

DEADZONE = 25        # counts, from motor_ctrl.hpp

# PID PASS thresholds
MAX_OVERSHOOT_PCT  = 5.0    # %
MAX_SETTLING_MS    = 2000   # ms
MAX_OSCILLATIONS   = 1
MAX_STEADY_ERROR   = DEADZONE


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
    """Parse a 6-byte work frame response. Returns (cmd, param) or None."""
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
                  timeout: float = 15.0, label: str = ""):
    """Send a command frame and wait for the expected response byte."""
    frame = build_frame(cmd, param)
    ser.reset_input_buffer()
    ser.write(frame)
    tag = label or f"CMD=0x{cmd:02X} param={param}"
    print(f"  TX {tag}: {frame.hex(' ')}")

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
                    print(f"  RX: cmd=0x{rsp_cmd:02X} param=0x{rsp_param:04X}")
                    if rsp_cmd == expected_rsp:
                        return True
    print(f"  TIMEOUT waiting for RSP=0x{expected_rsp:02X} ({tag})")
    return False


# ---------------------------------------------------------------------------
# SWO capture helpers
# ---------------------------------------------------------------------------
def clear_swo_dir():
    """Remove old SWO data as required by swo_cleanup rule."""
    if os.path.isdir(SWO_DIR):
        for f in os.listdir(SWO_DIR):
            fpath = os.path.join(SWO_DIR, f)
            try:
                os.remove(fpath)
            except OSError:
                pass
    else:
        os.makedirs(SWO_DIR, exist_ok=True)
    print(f"[SWO] Cleared {SWO_DIR}/")


def start_swo_capture(swo_time_ms: int) -> subprocess.Popen:
    """
    Start OpenOCD SWO capture in background.
    Uses 'reset halt → configure TPIU → resume' as mandated by
    docs/build-flash-swo-workflow.md (路径 B).
    traceclk = 64000000 (64MHz HSI×PLL).
    """
    ocd_cmds = (
        "init; "
        "reset halt; "
        "tpiu config internal build/swo/swo.bin uart off 64000000 2000000; "
        "itm port 0 on; "
        "resume; "
        f"sleep {swo_time_ms}; "
        "shutdown"
    )
    cmd = [
        OPENOCD,
        "-s", OCD_SCRIPTS,
        "-f", "interface/stlink-dap.cfg",
        "-f", "target/stm32f1x.cfg",
        "-c", ocd_cmds,
    ]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    print(f"[SWO] OpenOCD started (PID={proc.pid}), capture={swo_time_ms}ms")
    return proc


def stop_swo_capture(proc: subprocess.Popen):
    """Wait for OpenOCD to finish (it shuts down after its own sleep)."""
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.terminate()
        proc.wait()
    print("[SWO] OpenOCD finished")


def parse_swo_to_text() -> list[str]:
    """Run parse_swo.py and return all lines from the output."""
    result = subprocess.run(
        [sys.executable, PARSE_SWO, SWO_BIN],
        capture_output=True, text=True,
    )
    lines = result.stdout.splitlines()
    return lines


# ---------------------------------------------------------------------------
# PID log analysis
# ---------------------------------------------------------------------------
# Expected SWO [PID] line format (defined in motor_ctrl.cpp PID_TUNE_LOG block):
#   [PID] t=<ms> pos=<counts> target=<counts> err=<counts> pwm=<duty>
PID_RE = re.compile(
    r'\[PID\]\s+t=(\d+)\s+pos=(-?\d+)\s+target=(-?\d+)\s+err=(-?\d+)\s+pwm=(-?\d+)'
)


def extract_pid_samples(lines: list[str], target: int) -> list[dict]:
    """Extract [PID] samples that match the given target value."""
    samples = []
    for line in lines:
        m = PID_RE.search(line)
        if m:
            t_ms, pos, tgt, err, pwm = (int(x) for x in m.groups())
            if tgt == target:
                samples.append({'t': t_ms, 'pos': pos, 'target': tgt,
                                 'err': err, 'pwm': pwm})
    return samples


def analyze_step(samples: list[dict], label: str) -> dict:
    """
    Analyze one step response.
    Returns dict with keys: label, rise_ms, overshoot_pct, steady_err,
    settling_ms, oscillations, pass.
    """
    result = {
        'label': label,
        'n_samples': len(samples),
        'rise_ms': None,
        'overshoot_pct': 0.0,
        'steady_err': None,
        'settling_ms': None,
        'oscillations': 0,
        'pass': False,
        'note': '',
    }

    if len(samples) < 5:
        result['note'] = 'INSUFFICIENT_DATA'
        return result

    t0      = samples[0]['t']
    target  = samples[0]['target']
    start   = samples[0]['pos']
    total   = abs(target - start)

    if total == 0:
        result['note'] = 'NO_MOVEMENT'
        result['pass'] = True
        return result

    # Rise time: first sample within ±DEADZONE of target
    rise_ms = None
    for s in samples:
        if abs(s['pos'] - target) <= DEADZONE:
            rise_ms = s['t'] - t0
            break
    result['rise_ms'] = rise_ms

    # Overshoot: maximum exceedance beyond target
    max_pos  = max(s['pos'] for s in samples)
    min_pos  = min(s['pos'] for s in samples)
    if target > start:
        overshoot_counts = max(0, max_pos - target)
    else:
        overshoot_counts = max(0, target - min_pos)
    overshoot_pct = 100.0 * overshoot_counts / total if total else 0.0
    result['overshoot_pct'] = overshoot_pct

    # Steady-state: average of last 10 samples
    tail = samples[-10:]
    steady_pos = sum(s['pos'] for s in tail) / len(tail)
    steady_err = abs(steady_pos - target)
    result['steady_err'] = steady_err

    # Settling time: last time position was outside ±DEADZONE band
    settling_ms = None
    for s in reversed(samples):
        if abs(s['pos'] - target) > DEADZONE:
            settling_ms = s['t'] - t0
            break
    result['settling_ms'] = settling_ms if settling_ms is not None else 0

    # Oscillation count: zero-crossings of error around 0
    errors = [s['err'] for s in samples]
    crossings = 0
    for i in range(1, len(errors)):
        if (errors[i - 1] > 0) != (errors[i] > 0):
            crossings += 1
    result['oscillations'] = crossings // 2

    # PASS/FAIL
    passed = (
        overshoot_pct < MAX_OVERSHOOT_PCT
        and steady_err < MAX_STEADY_ERROR
        and result['oscillations'] <= MAX_OSCILLATIONS
        and result['settling_ms'] < MAX_SETTLING_MS
    )
    result['pass'] = passed
    return result


def print_report(analyses: list[dict]) -> bool:
    """Print formatted table. Returns True if all PASS."""
    print()
    print("=" * 72)
    print("PID STEP RESPONSE REPORT")
    print("=" * 72)
    header = (
        f"{'Step':<20} {'Rise(ms)':>8} {'OS%':>6} {'SteadyErr':>10} "
        f"{'Settle(ms)':>11} {'Osc':>4} {'RESULT':>7}"
    )
    print(header)
    print("-" * 72)

    all_pass = True
    for a in analyses:
        result_str = "PASS" if a['pass'] else "FAIL"
        if not a['pass']:
            all_pass = False
        rise    = f"{a['rise_ms']}" if a['rise_ms'] is not None else "N/A"
        os_pct  = f"{a['overshoot_pct']:.1f}" if a['overshoot_pct'] is not None else "N/A"
        s_err   = f"{a['steady_err']:.1f}" if a['steady_err'] is not None else "N/A"
        settle  = f"{a['settling_ms']}" if a['settling_ms'] is not None else "N/A"
        osc     = str(a['oscillations'])
        note    = f"  [{a['note']}]" if a['note'] else ""
        print(
            f"{a['label']:<20} {rise:>8} {os_pct:>6} {s_err:>10} "
            f"{settle:>11} {osc:>4} {result_str:>7}{note}"
        )

    print("-" * 72)
    print(f"Thresholds: OS<{MAX_OVERSHOOT_PCT}%, err<{MAX_STEADY_ERROR} counts, "
          f"osc<={MAX_OSCILLATIONS}, settle<{MAX_SETTLING_MS}ms")
    print()

    if not all_pass:
        print("TUNING SUGGESTIONS:")
        for a in analyses:
            if a['pass']:
                continue
            if a['overshoot_pct'] is not None and a['overshoot_pct'] >= MAX_OVERSHOOT_PCT:
                print(f"  [{a['label']}] Overshoot {a['overshoot_pct']:.1f}% >= {MAX_OVERSHOOT_PCT}%"
                      f" → Reduce KP or increase KD")
            if a['oscillations'] > MAX_OSCILLATIONS:
                print(f"  [{a['label']}] Oscillations={a['oscillations']} > {MAX_OSCILLATIONS}"
                      f" → Increase KD or reduce KP")
            if a['steady_err'] is not None and a['steady_err'] >= MAX_STEADY_ERROR:
                print(f"  [{a['label']}] Steady error={a['steady_err']:.1f} >= {MAX_STEADY_ERROR}"
                      f" → Increase KI or KP")
            if a['settling_ms'] is not None and a['settling_ms'] >= MAX_SETTLING_MS:
                print(f"  [{a['label']}] Settling={a['settling_ms']}ms >= {MAX_SETTLING_MS}ms"
                      f" → Increase KP (ensure stable first)")

    print()
    overall = "ALL PASS" if all_pass else "FAIL — see suggestions above"
    print(f"OVERALL: {overall}")
    print("=" * 72)
    return all_pass


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="ZLENS_DC PID tuning automation")
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--swo-time", type=int, default=60,
                        help="SWO capture duration in seconds (default: 60)")
    args = parser.parse_args()

    swo_ms = args.swo_time * 1000

    print("=" * 72)
    print("ZLENS_DC PID Tune Demo")
    print(f"  port={args.port}  baud={args.baud}  swo-time={args.swo_time}s")
    print("=" * 72)

    # Step 1: clear old SWO data
    clear_swo_dir()

    # Step 2: start SWO capture (background)
    ocd_proc = start_swo_capture(swo_ms)

    # Wait for OpenOCD to initialise and reset MCU (~4 s)
    print("[WAIT] Allowing 4s for OpenOCD reset + MCU boot ...")
    time.sleep(4)

    # Step 3-4: UART tests
    try:
        import serial
    except ImportError:
        print("ERROR: pyserial not installed. Run: pip3 install pyserial")
        ocd_proc.terminate()
        sys.exit(1)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=2)
    except serial.SerialException as e:
        print(f"ERROR opening {args.port}: {e}")
        ocd_proc.terminate()
        sys.exit(1)

    analyses = []

    # Step 3: HOMING
    print("\n[STEP] HOMING ...")
    ok = send_and_wait(ser, CMD_HOMING, 0, RSP_HOMING_DONE,
                       timeout=20.0, label="HOMING")
    if not ok:
        print("ERROR: HOMING did not complete. Aborting.")
        ser.close()
        ocd_proc.terminate()
        sys.exit(1)
    print("  HOMING done. Waiting 1s settle ...")
    time.sleep(1)

    # Step 4: 3 rounds of 1.0X <-> 2.0X step response
    zoom_pairs = [
        (10, "1.0X→2.0X", 20),
        (20, "2.0X→1.0X", 10),
        (10, "1.0X→2.0X", 20),
        (20, "2.0X→1.0X", 10),
        (10, "1.0X→2.0X", 20),
        (20, "2.0X→1.0X", 10),
    ]

    for (from_zoom, label, to_zoom) in zoom_pairs:
        print(f"\n[STEP] SET_ZOOM {label} ...")
        ok = send_and_wait(ser, CMD_SET_ZOOM, to_zoom, RSP_ARRIVED,
                           timeout=15.0, label=f"SET_ZOOM(x{to_zoom/10:.1f})")
        if not ok:
            print(f"  WARNING: no ARRIVED for {label}")
        time.sleep(0.5)

    ser.close()
    print("\n[UART] Tests done. Waiting for SWO capture to complete ...")

    # Step 5: stop SWO capture
    stop_swo_capture(ocd_proc)

    # Step 6: parse SWO
    print("[SWO] Parsing SWO data ...")
    if not os.path.exists(SWO_BIN):
        print(f"ERROR: {SWO_BIN} not found — SWO capture may have failed.")
        sys.exit(1)

    swo_lines = parse_swo_to_text()
    pid_lines = [l for l in swo_lines if '[PID]' in l]
    print(f"[SWO] Total lines={len(swo_lines)}, [PID] lines={len(pid_lines)}")

    if not pid_lines:
        print("WARNING: No [PID] log lines found.")
        print("  Ensure firmware was built with -DPID_TUNE_LOG=ON and flashed.")
        print("  Build: cmake -B build/fw -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake "
              "-DPID_TUNE_LOG=ON && cmake --build build/fw")
        sys.exit(1)

    # Step 7/8: analyze each step (1.0X→2.0X and 2.0X→1.0X separately)
    TARGET_1X = 13235   # encoder counts for 1.0X
    TARGET_2X = 27070   # encoder counts for 2.0X

    for target, label in [(TARGET_2X, "→2.0X"), (TARGET_1X, "→1.0X")]:
        samples = extract_pid_samples(swo_lines, target)
        print(f"  [{label}] {len(samples)} PID samples at target={target}")
        # Split into individual steps by detecting gaps > 500ms in timestamps
        steps = []
        current = []
        for s in samples:
            if current and (s['t'] - current[-1]['t']) > 500:
                steps.append(current)
                current = [s]
            else:
                current.append(s)
        if current:
            steps.append(current)

        for i, step in enumerate(steps):
            a = analyze_step(step, f"{label} #{i+1}")
            analyses.append(a)

    # Step 9: print report
    passed = print_report(analyses)
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
