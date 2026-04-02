#!/usr/bin/env python3
"""Phase 7 板上验证 — 齿轮间隙补偿与精度优化

验证项：
  1. 0x60 触发全量诊断链（自检→归零→间隙→精度）
  2. 归零完成（HOMING_DONE 响应）
  3. 诊断链完成后系统 READY
  4. 间隙补偿后正常定位精度

⚠ 涉及电机运动：归零会撞机械限位，间隙/精度测试会全行程运动。
"""

import argparse
import struct
import sys
import time

import serial


# ─── CRC16/MODBUS ───

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


# ─── Protocol v2.5 constants ───

FRAME_HEADER    = 0xA5
WORK_FRAME_SIZE = 6

# Commands
CMD_FORCE_STOP    = 0x02
CMD_SET_ZOOM      = 0x10
CMD_QUERY_ZOOM    = 0x20
CMD_QUERY_STATUS  = 0x21
CMD_SELF_TEST     = 0x60

# Response commands
RSP_HOMING_DONE = 0x01
RSP_ARRIVED     = 0x02
RSP_ZOOM        = 0x10
RSP_STATUS      = 0x11
RSP_STALL_STOP  = 0xE1
RSP_OVERCURRENT = 0xE2

# Response params
RSP_HOMING_DONE_PARAM = 0x000F
RSP_ARRIVED_PARAM     = 0x000A
RSP_OK                = 0x0000

# System status codes (from SystemManager)
STATUS_READY  = 0x0000
STATUS_BUSY   = 0x0001
STATUS_HOMING = 0x0002
STATUS_ERROR  = 0x00FF


# ─── Protocol helper ───

class ZlensProtocol:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 2.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def build_frame(self, cmd: int, param: int = 0) -> bytes:
        header = struct.pack('>BBH', FRAME_HEADER, cmd, param)
        crc = crc16_modbus(header)
        return header + struct.pack('<H', crc)

    def parse_frame(self, data: bytes):
        if len(data) < WORK_FRAME_SIZE:
            return None
        if data[0] != FRAME_HEADER:
            return None
        crc_calc = crc16_modbus(data[:4])
        crc_recv = data[4] | (data[5] << 8)
        if crc_calc != crc_recv:
            return None
        cmd = data[1]
        param = (data[2] << 8) | data[3]
        return {'cmd': cmd, 'param': param, 'raw': data[:WORK_FRAME_SIZE]}

    def read_frame(self, timeout: float = 2.0):
        self.ser.timeout = timeout
        data = self.ser.read(WORK_FRAME_SIZE)
        if len(data) < WORK_FRAME_SIZE:
            return None
        return self.parse_frame(data)

    def read_frames_until(self, target_cmd: int, timeout: float = 10.0):
        """Read frames until target_cmd is found or timeout. Returns all frames."""
        frames = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            f = self.read_frame(timeout=min(remaining, 2.0))
            if f:
                frames.append(f)
                if f['cmd'] == target_cmd:
                    break
        return frames

    def send_and_receive_with_echo(self, cmd: int, param: int = 0,
                                   timeout: float = 2.0):
        frame = self.build_frame(cmd, param)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        echo = self.read_frame(timeout)
        resp = self.read_frame(timeout)
        return echo, resp

    def send_cmd_echo_only(self, cmd: int, param: int = 0,
                           timeout: float = 2.0):
        """Send command and read only the echo (no second frame read)."""
        frame = self.build_frame(cmd, param)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        return self.read_frame(timeout)

    def drain(self, timeout: float = 0.5):
        """Drain any pending data from the serial buffer."""
        self.ser.timeout = timeout
        while True:
            data = self.ser.read(64)
            if len(data) == 0:
                break


# ─── Test runner ───

class Phase7Verify:
    def __init__(self, proto: ZlensProtocol):
        self.proto = proto
        self.passed = 0
        self.failed = 0

    def check(self, condition: bool, msg: str):
        if condition:
            self.passed += 1
            print(f"  [PASS] {msg}")
        else:
            self.failed += 1
            print(f"  [FAIL] {msg}")

    # --- Test 1: Trigger full diagnostics ---

    def test_trigger_diagnostics(self):
        print("\n--- 1. 触发全量诊断 (0x60) ---")
        self.proto.drain()
        echo, resp = self.proto.send_and_receive_with_echo(CMD_SELF_TEST)

        self.check(echo is not None, "echo received")
        if echo:
            self.check(echo['cmd'] == CMD_SELF_TEST,
                       f"echo cmd=0x{echo['cmd']:02X} == 0x60")

        self.check(resp is not None, "ACK received")
        if resp:
            self.check(resp['cmd'] == CMD_SELF_TEST,
                       f"ACK cmd=0x{resp['cmd']:02X} == 0x60")
            self.check(resp['param'] == RSP_OK,
                       f"ACK param=0x{resp['param']:04X} == OK")

    # --- Test 2: Wait for homing done ---

    def test_homing_done(self):
        print("\n--- 2. 等待归零完成 (HOMING_DONE) ---")
        print("    等待自检+归零（堵转检测×2）...")

        # Self-test ~5s + homing ~15s = ~20s, use 30s timeout
        frames = self.proto.read_frames_until(RSP_HOMING_DONE, timeout=30.0)

        # Log all received frames
        for f in frames:
            print(f"    收到: cmd=0x{f['cmd']:02X} param=0x{f['param']:04X}")

        homing_frame = None
        for f in frames:
            if f['cmd'] == RSP_HOMING_DONE:
                homing_frame = f
                break

        self.check(homing_frame is not None, "HOMING_DONE received")
        if homing_frame:
            self.check(homing_frame['param'] == RSP_HOMING_DONE_PARAM,
                       f"param=0x{homing_frame['param']:04X} == 0x000F")

        # Check ZOOM response was also sent before HOMING_DONE
        zoom_frame = None
        for f in frames:
            if f['cmd'] == RSP_ZOOM:
                zoom_frame = f
                break
        self.check(zoom_frame is not None, "ZOOM response before HOMING_DONE")
        if zoom_frame:
            print(f"    归零后 zoom = {zoom_frame['param'] / 10:.1f}x")

    # --- Test 3: Wait for diagnostics chain to complete ---

    def test_diagnostics_complete(self):
        print("\n--- 3. 等待诊断链完成（间隙+精度） ---")
        print("    间隙测量 8 轮 + 精度测试 8 趟，预计 90-120s...")
        print("    用探测性 SET_ZOOM 检测 motor task 是否空闲...")

        # The diagnostics chain (backlash + accuracy) runs inside
        # the motor task state machine without changing system state,
        # so QUERY_STATUS always returns READY.
        # Instead, probe with a SET_ZOOM: if the motor task is busy
        # (BACKLASH_MEASURE / ACCURACY_TEST), the command is ignored
        # and no ARRIVED response is sent.  When diagnostics finish,
        # the motor task returns to IDLE and the SET_ZOOM goes through.
        #
        # Probe target: 3.5x (mid-range), which is far from the
        # homing position (0.6x) so we get a real movement + ARRIVED.

        PROBE_ZOOM = 35  # 3.5x
        t0 = time.time()
        deadline = t0 + 180.0
        ready = False

        while time.time() < deadline:
            time.sleep(5.0)
            elapsed = time.time() - t0
            self.proto.drain()

            # Send probe SET_ZOOM and check for ARRIVED
            echo = self.proto.send_cmd_echo_only(CMD_SET_ZOOM, PROBE_ZOOM)
            if echo is None:
                print(f"    [{elapsed:.0f}s] 无 echo，串口异常")
                continue

            # Wait for ARRIVED (if motor task accepted the command)
            frames = self.proto.read_frames_until(RSP_ARRIVED, timeout=20.0)
            arrived = any(f['cmd'] == RSP_ARRIVED for f in frames)

            if arrived:
                print(f"    [{elapsed:.0f}s] 探测 SET_ZOOM 3.5x 收到 ARRIVED — 诊断链已完成")
                ready = True
                break
            else:
                # Check for overcurrent/stall errors
                errors = [f for f in frames
                          if f['cmd'] in (RSP_STALL_STOP, RSP_OVERCURRENT)]
                if errors:
                    print(f"    [{elapsed:.0f}s] 运动错误 0x{errors[0]['cmd']:02X}")
                    break
                print(f"    [{elapsed:.0f}s] 诊断链仍在运行...")

        self.check(ready, "诊断链完成（探测 SET_ZOOM 3.5x 成功）")
        if ready:
            print(f"    诊断链总耗时约 {time.time()-t0:.0f}s")

    # --- Test 4: Positioning accuracy ---

    def test_positioning(self):
        print("\n--- 4. 间隙补偿后定位验证 ---")

        # Test 3 已将镜头移动到 3.5x (探测用)
        # 从 3.5x 出发: max → min → mid(return)
        test_zooms = [
            (70, "7.0x (max)"),
            (6, "0.6x (min)"),
            (35, "3.5x (return)"),
        ]

        for zoom_x10, label in test_zooms:
            print(f"\n    → SET_ZOOM {label}")
            self.proto.drain()

            # Only read echo — don't consume response frames
            echo = self.proto.send_cmd_echo_only(
                CMD_SET_ZOOM, zoom_x10, timeout=2.0)
            self.check(echo is not None, f"SET_ZOOM echo ({label})")

            # Wait for ZOOM + ARRIVED (motor movement up to ~15s)
            frames = self.proto.read_frames_until(RSP_ARRIVED, timeout=20.0)

            zoom_resp = None
            arrived_resp = None
            error_resp = None
            for f in frames:
                if f['cmd'] == RSP_ZOOM:
                    zoom_resp = f
                elif f['cmd'] == RSP_ARRIVED:
                    arrived_resp = f
                elif f['cmd'] in (RSP_STALL_STOP, RSP_OVERCURRENT):
                    error_resp = f

            if error_resp:
                self.check(False, f"运动错误 cmd=0x{error_resp['cmd']:02X} ({label})")
                continue

            self.check(arrived_resp is not None, f"ARRIVED ({label})")
            self.check(zoom_resp is not None, f"ZOOM response ({label})")
            if zoom_resp:
                actual = zoom_resp['param']
                print(f"      实际 zoom = {actual / 10:.1f}x"
                      f" (目标 {zoom_x10 / 10:.1f}x)")
                self.check(actual == zoom_x10,
                           f"zoom 精确 {actual / 10:.1f}x == {zoom_x10 / 10:.1f}x"
                           f" ({label})")

    # --- Run all ---

    def run_all(self):
        print("=" * 60)
        print("ZLENS_DC Phase 7 — 间隙补偿与精度优化 板上验证")
        print("=" * 60)
        print("⚠ 本测试涉及电机运动（归零/间隙/精度/定位）")
        print("  预计总耗时 3-5 分钟")

        self.test_trigger_diagnostics()
        self.test_homing_done()
        self.test_diagnostics_complete()
        self.test_positioning()

        print("\n" + "=" * 60)
        total = self.passed + self.failed
        print(f"Results: {self.passed} passed, {self.failed} failed / {total} total")
        if self.failed == 0:
            print(">>> ALL PASSED <<<")
        else:
            print(">>> SOME TESTS FAILED <<<")
        print("=" * 60)
        return self.failed == 0


def main():
    parser = argparse.ArgumentParser(
        description="ZLENS_DC Phase 7 板上验证: 间隙补偿与精度优化")
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud}...")
    proto = ZlensProtocol(args.port, args.baud)
    time.sleep(0.5)

    try:
        test = Phase7Verify(proto)
        ok = test.run_all()
    finally:
        proto.close()

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
