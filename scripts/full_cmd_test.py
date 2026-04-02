#!/usr/bin/env python3
"""全命令自动化联调测试 — 覆盖协议 v2.5 所有命令

测试顺序：
1. 查询类命令 (0x20-0x25, 0x03)
2. Homing (0x01) — 电机运动
3. SET_ZOOM (0x10) — 电机运动
4. ZOOM_INC/DEC (0x11, 0x12) — 电机运动
5. BUSY rejection — 运动中发命令
6. FORCE_STOP (0x02) — 运动中强制停止
7. CYCLE_START/STOP (0x30, 0x31) — 循环变倍
8. Self-Test (0x60)
"""

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

# ─── Constants ───

FRAME_HEADER    = 0xA5
WORK_FRAME_SIZE = 6

CMD_HOMING        = 0x01
CMD_FORCE_STOP    = 0x02
CMD_GET_STALL_CNT = 0x03
CMD_SET_ZOOM      = 0x10
CMD_ZOOM_INC      = 0x11
CMD_ZOOM_DEC      = 0x12
CMD_QUERY_ZOOM    = 0x20
CMD_QUERY_STATUS  = 0x21
CMD_QUERY_SPEED   = 0x22
CMD_QUERY_TYPE    = 0x23
CMD_QUERY_RANGE   = 0x24
CMD_QUERY_VERSION = 0x25
CMD_CYCLE_START   = 0x30
CMD_CYCLE_STOP    = 0x31
CMD_SELF_TEST     = 0x60

RSP_HOMING_DONE = 0x01
RSP_ARRIVED     = 0x02
RSP_REQ_INVALID = 0x03
RSP_ZOOM        = 0x10
RSP_STATUS      = 0x11
RSP_SPEED       = 0x12
RSP_TYPE        = 0x13
RSP_MIN_ZOOM    = 0x14
RSP_MAX_ZOOM    = 0x15
RSP_VERSION     = 0x16
RSP_STALL_STOP  = 0xE1
RSP_OVERCURRENT = 0xE2
RSP_STALL_COUNT = 0xE3

CMD_NAMES = {
    0x01: "HOMING", 0x02: "FORCE_STOP", 0x03: "GET_STALL_CNT",
    0x10: "SET_ZOOM", 0x11: "ZOOM_INC", 0x12: "ZOOM_DEC",
    0x20: "QUERY_ZOOM", 0x21: "QUERY_STATUS", 0x22: "QUERY_SPEED",
    0x23: "QUERY_TYPE", 0x24: "QUERY_RANGE", 0x25: "QUERY_VERSION",
    0x30: "CYCLE_START", 0x31: "CYCLE_STOP", 0x60: "SELF_TEST",
}

RSP_NAMES = {
    0x01: "HOMING_DONE", 0x02: "ARRIVED", 0x03: "REQ_INVALID",
    0x10: "ZOOM", 0x11: "STATUS", 0x12: "SPEED",
    0x13: "TYPE", 0x14: "MIN_ZOOM", 0x15: "MAX_ZOOM", 0x16: "VERSION",
    0xE0: "ERR_PARAM", 0xE1: "STALL_STOP", 0xE2: "OVERCURRENT",
    0xE3: "STALL_COUNT",
}

# ─── Protocol ───

class Proto:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=2.0)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def build(self, cmd, param=0):
        hdr = struct.pack('>BBH', FRAME_HEADER, cmd, param)
        crc = crc16_modbus(hdr)
        return hdr + struct.pack('<H', crc)

    def parse(self, data):
        if len(data) < WORK_FRAME_SIZE or data[0] != FRAME_HEADER:
            return None
        crc_calc = crc16_modbus(data[:4])
        crc_recv = data[4] | (data[5] << 8)
        if crc_calc != crc_recv:
            return None
        return {'cmd': data[1], 'param': (data[2] << 8) | data[3]}

    def read_frame(self, timeout=2.0):
        self.ser.timeout = timeout
        data = self.ser.read(WORK_FRAME_SIZE)
        if len(data) < WORK_FRAME_SIZE:
            return None
        return self.parse(data)

    def send(self, cmd, param=0):
        frame = self.build(cmd, param)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()

    def send_recv(self, cmd, param=0, timeout=2.0):
        """Send, get echo + 1 response."""
        self.send(cmd, param)
        echo = self.read_frame(timeout)
        resp = self.read_frame(timeout)
        return echo, resp

    def send_recv_multi(self, cmd, param=0, n=2, timeout=2.0):
        """Send, get echo + N responses."""
        self.send(cmd, param)
        echo = self.read_frame(timeout)
        resps = [self.read_frame(timeout) for _ in range(n)]
        return echo, resps

    def collect_responses(self, timeout=30.0, idle_timeout=2.0):
        """Collect all response frames until timeout.

        Keeps polling until deadline. Returns early only after
        idle_timeout seconds of no data AND at least one response received.
        """
        resps = []
        deadline = time.time() + timeout
        last_recv = time.time()
        while time.time() < deadline:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            r = self.read_frame(timeout=min(1.0, remaining))
            if r:
                resps.append(r)
                last_recv = time.time()
                # If we got a terminal response, can stop early
                if r['cmd'] in (RSP_HOMING_DONE, RSP_ARRIVED, RSP_STALL_STOP,
                                RSP_OVERCURRENT):
                    # Give a moment for any trailing frames
                    trailing = self.read_frame(timeout=0.5)
                    if trailing:
                        resps.append(trailing)
                    break
            else:
                # Only give up early if we already have responses and
                # haven't received anything for idle_timeout
                if resps and (time.time() - last_recv) > idle_timeout:
                    break
        return resps

def rsp_str(r):
    if r is None:
        return "None"
    name = RSP_NAMES.get(r['cmd'], f"0x{r['cmd']:02X}")
    return f"{name}(0x{r['param']:04X})"

# ─── Test runner ───

class FullTest:
    def __init__(self, proto):
        self.p = proto
        self.passed = 0
        self.failed = 0
        self.errors = []

    def check(self, ok, msg):
        if ok:
            self.passed += 1
            print(f"  [PASS] {msg}")
        else:
            self.failed += 1
            self.errors.append(msg)
            print(f"  [FAIL] {msg}")

    def verify_echo(self, echo, cmd, param, label="echo"):
        self.check(echo is not None, f"{label}: received")
        if echo:
            self.check(echo['cmd'] == cmd,
                       f"{label}: cmd=0x{echo['cmd']:02X} == 0x{cmd:02X}")

    # ─── Query tests ───

    def test_queries(self):
        print("\n" + "=" * 55)
        print("Section 1: Query Commands (no motor movement)")
        print("=" * 55)

        # Query Zoom
        print("\n--- 1.1 Query Zoom (0x20) ---")
        echo, resp = self.p.send_recv(CMD_QUERY_ZOOM)
        self.verify_echo(echo, CMD_QUERY_ZOOM, 0)
        self.check(resp is not None and resp['cmd'] == RSP_ZOOM,
                   f"response: {rsp_str(resp)}")

        # Query Status
        print("\n--- 1.2 Query Status (0x21) ---")
        echo, resp = self.p.send_recv(CMD_QUERY_STATUS)
        self.verify_echo(echo, CMD_QUERY_STATUS, 0)
        self.check(resp is not None and resp['cmd'] == RSP_STATUS,
                   f"response: {rsp_str(resp)}")

        # Query Speed
        print("\n--- 1.3 Query Speed (0x22) ---")
        echo, resp = self.p.send_recv(CMD_QUERY_SPEED)
        self.verify_echo(echo, CMD_QUERY_SPEED, 0)
        self.check(resp is not None and resp['cmd'] == RSP_SPEED,
                   f"response: {rsp_str(resp)}")

        # Query Type
        print("\n--- 1.4 Query Type (0x23) ---")
        echo, resp = self.p.send_recv(CMD_QUERY_TYPE)
        self.verify_echo(echo, CMD_QUERY_TYPE, 0)
        self.check(resp is not None and resp['cmd'] == RSP_TYPE,
                   f"response: {rsp_str(resp)}")

        # Query Range (2 responses: MIN_ZOOM + MAX_ZOOM)
        print("\n--- 1.5 Query Range (0x24) ---")
        echo, resps = self.p.send_recv_multi(CMD_QUERY_RANGE, n=2)
        self.verify_echo(echo, CMD_QUERY_RANGE, 0)
        self.check(resps[0] is not None and resps[0]['cmd'] == RSP_MIN_ZOOM,
                   f"MIN_ZOOM: {rsp_str(resps[0])}")
        self.check(resps[1] is not None and resps[1]['cmd'] == RSP_MAX_ZOOM,
                   f"MAX_ZOOM: {rsp_str(resps[1])}")

        # Query Version
        print("\n--- 1.6 Query Version (0x25) ---")
        echo, resp = self.p.send_recv(CMD_QUERY_VERSION)
        self.verify_echo(echo, CMD_QUERY_VERSION, 0)
        self.check(resp is not None and resp['cmd'] == RSP_VERSION,
                   f"response: {rsp_str(resp)}")

        # Stall Count
        print("\n--- 1.7 Get Stall Count (0x03) ---")
        echo, resp = self.p.send_recv(CMD_GET_STALL_CNT)
        self.verify_echo(echo, CMD_GET_STALL_CNT, 0)
        self.check(resp is not None and resp['cmd'] == RSP_STALL_COUNT,
                   f"response: {rsp_str(resp)}")

    # ─── Homing test ───

    def test_homing(self):
        print("\n" + "=" * 55)
        print("Section 2: Homing (0x01) — MOTOR WILL MOVE")
        print("=" * 55)

        self.p.send(CMD_HOMING)
        echo = self.p.read_frame(2.0)
        self.verify_echo(echo, CMD_HOMING, 0)

        # Wait for homing to complete: expect ZOOM + HOMING_DONE
        # Homing: reverse scan + retract + forward scan + move to soft min
        # Large encoder range (~400k counts) can take 2+ minutes
        print("  Waiting for homing completion (up to 180s)...")
        resps = self.p.collect_responses(timeout=180.0)
        resp_names = [rsp_str(r) for r in resps]
        print(f"  Received: {resp_names}")

        has_zoom = any(r['cmd'] == RSP_ZOOM for r in resps)
        has_done = any(r['cmd'] == RSP_HOMING_DONE for r in resps)
        has_stall = any(r['cmd'] == RSP_STALL_STOP for r in resps)
        has_overcurrent = any(r['cmd'] == RSP_OVERCURRENT for r in resps)

        self.check(has_zoom, "ZOOM response received")
        self.check(has_done, "HOMING_DONE response received")
        self.check(not has_stall, "no unexpected STALL_STOP")
        self.check(not has_overcurrent, "no OVERCURRENT")

        if has_done:
            # Verify system is READY after homing
            time.sleep(0.5)
            _, resp = self.p.send_recv(CMD_QUERY_STATUS)
            if resp:
                print(f"  Post-homing status: {rsp_str(resp)}")

    # ─── SET_ZOOM test ───

    def test_set_zoom(self):
        print("\n" + "=" * 55)
        print("Section 3: SET_ZOOM (0x10) — MOTOR WILL MOVE")
        print("=" * 55)

        # Get current range
        _, resps = self.p.send_recv_multi(CMD_QUERY_RANGE, n=2)
        min_zoom = resps[0]['param'] if resps[0] else 6
        max_zoom = resps[1]['param'] if resps[1] else 60
        print(f"  Zoom range: {min_zoom/10:.1f}x - {max_zoom/10:.1f}x")

        # Test: move to max zoom
        target = max_zoom
        print(f"\n--- 3.1 SET_ZOOM to {target/10:.1f}x ---")
        self.p.send(CMD_SET_ZOOM, target)
        echo = self.p.read_frame(2.0)
        self.verify_echo(echo, CMD_SET_ZOOM, target)

        resps = self.p.collect_responses(timeout=30.0)
        resp_names = [rsp_str(r) for r in resps]
        print(f"  Received: {resp_names}")

        has_zoom = any(r['cmd'] == RSP_ZOOM for r in resps)
        has_arrived = any(r['cmd'] == RSP_ARRIVED for r in resps)
        has_overcurrent = any(r['cmd'] == RSP_OVERCURRENT for r in resps)

        self.check(has_zoom, "ZOOM response received")
        self.check(has_arrived, "ARRIVED response received")
        self.check(not has_overcurrent, "no OVERCURRENT")

        time.sleep(0.5)

        # Test: move to min zoom
        target = min_zoom
        print(f"\n--- 3.2 SET_ZOOM to {target/10:.1f}x ---")
        self.p.send(CMD_SET_ZOOM, target)
        echo = self.p.read_frame(2.0)
        self.verify_echo(echo, CMD_SET_ZOOM, target)

        resps = self.p.collect_responses(timeout=30.0)
        resp_names = [rsp_str(r) for r in resps]
        print(f"  Received: {resp_names}")

        has_zoom = any(r['cmd'] == RSP_ZOOM for r in resps)
        has_arrived = any(r['cmd'] == RSP_ARRIVED for r in resps)
        has_overcurrent = any(r['cmd'] == RSP_OVERCURRENT for r in resps)

        self.check(has_zoom, "ZOOM response received")
        self.check(has_arrived, "ARRIVED response received")
        self.check(not has_overcurrent, "no OVERCURRENT")

        time.sleep(0.5)

        # Test: move to mid zoom
        mid = (min_zoom + max_zoom) // 2
        print(f"\n--- 3.3 SET_ZOOM to {mid/10:.1f}x ---")
        self.p.send(CMD_SET_ZOOM, mid)
        echo = self.p.read_frame(2.0)
        self.verify_echo(echo, CMD_SET_ZOOM, mid)

        resps = self.p.collect_responses(timeout=30.0)
        resp_names = [rsp_str(r) for r in resps]
        print(f"  Received: {resp_names}")

        has_arrived = any(r['cmd'] == RSP_ARRIVED for r in resps)
        self.check(has_arrived, "ARRIVED response received")

    # ─── ZOOM_INC / ZOOM_DEC ───

    def test_zoom_inc_dec(self):
        print("\n" + "=" * 55)
        print("Section 4: ZOOM_INC/DEC (0x11, 0x12) — MOTOR WILL MOVE")
        print("=" * 55)

        # ZOOM_INC by 2 (0.2x)
        print("\n--- 4.1 ZOOM_INC +2 ---")
        self.p.send(CMD_ZOOM_INC, 2)
        echo = self.p.read_frame(2.0)
        self.verify_echo(echo, CMD_ZOOM_INC, 2)

        resps = self.p.collect_responses(timeout=15.0)
        resp_names = [rsp_str(r) for r in resps]
        print(f"  Received: {resp_names}")
        has_arrived = any(r['cmd'] == RSP_ARRIVED for r in resps)
        self.check(has_arrived, "ARRIVED response received")

        time.sleep(0.5)

        # ZOOM_DEC by 2 (0.2x)
        print("\n--- 4.2 ZOOM_DEC -2 ---")
        self.p.send(CMD_ZOOM_DEC, 2)
        echo = self.p.read_frame(2.0)
        self.verify_echo(echo, CMD_ZOOM_DEC, 2)

        resps = self.p.collect_responses(timeout=15.0)
        resp_names = [rsp_str(r) for r in resps]
        print(f"  Received: {resp_names}")
        has_arrived = any(r['cmd'] == RSP_ARRIVED for r in resps)
        self.check(has_arrived, "ARRIVED response received")

    # ─── BUSY rejection ───

    def test_busy_rejection(self):
        print("\n" + "=" * 55)
        print("Section 5: BUSY Rejection")
        print("=" * 55)

        # Start a long move
        _, resps = self.p.send_recv_multi(CMD_QUERY_RANGE, n=2)
        max_zoom = resps[1]['param'] if resps[1] else 60

        print("\n--- 5.1 Send SET_ZOOM (long move), then another SET_ZOOM ---")
        self.p.send(CMD_SET_ZOOM, max_zoom)
        echo = self.p.read_frame(2.0)
        self.verify_echo(echo, CMD_SET_ZOOM, max_zoom)

        time.sleep(0.3)  # let motor start

        # Send another motion command while busy
        echo2, resp2 = self.p.send_recv(CMD_SET_ZOOM, max_zoom)
        self.verify_echo(echo2, CMD_SET_ZOOM, max_zoom, "busy echo")
        self.check(resp2 is not None and resp2['cmd'] == RSP_REQ_INVALID,
                   f"BUSY rejection: {rsp_str(resp2)}")

        # Queries should still work during BUSY
        print("\n--- 5.2 Queries during BUSY ---")
        _, qr = self.p.send_recv(CMD_QUERY_STATUS)
        self.check(qr is not None and qr['cmd'] == RSP_STATUS,
                   f"query during BUSY: {rsp_str(qr)}")

        # Wait for move to complete
        resps = self.p.collect_responses(timeout=30.0)
        has_arrived = any(r['cmd'] == RSP_ARRIVED for r in resps)
        self.check(has_arrived, "original move completed")

    # ─── FORCE_STOP ───

    def test_force_stop(self):
        print("\n" + "=" * 55)
        print("Section 6: FORCE_STOP (0x02)")
        print("=" * 55)

        # Get range for move
        _, resps = self.p.send_recv_multi(CMD_QUERY_RANGE, n=2)
        min_zoom = resps[0]['param'] if resps[0] else 6

        # Start a move to min (long distance from current)
        print("\n--- 6.1 Start move, then FORCE_STOP ---")
        self.p.send(CMD_SET_ZOOM, min_zoom)
        echo = self.p.read_frame(2.0)
        self.verify_echo(echo, CMD_SET_ZOOM, min_zoom)

        time.sleep(1.0)  # let motor run for 1s

        # Force stop
        echo2, resp2 = self.p.send_recv(CMD_FORCE_STOP)
        self.verify_echo(echo2, CMD_FORCE_STOP, 0, "FORCE_STOP echo")
        self.check(resp2 is not None and resp2['cmd'] == CMD_FORCE_STOP,
                   f"FORCE_STOP ACK: {rsp_str(resp2)}")

        time.sleep(0.5)

        # Verify system is READY after force stop
        _, status = self.p.send_recv(CMD_QUERY_STATUS)
        if status:
            print(f"  Post-FORCE_STOP status: {rsp_str(status)}")

    # ─── CYCLE ───

    def test_cycle(self):
        print("\n" + "=" * 55)
        print("Section 7: CYCLE_START/STOP (0x30, 0x31)")
        print("=" * 55)

        # Cycle: step=+1, dwell=5 (500ms)
        # param format: (step << 8) | dwell_x100ms
        param = (1 << 8) | 5  # step=+1, dwell=500ms
        print(f"\n--- 7.1 CYCLE_START step=+1 dwell=500ms (run 5s, then stop) ---")
        self.p.send(CMD_CYCLE_START, param)
        echo = self.p.read_frame(2.0)
        self.verify_echo(echo, CMD_CYCLE_START, param)

        # Let cycle run for 5 seconds
        time.sleep(5.0)

        # Stop cycle
        echo2, resp2 = self.p.send_recv(CMD_CYCLE_STOP)
        self.verify_echo(echo2, CMD_CYCLE_STOP, 0, "CYCLE_STOP echo")
        self.check(resp2 is not None and resp2['cmd'] == CMD_CYCLE_STOP,
                   f"CYCLE_STOP ACK: {rsp_str(resp2)}")

        # Drain any remaining responses
        self.p.collect_responses(timeout=2.0)

    # ─── Self-Test ───

    def test_self_test(self):
        print("\n" + "=" * 55)
        print("Section 8: Self-Test (0x60)")
        print("=" * 55)

        echo, resp = self.p.send_recv(CMD_SELF_TEST)
        self.verify_echo(echo, CMD_SELF_TEST, 0)
        self.check(resp is not None and resp['cmd'] == CMD_SELF_TEST,
                   f"SELF_TEST ACK: {rsp_str(resp)}")

    # ─── Run all ───

    def run_all(self):
        print("=" * 55)
        print("ZLENS_DC Full Command Test — Protocol v2.5")
        print("WARNING: Motor WILL move during this test!")
        print("=" * 55)

        self.test_queries()
        self.test_homing()
        self.test_set_zoom()
        self.test_zoom_inc_dec()
        self.test_busy_rejection()
        self.test_force_stop()
        self.test_cycle()
        self.test_self_test()

        # Final summary
        print("\n" + "=" * 55)
        total = self.passed + self.failed
        print(f"Results: {self.passed} passed, {self.failed} failed / {total} total")
        if self.errors:
            print("\nFailed tests:")
            for e in self.errors:
                print(f"  - {e}")
        if self.failed == 0:
            print("\n>>> ALL PASSED <<<")
        else:
            print("\n>>> SOME TESTS FAILED <<<")
        print("=" * 55)
        return self.failed == 0


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    print(f"Opening {port} @ {baud}...")
    proto = Proto(port, baud)
    time.sleep(0.5)
    try:
        test = FullTest(proto)
        ok = test.run_all()
    finally:
        proto.close()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
