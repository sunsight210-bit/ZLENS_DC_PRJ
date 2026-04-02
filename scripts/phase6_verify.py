#!/usr/bin/env python3
"""Phase 6 板上验证 — 协议 v2.5 命令/响应格式验证

仅发送查询类命令，不触发电机运动。
验证项：握手回显、查询响应 cmd=0x1X、BUSY 格式、堵转计数等。
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

# Commands (cmd namespace)
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
CMD_SELF_TEST     = 0x65
CMD_SWITCH_FACTORY = 0xFA

# Response commands (rsp_cmd namespace)
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
RSP_ERR_PARAM   = 0xE0
RSP_STALL_STOP  = 0xE1
RSP_OVERCURRENT = 0xE2
RSP_STALL_COUNT = 0xE3

# Response param constants
LENS_TYPE          = 0x0004
FW_VERSION         = 0x1000  # v1.000
DEFAULT_SPEED_DUTY = 188
REQ_INVALID_PARAM  = 0x000E


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

    def send_and_receive_with_echo(self, cmd: int, param: int = 0,
                                   timeout: float = 2.0):
        """Send command, expect echo + response. Returns (echo, response)."""
        frame = self.build_frame(cmd, param)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        echo = self.read_frame(timeout)
        resp = self.read_frame(timeout)
        return echo, resp

    def send_and_receive_with_echo_multi(self, cmd: int, param: int = 0,
                                         n_responses: int = 2,
                                         timeout: float = 2.0):
        """Send command, expect echo + N response frames."""
        frame = self.build_frame(cmd, param)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        echo = self.read_frame(timeout)
        responses = []
        for _ in range(n_responses):
            r = self.read_frame(timeout)
            responses.append(r)
        return echo, responses


# ─── Test runner ───

class Phase6Verify:
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

    def verify_echo(self, echo, sent_cmd: int, sent_param: int, label: str):
        """Verify handshake echo matches sent frame."""
        self.check(echo is not None, f"{label}: echo received")
        if echo:
            self.check(echo['cmd'] == sent_cmd,
                       f"{label}: echo cmd=0x{echo['cmd']:02X} == 0x{sent_cmd:02X}")
            self.check(echo['param'] == sent_param,
                       f"{label}: echo param=0x{echo['param']:04X} == 0x{sent_param:04X}")

    # --- Individual tests ---

    def test_query_zoom(self):
        print("\n--- 1. Query Zoom (0x20) → rsp cmd=0x10 ---")
        echo, resp = self.proto.send_and_receive_with_echo(CMD_QUERY_ZOOM)
        self.verify_echo(echo, CMD_QUERY_ZOOM, 0, "echo")
        self.check(resp is not None, "response received")
        if resp:
            self.check(resp['cmd'] == RSP_ZOOM,
                       f"rsp cmd=0x{resp['cmd']:02X} == 0x10 (ZOOM)")
            print(f"    zoom = {resp['param'] / 10:.1f}x")

    def test_query_status(self):
        print("\n--- 2. Query Status (0x21) → rsp cmd=0x11 ---")
        echo, resp = self.proto.send_and_receive_with_echo(CMD_QUERY_STATUS)
        self.verify_echo(echo, CMD_QUERY_STATUS, 0, "echo")
        self.check(resp is not None, "response received")
        if resp:
            self.check(resp['cmd'] == RSP_STATUS,
                       f"rsp cmd=0x{resp['cmd']:02X} == 0x11 (STATUS)")
            print(f"    status = 0x{resp['param']:04X}")

    def test_query_speed(self):
        print("\n--- 3. Query Speed (0x22) → rsp cmd=0x12 ---")
        echo, resp = self.proto.send_and_receive_with_echo(CMD_QUERY_SPEED)
        self.verify_echo(echo, CMD_QUERY_SPEED, 0, "echo")
        self.check(resp is not None, "response received")
        if resp:
            self.check(resp['cmd'] == RSP_SPEED,
                       f"rsp cmd=0x{resp['cmd']:02X} == 0x12 (SPEED)")
            self.check(resp['param'] == DEFAULT_SPEED_DUTY,
                       f"speed_duty = {resp['param']} == {DEFAULT_SPEED_DUTY}")

    def test_query_type(self):
        print("\n--- 4. Query Type (0x23) → rsp cmd=0x13 ---")
        echo, resp = self.proto.send_and_receive_with_echo(CMD_QUERY_TYPE)
        self.verify_echo(echo, CMD_QUERY_TYPE, 0, "echo")
        self.check(resp is not None, "response received")
        if resp:
            self.check(resp['cmd'] == RSP_TYPE,
                       f"rsp cmd=0x{resp['cmd']:02X} == 0x13 (TYPE)")
            self.check(resp['param'] == LENS_TYPE,
                       f"type = 0x{resp['param']:04X} == 0x{LENS_TYPE:04X}")

    def test_query_range(self):
        print("\n--- 5. Query Range (0x24) → rsp 0x14 + 0x15 (two frames) ---")
        echo, resps = self.proto.send_and_receive_with_echo_multi(
            CMD_QUERY_RANGE, n_responses=2)
        self.verify_echo(echo, CMD_QUERY_RANGE, 0, "echo")
        r1, r2 = resps[0], resps[1]
        self.check(r1 is not None, "frame 1 (MIN_ZOOM) received")
        self.check(r2 is not None, "frame 2 (MAX_ZOOM) received")
        if r1:
            self.check(r1['cmd'] == RSP_MIN_ZOOM,
                       f"frame1 cmd=0x{r1['cmd']:02X} == 0x14 (MIN_ZOOM)")
            print(f"    min zoom = {r1['param'] / 10:.1f}x")
        if r2:
            self.check(r2['cmd'] == RSP_MAX_ZOOM,
                       f"frame2 cmd=0x{r2['cmd']:02X} == 0x15 (MAX_ZOOM)")
            print(f"    max zoom = {r2['param'] / 10:.1f}x")

    def test_query_version(self):
        print("\n--- 6. Query Version (0x25) → rsp cmd=0x16 ---")
        echo, resp = self.proto.send_and_receive_with_echo(CMD_QUERY_VERSION)
        self.verify_echo(echo, CMD_QUERY_VERSION, 0, "echo")
        self.check(resp is not None, "response received")
        if resp:
            self.check(resp['cmd'] == RSP_VERSION,
                       f"rsp cmd=0x{resp['cmd']:02X} == 0x16 (VERSION)")
            self.check(resp['param'] == FW_VERSION,
                       f"version = 0x{resp['param']:04X} == 0x{FW_VERSION:04X}")

    def test_get_stall_count(self):
        print("\n--- 7. Get Stall Count (0x03) → rsp cmd=0xE3 ---")
        echo, resp = self.proto.send_and_receive_with_echo(CMD_GET_STALL_CNT)
        self.verify_echo(echo, CMD_GET_STALL_CNT, 0, "echo")
        self.check(resp is not None, "response received")
        if resp:
            self.check(resp['cmd'] == RSP_STALL_COUNT,
                       f"rsp cmd=0x{resp['cmd']:02X} == 0xE3 (STALL_COUNT)")
            print(f"    stall count = {resp['param']}")

    def test_self_test_command(self):
        print("\n--- 8. Self-Test (0x65) → echo + forwarded to MotorTask ---")
        echo, resp = self.proto.send_and_receive_with_echo(CMD_SELF_TEST)
        self.verify_echo(echo, CMD_SELF_TEST, 0, "echo")
        # 0x65 is now forwarded to MotorTask (triggers full diagnostics)
        # Response depends on motor state; just verify echo was correct

    def run_all(self):
        print("=" * 55)
        print("ZLENS_DC Phase 6 — Protocol v2.5 Board Verification")
        print("=" * 55)
        print("仅查询命令，不触发电机运动")

        self.test_query_zoom()
        self.test_query_status()
        self.test_query_speed()
        self.test_query_type()
        self.test_query_range()
        self.test_query_version()
        self.test_get_stall_count()
        self.test_self_test_command()

        print("\n" + "=" * 55)
        total = self.passed + self.failed
        print(f"Results: {self.passed} passed, {self.failed} failed / {total} total")
        if self.failed == 0:
            print(">>> ALL PASSED <<<")
        else:
            print(">>> SOME TESTS FAILED <<<")
        print("=" * 55)
        return self.failed == 0


def main():
    parser = argparse.ArgumentParser(
        description="ZLENS_DC Phase 6 protocol v2.5 board verification")
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud}...")
    proto = ZlensProtocol(args.port, args.baud)
    time.sleep(0.5)  # let serial settle

    try:
        test = Phase6Verify(proto)
        ok = test.run_all()
    finally:
        proto.close()

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
