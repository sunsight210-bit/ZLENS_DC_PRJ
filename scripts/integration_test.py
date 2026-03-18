#!/usr/bin/env python3
"""ZLENS_DC 端到端集成测试 — 通过串口验证完整命令链路"""

import argparse
import struct
import sys
import time

import serial


# ─── CRC16/MODBUS (与固件 crc16_modbus 精确匹配) ───

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


# ─── 协议常量 ───

FRAME_HEADER = 0xA5
WORK_FRAME_SIZE = 6

CMD_HANDSHAKE    = 0x01
CMD_HOMING       = 0x02
CMD_FORCE_STOP   = 0x03
CMD_SET_ZOOM     = 0x10
CMD_CYCLE_START  = 0x11
CMD_CYCLE_STOP   = 0x12
CMD_QUERY_ZOOM   = 0x20
CMD_QUERY_STATUS = 0x21
CMD_QUERY_RANGE  = 0x22
CMD_QUERY_TYPE   = 0x23
CMD_SWITCH_FACTORY = 0x30
CMD_SELF_TEST    = 0x60

RSP_OK   = 0x0000
RSP_BUSY = 0xFFFF
LENS_TYPE = 0x0004


# ─── 协议类 ───

class ZlensProtocol:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 2.0):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def build_frame(self, cmd: int, param: int) -> bytes:
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
        return {'cmd': cmd, 'param': param}

    def send_command(self, cmd: int, param: int = 0):
        frame = self.build_frame(cmd, param)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()

    def receive_response(self, timeout: float = 2.0):
        self.ser.timeout = timeout
        data = self.ser.read(WORK_FRAME_SIZE)
        if len(data) < WORK_FRAME_SIZE:
            return None
        return self.parse_frame(data)

    def send_and_receive(self, cmd: int, param: int = 0, timeout: float = 2.0):
        self.send_command(cmd, param)
        return self.receive_response(timeout)

    def wait_for_response(self, expected_cmd: int, timeout: float = 120.0):
        """等待指定命令的响应，忽略其他帧"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            remaining = deadline - time.time()
            resp = self.receive_response(timeout=min(remaining, 2.0))
            if resp and resp['cmd'] == expected_cmd:
                return resp
        return None


# ─── 测试用例 ───

class IntegrationTest:
    def __init__(self, proto: ZlensProtocol, skip_homing: bool = False,
                 power_down: bool = False):
        self.proto = proto
        self.skip_homing = skip_homing
        self.power_down = power_down
        self.passed = 0
        self.failed = 0
        self.skipped = 0

    def _assert(self, condition: bool, msg: str):
        if condition:
            self.passed += 1
            print(f"  [PASS] {msg}")
        else:
            self.failed += 1
            print(f"  [FAIL] {msg}")

    def _skip(self, msg: str):
        self.skipped += 1
        print(f"  [SKIP] {msg}")

    def test_handshake(self):
        print("\n--- test_handshake ---")
        resp = self.proto.send_and_receive(CMD_HANDSHAKE, 0x1234)
        self._assert(resp is not None, "got response")
        if resp:
            self._assert(resp['cmd'] == CMD_HANDSHAKE, f"cmd=0x{resp['cmd']:02X} == 0x01")
            self._assert(resp['param'] == 0x1234, f"param=0x{resp['param']:04X} == 0x1234")

    def test_query_status(self):
        print("\n--- test_query_status ---")
        resp = self.proto.send_and_receive(CMD_QUERY_STATUS)
        self._assert(resp is not None, "got response")
        if resp:
            self._assert(resp['cmd'] == CMD_QUERY_STATUS, f"cmd=0x{resp['cmd']:02X}")
            self._assert(resp['param'] == 0x0000, f"status=0x{resp['param']:04X} (READY)")

    def test_query_type(self):
        print("\n--- test_query_type ---")
        resp = self.proto.send_and_receive(CMD_QUERY_TYPE)
        self._assert(resp is not None, "got response")
        if resp:
            self._assert(resp['cmd'] == CMD_QUERY_TYPE, f"cmd=0x{resp['cmd']:02X}")
            self._assert(resp['param'] == LENS_TYPE, f"type=0x{resp['param']:04X} == 0x0004")

    def test_homing(self):
        print("\n--- test_homing ---")
        if self.skip_homing:
            self._skip("--skip-homing specified")
            return
        self.proto.send_command(CMD_HOMING)
        resp = self.proto.wait_for_response(CMD_HOMING, timeout=120.0)
        self._assert(resp is not None, "homing response received")
        if resp:
            self._assert(resp['param'] == RSP_OK,
                         f"result=0x{resp['param']:04X} == OK")

    def test_query_range(self):
        print("\n--- test_query_range ---")
        if self.skip_homing:
            self._skip("requires homing first")
            return
        self.proto.send_command(CMD_QUERY_RANGE)
        resp1 = self.proto.receive_response(timeout=2.0)
        resp2 = self.proto.receive_response(timeout=2.0)
        self._assert(resp1 is not None, "frame 1 (min zoom)")
        self._assert(resp2 is not None, "frame 2 (max zoom)")
        if resp1 and resp2:
            self._assert(resp1['cmd'] == CMD_QUERY_RANGE, "cmd matches")
            print(f"    range: {resp1['param']/10:.1f}x ~ {resp2['param']/10:.1f}x")

    def test_query_zoom(self):
        print("\n--- test_query_zoom ---")
        resp = self.proto.send_and_receive(CMD_QUERY_ZOOM)
        self._assert(resp is not None, "got response")
        if resp:
            self._assert(resp['cmd'] == CMD_QUERY_ZOOM, f"cmd=0x{resp['cmd']:02X}")
            print(f"    current zoom: {resp['param']/10:.1f}x")

    def test_set_zoom(self):
        print("\n--- test_set_zoom ---")
        if self.skip_homing:
            self._skip("requires homing first")
            return
        target = 60  # 6.0x (within 0.6x~7.0x range)
        self.proto.send_command(CMD_SET_ZOOM, target)
        resp = self.proto.wait_for_response(CMD_SET_ZOOM, timeout=30.0)
        self._assert(resp is not None, "set_zoom response received")
        if resp:
            self._assert(resp['param'] == target,
                         f"zoom={resp['param']/10:.1f}x == 6.0x")
        # Verify with query
        time.sleep(0.5)
        qr = self.proto.send_and_receive(CMD_QUERY_ZOOM)
        if qr:
            self._assert(qr['param'] == target,
                         f"query confirms zoom={qr['param']/10:.1f}x")

    def test_busy_rejection(self):
        print("\n--- test_busy_rejection ---")
        if self.skip_homing:
            self._skip("requires homing first")
            return
        # Send a long move (6.0x → 0.6x, far enough to stay busy)
        target1 = 6  # 0.6x
        self.proto.send_command(CMD_SET_ZOOM, target1)
        time.sleep(0.5)  # let motor start
        # Send second command while busy
        resp = self.proto.send_and_receive(CMD_SET_ZOOM, 60, timeout=2.0)
        self._assert(resp is not None, "got busy rejection")
        if resp:
            self._assert(resp['param'] == RSP_BUSY,
                         f"param=0x{resp['param']:04X} == BUSY")
        # Wait for first move to complete
        self.proto.wait_for_response(CMD_SET_ZOOM, timeout=30.0)

    def test_force_stop(self):
        print("\n--- test_force_stop ---")
        if self.skip_homing:
            self._skip("requires homing first")
            return
        # Start a long move (to 7.0x from wherever we are)
        self.proto.send_command(CMD_SET_ZOOM, 70)
        time.sleep(0.5)
        # Force stop
        self.proto.send_command(CMD_FORCE_STOP)
        resp = self.proto.wait_for_response(CMD_FORCE_STOP, timeout=5.0)
        self._assert(resp is not None, "force_stop response")
        # Wait a bit, then verify motor stopped
        time.sleep(1.0)
        sr = self.proto.send_and_receive(CMD_QUERY_STATUS)
        if sr:
            self._assert(sr['param'] == 0x0000,
                         f"status=READY after stop")

    def test_self_test_command(self):
        print("\n--- test_self_test_command ---")
        resp = self.proto.send_and_receive(CMD_SELF_TEST, 0, timeout=2.0)
        self._assert(resp is not None, "got self_test ACK")
        if resp:
            self._assert(resp['cmd'] == CMD_SELF_TEST,
                         f"cmd=0x{resp['cmd']:02X}")
            self._assert(resp['param'] == RSP_OK,
                         f"param=0x{resp['param']:04X} == OK")

    def test_power_down_recovery(self):
        print("\n--- test_power_down_recovery ---")
        if not self.power_down:
            self._skip("--power-down not specified (requires manual power cycle)")
            return
        # Move to a known position first
        if not self.skip_homing:
            self.proto.send_command(CMD_SET_ZOOM, 80)
            self.proto.wait_for_response(CMD_SET_ZOOM, timeout=30.0)

        print("    >>> PULL POWER NOW, then reconnect within 30s <<<")
        self.proto.close()
        input("    Press Enter after power cycle and reconnection...")

        # Reconnect
        port = self.proto.ser.port
        baudrate = self.proto.ser.baudrate
        time.sleep(3.0)
        self.proto = ZlensProtocol(port, baudrate, timeout=5.0)

        # Wait for boot
        time.sleep(5.0)
        resp = self.proto.send_and_receive(CMD_QUERY_STATUS, timeout=5.0)
        self._assert(resp is not None, "device responds after power cycle")
        if resp:
            self._assert(resp['param'] == 0x0000,
                         f"status=READY after recovery")

    def run_all(self):
        print("=" * 50)
        print("ZLENS_DC Integration Test")
        print("=" * 50)

        self.test_handshake()
        self.test_query_status()
        self.test_query_type()
        self.test_homing()
        self.test_query_range()
        self.test_query_zoom()
        self.test_set_zoom()
        self.test_busy_rejection()
        self.test_force_stop()
        self.test_self_test_command()
        self.test_power_down_recovery()

        print("\n" + "=" * 50)
        total = self.passed + self.failed + self.skipped
        print(f"Results: {self.passed} passed, {self.failed} failed, "
              f"{self.skipped} skipped / {total} total")
        print("=" * 50)
        return self.failed == 0


def main():
    parser = argparse.ArgumentParser(description="ZLENS_DC integration test")
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="Serial port (default: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--skip-homing", action="store_true",
                        help="Skip homing and motion tests")
    parser.add_argument("--power-down", action="store_true",
                        help="Include power-down recovery test (manual)")
    args = parser.parse_args()

    proto = ZlensProtocol(args.port, args.baud)
    try:
        test = IntegrationTest(proto, args.skip_homing, args.power_down)
        ok = test.run_all()
    finally:
        proto.close()

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
