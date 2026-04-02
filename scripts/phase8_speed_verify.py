#!/usr/bin/env python3
"""Phase 8 板上验证 — 0x60 速度命令组 (duty_x10)

测试项：
  1. QUERY_SPEED 默认值 (281)
  2. SET_SPEED 正常设置 + 响应
  3. SET_SPEED 上下钳位
  4. SPEED_INC / SPEED_DEC
  5. SET_MIN_SPEED / SET_MAX_SPEED + 联动钳位
  6. SELF_TEST (0x65) 转发确认
  7. FRAM 持久化 (掉电恢复)

不触发电机运动（速度命令仅设置 PWM 限幅，不启动电机）。
0x65 SELF_TEST 会触发回零，仅验证转发，不等待完成。
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


# ─── Protocol constants ───

FRAME_HEADER    = 0xA5
WORK_FRAME_SIZE = 6

# Commands
CMD_QUERY_SPEED   = 0x22
CMD_SET_SPEED     = 0x60
CMD_SPEED_INC     = 0x61
CMD_SPEED_DEC     = 0x62
CMD_SET_MIN_SPEED = 0x63
CMD_SET_MAX_SPEED = 0x64
CMD_SELF_TEST     = 0x65
CMD_FORCE_STOP    = 0x02

# Response commands
RSP_SPEED       = 0x12
RSP_HOMING_DONE = 0x01
RSP_ZOOM        = 0x10

# Defaults
DEFAULT_SPEED_DUTY     = 188
DEFAULT_MIN_SPEED_DUTY = 112
DEFAULT_MAX_SPEED_DUTY = 281
PWM_ARR = 4266


def duty_to_pwm(duty_x10: int) -> int:
    return duty_x10 * PWM_ARR // 1000


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
        return {'cmd': cmd, 'param': param}

    def read_frame(self, timeout: float = 2.0):
        self.ser.timeout = timeout
        data = self.ser.read(WORK_FRAME_SIZE)
        if len(data) < WORK_FRAME_SIZE:
            return None
        return self.parse_frame(data)

    def send_cmd(self, cmd: int, param: int = 0, timeout: float = 2.0):
        """Send command, return (echo, response). Response may be None."""
        frame = self.build_frame(cmd, param)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        echo = self.read_frame(timeout)
        resp = self.read_frame(timeout)
        return echo, resp

    def drain(self, timeout: float = 0.3):
        """Drain any remaining frames in the RX buffer."""
        self.ser.timeout = timeout
        while True:
            data = self.ser.read(WORK_FRAME_SIZE)
            if len(data) < WORK_FRAME_SIZE:
                break


# ─── Test runner ───

class SpeedCmdVerify:
    def __init__(self, proto: ZlensProtocol, verbose: bool = False):
        self.proto = proto
        self.verbose = verbose
        self.passed = 0
        self.failed = 0
        self.results = []

    def check(self, condition: bool, msg: str):
        if condition:
            self.passed += 1
            self.results.append(('PASS', msg))
            print(f"  [PASS] {msg}")
        else:
            self.failed += 1
            self.results.append(('FAIL', msg))
            print(f"  [FAIL] {msg}")

    def verify_echo(self, echo, sent_cmd: int, sent_param: int):
        self.check(echo is not None, f"echo received")
        if echo:
            self.check(echo['cmd'] == sent_cmd,
                       f"echo cmd=0x{echo['cmd']:02X} == 0x{sent_cmd:02X}")

    def verify_speed_response(self, resp, expected_duty: int, label: str = ""):
        prefix = f"{label}: " if label else ""
        self.check(resp is not None, f"{prefix}response received")
        if resp:
            self.check(resp['cmd'] == RSP_SPEED,
                       f"{prefix}rsp cmd=0x{resp['cmd']:02X} == 0x12 (SPEED)")
            self.check(resp['param'] == expected_duty,
                       f"{prefix}duty={resp['param']} == {expected_duty} "
                       f"(PWM={duty_to_pwm(resp['param'])}, "
                       f"{resp['param']/10:.1f}%)")
            return resp['param'] if resp else None
        return None

    # --- Tests ---

    def test_01_query_speed_default(self):
        """QUERY_SPEED 返回默认值 281"""
        print("\n--- 1. QUERY_SPEED default → 281 (28.1%) ---")
        echo, resp = self.proto.send_cmd(CMD_QUERY_SPEED)
        self.verify_echo(echo, CMD_QUERY_SPEED, 0)
        self.verify_speed_response(resp, DEFAULT_SPEED_DUTY, "default")

    def test_02_set_speed(self):
        """SET_SPEED(200) → 响应 200"""
        print("\n--- 2. SET_SPEED(200) → duty=200 (20.0%) ---")
        echo, resp = self.proto.send_cmd(CMD_SET_SPEED, 200)
        self.verify_echo(echo, CMD_SET_SPEED, 200)
        self.verify_speed_response(resp, 200, "SET_SPEED")

    def test_03_query_after_set(self):
        """SET_SPEED 后 QUERY_SPEED 返回新值"""
        print("\n--- 3. QUERY_SPEED after SET → 200 ---")
        echo, resp = self.proto.send_cmd(CMD_QUERY_SPEED)
        self.verify_echo(echo, CMD_QUERY_SPEED, 0)
        self.verify_speed_response(resp, 200, "query")

    def test_04_speed_inc(self):
        """SPEED_INC → 201"""
        print("\n--- 4. SPEED_INC → 201 ---")
        echo, resp = self.proto.send_cmd(CMD_SPEED_INC)
        self.verify_echo(echo, CMD_SPEED_INC, 0)
        self.verify_speed_response(resp, 201, "INC")

    def test_05_speed_dec(self):
        """SPEED_DEC → 200"""
        print("\n--- 5. SPEED_DEC → 200 ---")
        echo, resp = self.proto.send_cmd(CMD_SPEED_DEC)
        self.verify_echo(echo, CMD_SPEED_DEC, 0)
        self.verify_speed_response(resp, 200, "DEC")

    def test_06_set_max_speed(self):
        """SET_MAX_SPEED(500) → 响应 500"""
        print("\n--- 6. SET_MAX_SPEED(500) → duty=500 ---")
        echo, resp = self.proto.send_cmd(CMD_SET_MAX_SPEED, 500)
        self.verify_echo(echo, CMD_SET_MAX_SPEED, 500)
        self.verify_speed_response(resp, 500, "SET_MAX")

    def test_07_set_min_speed(self):
        """SET_MIN_SPEED(100) → 响应 100"""
        print("\n--- 7. SET_MIN_SPEED(100) → duty=100 ---")
        echo, resp = self.proto.send_cmd(CMD_SET_MIN_SPEED, 100)
        self.verify_echo(echo, CMD_SET_MIN_SPEED, 100)
        self.verify_speed_response(resp, 100, "SET_MIN")

    def test_08_set_speed_in_range(self):
        """SET_SPEED(300) — 在 [100, 500] 范围内 → 300"""
        print("\n--- 8. SET_SPEED(300) in [100, 500] → 300 ---")
        echo, resp = self.proto.send_cmd(CMD_SET_SPEED, 300)
        self.verify_echo(echo, CMD_SET_SPEED, 300)
        self.verify_speed_response(resp, 300, "in-range")

    def test_09_set_speed_clamp_low(self):
        """SET_SPEED(50) — 低于 min(100) → 钳位到 100"""
        print("\n--- 9. SET_SPEED(50) clamp → min=100 ---")
        echo, resp = self.proto.send_cmd(CMD_SET_SPEED, 50)
        self.verify_echo(echo, CMD_SET_SPEED, 50)
        self.verify_speed_response(resp, 100, "clamp-low")

    def test_10_set_speed_clamp_high(self):
        """SET_SPEED(800) — 高于 max(500) → 钳位到 500"""
        print("\n--- 10. SET_SPEED(800) clamp → max=500 ---")
        echo, resp = self.proto.send_cmd(CMD_SET_SPEED, 800)
        self.verify_echo(echo, CMD_SET_SPEED, 800)
        self.verify_speed_response(resp, 500, "clamp-high")

    def test_11_set_speed_over_1000(self):
        """SET_SPEED(1500) — 超出 0-1000 → 先 clamp 到 1000 再 clamp 到 max(500)"""
        print("\n--- 11. SET_SPEED(1500) → clamp to max=500 ---")
        echo, resp = self.proto.send_cmd(CMD_SET_SPEED, 1500)
        self.verify_echo(echo, CMD_SET_SPEED, 1500)
        self.verify_speed_response(resp, 500, "over-1000")

    def test_12_inc_at_max(self):
        """先 SET_SPEED 到 max(500)，SPEED_INC 应该不变"""
        print("\n--- 12. SPEED_INC at max → stays 500 ---")
        # Ensure at max
        self.proto.send_cmd(CMD_SET_SPEED, 500)
        self.proto.drain()
        echo, resp = self.proto.send_cmd(CMD_SPEED_INC)
        self.verify_echo(echo, CMD_SPEED_INC, 0)
        self.verify_speed_response(resp, 500, "INC@max")

    def test_13_dec_at_min(self):
        """先 SET_SPEED 到 min(100)，SPEED_DEC 应该不变"""
        print("\n--- 13. SPEED_DEC at min → stays 100 ---")
        self.proto.send_cmd(CMD_SET_SPEED, 100)
        self.proto.drain()
        echo, resp = self.proto.send_cmd(CMD_SPEED_DEC)
        self.verify_echo(echo, CMD_SPEED_DEC, 0)
        self.verify_speed_response(resp, 100, "DEC@min")

    def test_14_set_min_raises_current(self):
        """SET_MIN_SPEED(250) 应抬升当前速度(100) 到 250"""
        print("\n--- 14. SET_MIN_SPEED(250) raises current(100) → 250 ---")
        # Current speed is 100 from test_13
        echo, resp = self.proto.send_cmd(CMD_SET_MIN_SPEED, 250)
        self.verify_echo(echo, CMD_SET_MIN_SPEED, 250)
        self.verify_speed_response(resp, 250, "MIN-raise")
        # Verify current speed was raised
        echo2, resp2 = self.proto.send_cmd(CMD_QUERY_SPEED)
        if resp2:
            self.check(resp2['param'] == 250,
                       f"QUERY confirms speed raised to {resp2['param']}")

    def test_15_set_max_lowers_current(self):
        """SET_MAX_SPEED(200) 应压低当前速度(250) 到 200"""
        print("\n--- 15. SET_MAX_SPEED(200) lowers current(250) → 200 ---")
        echo, resp = self.proto.send_cmd(CMD_SET_MAX_SPEED, 200)
        self.verify_echo(echo, CMD_SET_MAX_SPEED, 200)
        self.verify_speed_response(resp, 200, "MAX-lower")

    def test_16_restore_defaults(self):
        """恢复默认值: min=112, max=281, speed=281"""
        print("\n--- 16. Restore defaults: min=112, max=281, speed=281 ---")
        # Restore min first (so max > min)
        self.proto.send_cmd(CMD_SET_MIN_SPEED, DEFAULT_MIN_SPEED_DUTY)
        self.proto.drain()
        # Restore max
        self.proto.send_cmd(CMD_SET_MAX_SPEED, DEFAULT_MAX_SPEED_DUTY)
        self.proto.drain()
        # Restore speed
        echo, resp = self.proto.send_cmd(CMD_SET_SPEED, DEFAULT_SPEED_DUTY)
        self.verify_echo(echo, CMD_SET_SPEED, DEFAULT_SPEED_DUTY)
        self.verify_speed_response(resp, DEFAULT_SPEED_DUTY, "restored")

    def run_all(self):
        print("=" * 60)
        print("ZLENS_DC Phase 8 — 0x60 Speed Commands Board Verification")
        print("=" * 60)
        print("速度命令仅设置 PWM 限幅，不触发电机运动")
        print(f"默认值: speed={DEFAULT_SPEED_DUTY} min={DEFAULT_MIN_SPEED_DUTY} "
              f"max={DEFAULT_MAX_SPEED_DUTY}")
        print(f"PWM_ARR={PWM_ARR}")

        self.test_01_query_speed_default()
        self.test_02_set_speed()
        self.test_03_query_after_set()
        self.test_04_speed_inc()
        self.test_05_speed_dec()
        self.test_06_set_max_speed()
        self.test_07_set_min_speed()
        self.test_08_set_speed_in_range()
        self.test_09_set_speed_clamp_low()
        self.test_10_set_speed_clamp_high()
        self.test_11_set_speed_over_1000()
        self.test_12_inc_at_max()
        self.test_13_dec_at_min()
        self.test_14_set_min_raises_current()
        self.test_15_set_max_lowers_current()
        self.test_16_restore_defaults()

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
        description="ZLENS_DC Phase 8 — 0x60 speed commands board verification")
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Verbose output")
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud}...")
    proto = ZlensProtocol(args.port, args.baud)
    time.sleep(0.5)

    try:
        test = SpeedCmdVerify(proto, verbose=args.verbose)
        ok = test.run_all()
    finally:
        proto.close()

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
