#!/bin/bash
# 发送自检命令 (0x60) 并通过 SWO 抓取自检报告
# 流程: OpenOCD reset run + SWO → 等待启动 → 串口发自检命令 → 等待SWO报告

set -e

OPENOCD="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.linux64_2.4.400.202601091506/tools/bin/openocd"
SCRIPTS="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.debug.openocd_2.3.300.202602021527/resources/openocd/st_scripts"
PROJ_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SWO_LOG="$PROJ_DIR/Test/logs/swo_selftest.log"
mkdir -p "$PROJ_DIR/Test/logs"
UART_PORT="/dev/ttyUSB0"
WAIT_SELFTEST=90  # 自检含归零，最多等90秒

# CRC16/MODBUS 计算 (用 python)
build_frame() {
    python3 -c "
import struct
def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc
cmd = 0x60
param = 0
payload = struct.pack('<BH', cmd, param)
c = crc16(payload)
frame = b'\\xAA' + payload + struct.pack('<H', c) + b'\\x55'
import sys; sys.stdout.buffer.write(frame)
"
}

echo "=== ZLENS_DC Self-Test with SWO Report ==="

# 清空旧 SWO log
> "$SWO_LOG"

# 启动 OpenOCD 后台抓 SWO
echo "[1/4] Starting OpenOCD + SWO capture..."
$OPENOCD -s "$SCRIPTS" -f interface/stlink-dap.cfg -f target/stm32f1x.cfg \
    -c "init; tpiu config internal $SWO_LOG uart off 72000000 2000000; reset run; sleep ${WAIT_SELFTEST}000; shutdown" \
    2>/dev/null &
OCD_PID=$!

# 等待 MCU 启动
echo "[2/4] Waiting 3s for MCU boot..."
sleep 3

# 发送自检命令
echo "[3/4] Sending SELF_TEST command (0x60) via $UART_PORT..."
build_frame | python3 -c "
import serial, sys, time
ser = serial.Serial('$UART_PORT', 115200, timeout=2)
ser.reset_input_buffer()
frame = sys.stdin.buffer.read()
ser.write(frame)
time.sleep(1)
# 读取回显+响应
data = ser.read(100)
if data:
    print(f'  UART response: {data.hex()}')
else:
    print('  WARNING: No UART response')
ser.close()
"

# 等待自检完成 (含归零 ~60-80s)
echo "[4/4] Waiting for self-test to complete (up to ${WAIT_SELFTEST}s)..."
echo "  Monitoring SWO for SELF-TEST REPORT..."

# 轮询 SWO log 等待报告出现
ELAPSED=0
while [ $ELAPSED -lt $WAIT_SELFTEST ]; do
    if grep -q "SELF-TEST REPORT" "$SWO_LOG" 2>/dev/null; then
        echo "  Report detected after ${ELAPSED}s!"
        sleep 2  # 等报告完全输出
        break
    fi
    sleep 2
    ELAPSED=$((ELAPSED + 2))
done

# 终止 OpenOCD
kill $OCD_PID 2>/dev/null || true
wait $OCD_PID 2>/dev/null || true

echo ""
echo "=== SWO Output ==="
# 清理 SWO 输出中的间隔字符
sed 's/   //g' "$SWO_LOG"

echo ""
# 判断结果
if grep -q "ALL PASSED" "$SWO_LOG"; then
    echo "=== RESULT: SELF-TEST ALL PASSED ==="
    exit 0
elif grep -q "SOME FAILED" "$SWO_LOG"; then
    echo "=== RESULT: SELF-TEST SOME FAILED ==="
    exit 1
else
    echo "=== RESULT: NO REPORT CAPTURED (timeout?) ==="
    exit 2
fi
