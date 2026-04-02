#!/bin/bash
# scripts/swo_capture_test.sh — SWO capture + full command test
# Usage: bash scripts/swo_capture_test.sh [serial_port] [timeout_sec]
#   serial_port: default /dev/ttyUSB0
#   timeout_sec: default 180
set -euo pipefail

SERIAL_PORT="${1:-/dev/ttyUSB0}"
TIMEOUT="${2:-180}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

OPENOCD="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.linux64_2.4.400.202601091506/tools/bin/openocd"
SCRIPTS="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.debug.openocd_2.3.300.202602021527/resources/openocd/st_scripts"
RAW_LOG="Test/logs/swo_raw.log"

mkdir -p Test/logs
rm -f "$RAW_LOG"

echo "Starting SWO capture (timeout: ${TIMEOUT}s)..."
$OPENOCD -s "$SCRIPTS" \
    -f interface/stlink-dap.cfg \
    -f target/stm32f1x.cfg \
    -c "init" \
    -c "tpiu config internal $RAW_LOG uart off 64000000 2000000" \
    -c "itm port 0 on" \
    -c "resume" \
    -c "sleep $((TIMEOUT * 1000))" \
    -c "shutdown" 2>&1 &
SWO_PID=$!
echo "SWO PID: $SWO_PID"

sleep 3
echo "SWO capturing in background, running test..."

python3 scripts/full_cmd_test.py "$SERIAL_PORT" 2>&1
TEST_EXIT=$?

echo "Test finished (exit=$TEST_EXIT), waiting for SWO capture to complete..."
wait $SWO_PID 2>/dev/null || true

echo "SWO capture done."
exit $TEST_EXIT
