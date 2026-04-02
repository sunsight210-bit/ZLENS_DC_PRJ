#!/bin/bash
# scripts/swo_live.sh — 烧录后持续捕获 SWO 输出（实时解码显示）
# 用法: bash scripts/swo_live.sh [超时秒数，默认60]
# 在另一个终端用 uart_debug.py 发串口数据，观察此窗口的 SWO 输出
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

OPENOCD="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.linux64_2.4.400.202601091506/tools/bin/openocd"
SCRIPTS="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.debug.openocd_2.3.300.202602021527/resources/openocd/st_scripts"
ELF="build/fw/ZLENS_DC.elf"
RAW_LOG="Test/logs/swo_raw.log"
SWO_LOG="Test/logs/swo_output.log"
TIMEOUT="${1:-60}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}=== SWO Live Monitor (${TIMEOUT}s) ===${NC}"

# Build
echo -e "${YELLOW}[1/3] Building...${NC}"
ninja -C build/fw
echo -e "${GREEN}[1/3] Build OK${NC}"

# Flash + SWO capture
echo -e "${YELLOW}[2/3] Flashing and capturing SWO for ${TIMEOUT}s...${NC}"
echo -e "${YELLOW}      >>> 在另一个终端运行: python3 scripts/uart_debug.py /dev/ttyUSB0 <<<${NC}"
mkdir -p "$(dirname "$RAW_LOG")"
rm -f "$RAW_LOG" "$SWO_LOG"

$OPENOCD -s "$SCRIPTS" \
    -f interface/stlink-dap.cfg \
    -f target/stm32f1x.cfg \
    -c "program $ELF verify" \
    -c "reset halt" \
    -c "tpiu config internal $RAW_LOG uart off 64000000 2000000" \
    -c "itm port 0 on" \
    -c "resume" \
    -c "sleep $((TIMEOUT * 1000))" \
    -c "shutdown" 2>/dev/null

echo -e "${GREEN}[2/3] Capture done${NC}"

# Decode
echo -e "${YELLOW}[3/3] Decoding SWO...${NC}"
python3 scripts/itm_decode.py "$RAW_LOG" "$SWO_LOG"

echo ""
echo -e "${YELLOW}=== SWO Output ===${NC}"
cat "$SWO_LOG"
echo -e "${YELLOW}=== End ===${NC}"
