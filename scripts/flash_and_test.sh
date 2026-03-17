#!/bin/bash
# scripts/flash_and_test.sh — Auto compile, flash, SWO capture, and result parsing
set -euo pipefail

OPENOCD="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.linux64_2.4.400.202601091506/tools/bin/openocd"
SCRIPTS="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.debug.openocd_2.3.300.202602021527/resources/openocd/st_scripts"
ELF="build/fw/ZLENS_DC.elf"
RAW_LOG="logs/swo_raw.log"
SWO_LOG="logs/swo_output.log"
TIMEOUT=${1:-10}  # seconds to wait for SWO output, default 10

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}=== ZLENS_DC Auto Flash & Test ===${NC}"

# Step 1: Build ARM target
echo -e "${YELLOW}[1/5] Building ARM target...${NC}"
if [ ! -d "build/fw" ]; then
    cmake -B build/fw -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake
fi
ninja -C build/fw
echo -e "${GREEN}[1/5] Build OK${NC}"

# Step 2: Flash + SWO capture
echo -e "${YELLOW}[2/5] Flashing and capturing SWO (${TIMEOUT}s)...${NC}"
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

echo -e "${GREEN}[2/5] Flash + capture done${NC}"

# Step 3: Decode ITM frames to plain text
echo -e "${YELLOW}[3/5] Decoding ITM frames...${NC}"
if [ ! -f "$RAW_LOG" ]; then
    echo -e "${RED}ERROR: Raw SWO output file not found${NC}"
    exit 1
fi

python3 scripts/itm_decode.py "$RAW_LOG" "$SWO_LOG"
echo -e "${GREEN}[3/5] Decode done${NC}"

# Step 4: Parse SWO output
echo -e "${YELLOW}[4/5] Parsing SWO output...${NC}"
if [ ! -f "$SWO_LOG" ]; then
    echo -e "${RED}ERROR: Decoded SWO output file not found${NC}"
    exit 1
fi

# Display decoded output
echo "--- SWO Output ---"
cat "$SWO_LOG" 2>/dev/null || true
echo "--- End SWO Output ---"

# Step 5: Count PASS/FAIL
PASS_COUNT=$(grep -c '\[PASS\]' "$SWO_LOG" || true)
FAIL_COUNT=$(grep -c '\[FAIL\]' "$SWO_LOG" || true)

echo ""
echo -e "${YELLOW}[5/5] Results:${NC}"
echo -e "  ${GREEN}PASS: ${PASS_COUNT}${NC}"
echo -e "  ${RED}FAIL: ${FAIL_COUNT}${NC}"

if [ "$FAIL_COUNT" -gt 0 ]; then
    echo ""
    echo -e "${RED}=== FAILED TESTS ===${NC}"
    grep '\[FAIL\]' "$SWO_LOG" 2>/dev/null || true
    exit 1
fi

if [ "$PASS_COUNT" -eq 0 ]; then
    echo -e "${RED}WARNING: No test results found in SWO output${NC}"
    exit 1
fi

echo -e "${GREEN}=== ALL TESTS PASSED ===${NC}"
exit 0
