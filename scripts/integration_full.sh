#!/bin/bash
# scripts/integration_full.sh — 端到端集成测试：编译→烧录→等待启动→Python集成测试
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

# Tools
OPENOCD="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.linux64_2.4.400.202601091506/tools/bin/openocd"
SCRIPTS="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.debug.openocd_2.3.300.202602021527/resources/openocd/st_scripts"
ELF="build/fw/ZLENS_DC.elf"
PORT="${1:-/dev/ttyACM0}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Parse extra args for integration_test.py
shift || true
EXTRA_ARGS="$*"

echo -e "${YELLOW}=== ZLENS_DC 端到端集成测试 ===${NC}"

# Step 1: Build firmware
echo -e "${YELLOW}[1/4] Building firmware...${NC}"
if [ ! -d "build/fw" ]; then
    cmake -B build/fw -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/stm32f103rc.cmake
fi
ninja -C build/fw
echo -e "${GREEN}[1/4] Build OK${NC}"

# Step 2: Flash
echo -e "${YELLOW}[2/4] Flashing...${NC}"
$OPENOCD -s "$SCRIPTS" \
    -f interface/stlink-dap.cfg \
    -f target/stm32f1x.cfg \
    -c "program $ELF verify reset exit" 2>/dev/null
echo -e "${GREEN}[2/4] Flash OK${NC}"

# Step 3: Wait for boot + self-test
echo -e "${YELLOW}[3/4] Waiting for boot (8s)...${NC}"
sleep 8
echo -e "${GREEN}[3/4] Boot wait done${NC}"

# Step 4: Run integration tests
echo -e "${YELLOW}[4/4] Running integration tests...${NC}"
python3 scripts/integration_test.py --port "$PORT" $EXTRA_ARGS
RESULT=$?

if [ $RESULT -eq 0 ]; then
    echo -e "${GREEN}=== 集成测试全部通过 ===${NC}"
else
    echo -e "${RED}=== 集成测试有失败 ===${NC}"
fi
exit $RESULT
