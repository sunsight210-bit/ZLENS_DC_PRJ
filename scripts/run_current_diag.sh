#!/bin/bash
# 电流测量诊断：烧录 → reset → SWO 捕获电流数据
set -e

PROJ="/home/cbn/VSCode/STM32_PRJ/ZLENS_DC"
SWO_LOG="$PROJ/Test/logs/swo_current_diag.log"
PROG="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.2.400.202601091506/tools/bin/STM32_Programmer_CLI"
OPENOCD="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.linux64_2.4.400.202601091506/tools/bin/openocd"
SCRIPTS="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.debug.openocd_2.3.300.202602021527/resources/openocd/st_scripts"
ELF="$PROJ/build/diag/Test/diag/ZLENS_DIAG.elf"
WAIT_TIME=180  # 3 minutes max (homing + 4 speeds x 2 dirs + stall)

mkdir -p "$PROJ/Test/logs"

parse_swo() {
    python3 -c "
d=open('$SWO_LOG','rb').read();t=[];i=0
while i<len(d):
    if i+4<len(d) and d[i]==0x03:t.append(chr(d[i+1]));i+=5
    else:i+=1
print(''.join(t))
"
}

echo "=== Current Profile Diagnostic ==="
echo "  WARNING: Motor will move at multiple speeds and hit limits!"
echo ""

echo "=== [1] Flash diag firmware ==="
$PROG -c port=SWD -w "$ELF" -v -rst 2>&1 | grep "verified"

echo "=== [2] OpenOCD reset + SWO ==="
> "$SWO_LOG"
$OPENOCD -s "$SCRIPTS" -f interface/stlink-dap.cfg -f target/stm32f1x.cfg \
    -c "init; tpiu config internal $SWO_LOG uart off 72000000 2000000; reset run; sleep ${WAIT_TIME}000; shutdown" \
    2>/dev/null &
OCD_PID=$!

echo "=== [3] Waiting for diagnostic to complete ==="
echo "  (up to ${WAIT_TIME}s, polling every 5s...)"
for i in $(seq 1 $((WAIT_TIME / 5))); do
    sleep 5
    if parse_swo | grep -q "DONE"; then
        echo "  Diagnostic completed at $((i*5))s"
        sleep 2
        break
    fi
    echo "  ...$((i*5))s"
done

kill $OCD_PID 2>/dev/null
wait $OCD_PID 2>/dev/null || true

echo ""
echo "=== SWO Output ==="
REPORT="$PROJ/Test/logs/current_diag_report.txt"
parse_swo | tee "$REPORT"

echo ""
if grep -q "DONE" "$REPORT"; then
    echo "=== DIAGNOSTIC COMPLETE ==="
else
    echo "=== WARNING: Diagnostic may not have finished ==="
fi
