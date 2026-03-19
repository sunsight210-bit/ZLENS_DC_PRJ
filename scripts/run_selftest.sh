#!/bin/bash
# 自检板测脚本：烧录 → reset → 发 0x60 → SWO 捕获报告
set -e

PROJ="/home/cbn/VSCode/STM32_PRJ/ZLENS_DC"
SWO_LOG="$PROJ/Test/logs/swo_selftest.log"
PROG="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.2.400.202601091506/tools/bin/STM32_Programmer_CLI"
OPENOCD="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.externaltools.openocd.linux64_2.4.400.202601091506/tools/bin/openocd"
SCRIPTS="/home/cbn/st/stm32cubeide_2.1.0/plugins/com.st.stm32cube.ide.mcu.debug.openocd_2.3.300.202602021527/resources/openocd/st_scripts"

parse_swo() {
    python3 -c "
d=open('$SWO_LOG','rb').read();t=[];i=0
while i<len(d):
    if i+4<len(d) and d[i]==0x03:t.append(chr(d[i+1]));i+=5
    else:i+=1
print(''.join(t))
"
}

mkdir -p "$PROJ/Test/logs"

echo "=== [1] Flash ==="
$PROG -c port=SWD -w "$PROJ/build/fw/ZLENS_DC.elf" -v -rst 2>&1 | grep "verified"

echo "=== [2] OpenOCD reset + SWO ==="
> "$SWO_LOG"
$OPENOCD -s "$SCRIPTS" -f interface/stlink-dap.cfg -f target/stm32f1x.cfg \
    -c "init; tpiu config internal $SWO_LOG uart off 72000000 2000000; reset run; sleep 90000; shutdown" \
    2>/dev/null &
OCD_PID=$!

echo "=== [3] Wait boot (5s) ==="
sleep 5

echo "=== [4] Send SELF_TEST (0x60) ==="
python3 -c "
import serial,struct,time
def crc16(d):
    c=0xFFFF
    for b in d:
        c^=b
        for _ in range(8): c=(c>>1)^0xA001 if c&1 else c>>1
    return c
s=serial.Serial('/dev/ttyUSB0',115200,timeout=2)
s.reset_input_buffer()
h=struct.pack('>BBH',0xA5,0x60,0)
s.write(h+struct.pack('<H',crc16(h)))
time.sleep(1)
rx=s.read(100)
print(f'  UART RX: {len(rx)}B')
s.close()
"

echo "=== [5] Poll SWO for report ==="
for i in $(seq 1 17); do
    sleep 5
    if parse_swo | grep -q "SELF-TEST REPORT"; then
        echo "  Report found at $((i*5))s"
        sleep 2
        break
    fi
    echo "  ...$((i*5))s"
done

kill $OCD_PID 2>/dev/null
wait $OCD_PID 2>/dev/null || true

echo "=== SWO Output ==="
parse_swo | tee "$PROJ/Test/logs/selftest_report.txt"

echo ""
if grep -q "ALL PASSED" "$PROJ/Test/logs/selftest_report.txt"; then
    echo "=== RESULT: ALL PASSED ==="
elif grep -q "SOME FAILED" "$PROJ/Test/logs/selftest_report.txt"; then
    echo "=== RESULT: SOME FAILED ==="
else
    echo "=== RESULT: NO REPORT (timeout?) ==="
fi
