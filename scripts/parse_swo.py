#!/usr/bin/env python3
"""解析 OpenOCD SWO 日志文件，提取 ITM port 0 文本输出

用法：python3 scripts/parse_swo.py [swo_bin_path]
默认：build/swo/swo.bin
输出：解析后的文本同时写入 build/swo/swo.txt
"""
import os
import sys


def parse_swo(filepath: str) -> str:
    data = open(filepath, 'rb').read()
    text = []
    i = 0
    while i < len(data):
        hdr = data[i]
        if hdr == 0x01:  # 1-byte ITM port 0 packet
            if i + 1 < len(data):
                text.append(chr(data[i + 1]))
            i += 2
        elif hdr == 0x03:  # 4-byte ITM port 0 packet
            if i + 4 < len(data):
                text.append(chr(data[i + 1]))
            i += 5
        else:
            i += 1
    return ''.join(text)


if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else 'build/swo/swo.bin'
    result = parse_swo(path)
    if not result:
        print(f'(empty — no ITM data in {path})', file=sys.stderr)
        sys.exit(1)

    # 输出到终端
    print(result)

    # 同时保存到 build/swo/swo.txt
    txt_path = os.path.join(os.path.dirname(path), 'swo.txt')
    with open(txt_path, 'w') as f:
        f.write(result)
    print(f'[saved] {txt_path}', file=sys.stderr)
