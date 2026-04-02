#!/usr/bin/env python3
"""Decode raw ITM binary frames from SWO capture to plain text.

Usage: python3 itm_decode.py <raw_itm_file> [output_file]
       If output_file is omitted, decoded text is written to stdout.
"""

import sys


def decode_itm(data: bytes) -> str:
    """Extract port 0 character data from raw ITM binary frames."""
    result = []
    i = 0
    length = len(data)

    while i < length:
        header = data[i]

        if header == 0x03 and i + 4 < length:
            # 4-byte ITM software stimulus packet (port 0)
            ch = data[i + 1]
            if ch == 0x0A or (0x20 <= ch <= 0x7E):
                result.append(chr(ch))
            i += 5
        elif header == 0x01 and i + 1 < length:
            # 1-byte ITM software stimulus packet (port 0)
            ch = data[i + 1]
            if ch == 0x0A or (0x20 <= ch <= 0x7E):
                result.append(chr(ch))
            i += 2
        else:
            # Skip unrecognized byte
            i += 1

    return "".join(result)


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <raw_itm_file> [output_file]",
              file=sys.stderr)
        sys.exit(1)

    raw_path = sys.argv[1]

    with open(raw_path, "rb") as f:
        data = f.read()

    text = decode_itm(data)

    if len(sys.argv) >= 3:
        with open(sys.argv[2], "w") as f:
            f.write(text)
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
