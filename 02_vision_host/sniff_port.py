"""
串口嗅探器（W1 调试工具）：监听某个串口收到的数据，并把 GBK 中文解码出来。

用途：
  - 弄清板子上跑的固件到底在发什么
  - 调试时看 STM32 回传的内容是否正常
  - 判断板子是"主动定时上报"还是"收到才回应"

用法：
    python sniff_port.py COM4                  # 只监听 5 秒
    python sniff_port.py COM4 --seconds 10
    python sniff_port.py COM4 --send "R,120,80,45,60\r\n"   # 先发一条再听
    python sniff_port.py COM4 --send "PING" --seconds 3
"""

from __future__ import annotations

import argparse
import sys
import time

import serial

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def describe(raw: bytes) -> None:
    """把收到的字节用三种方式展示：原始十六进制、ASCII、GBK 解码。"""
    if not raw:
        print("   (没有收到任何数据)")
        return
    print(f"   字节数 : {len(raw)}")
    print(f"   原始   : {raw[:160]!r}{' ...' if len(raw) > 160 else ''}")
    print(f"   十六进制: {raw[:64].hex(' ')}{' ...' if len(raw) > 64 else ''}")
    try:
        txt = raw.decode("gbk", errors="replace")
    except Exception:
        txt = repr(raw)
    print("   GBK 解码:")
    for line in txt.splitlines():
        if line.strip():
            print(f"      | {line}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="串口号，如 COM4")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=5.0, help="监听时长（秒）")
    ap.add_argument("--send", default=None, help="监听前先发送的字符串（支持 \\r\\n 转义）")
    args = ap.parse_args()

    payload = None
    if args.send is not None:
        payload = args.send.encode("utf-8").decode("unicode_escape").encode("latin-1")

    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as s:
            print(f"已打开 {args.port} @ {args.baud}")
            s.reset_input_buffer()

            if payload:
                n = s.write(payload)
                s.flush()
                print(f"\n[发送] {n} 字节: {payload!r}")

            print(f"\n[监听] 持续 {args.seconds} 秒 ...")
            t0 = time.time()
            data = bytearray()
            while time.time() - t0 < args.seconds:
                chunk = s.read(256)
                if chunk:
                    data.extend(chunk)

            print("\n[收到]")
            describe(bytes(data))

            print("\n[结论提示]")
            if not data:
                print("   板子全程没有发数据 —— 它可能只在收到特定格式时才回应，")
                print("   或者它根本不回传（很多小车程序只收不发）。")
            elif args.send:
                print("   发了就有回应 → 板子会回传数据。")
            else:
                print("   什么都没发也收到数据 → 板子在主动/定时上报。")
        return 0
    except serial.SerialException as e:
        print(f"[错误] {args.port} 打不开：{e}")
        print("   最常见原因：另一个程序（XCOM 等）占用了该串口。")
        return 1


if __name__ == "__main__":
    sys.exit(main())
