"""
W1 最小验证脚本：让 Python 通过串口说第一句话。

用途：在动手写视觉之前，先确认"串口通不通"。这是整条链路的地基。

步骤：
  1. 用 USB 线把 STM32（或 USB-TTL 模块）插到电脑
  2. 打开 XCOM（D:\\STM32_小车项目\\02_工具\\串口助手\\），选好串口，波特率 115200，打开
  3. 运行本脚本，XCOM 里应该出现文字
  4. 再在 XCOM 里手动发一句，本脚本应该打印出来

运行：
    python serial_hello.py                 # 自动列出串口，选第一个
    python serial_hello.py --port COM5     # 指定串口
    python serial_hello.py --list          # 只列出所有串口
"""

from __future__ import annotations

import argparse
import sys
import time

# Windows 控制台默认编码是 GBK，直接打印 UTF-8 字符会抛 UnicodeEncodeError
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import serial
import serial.tools.list_ports

BAUD = 115200


def list_ports() -> list[str]:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[错误] 没有找到任何串口设备。")
        print("   检查：USB 线是否插好？CH340 驱动是否已装？")
        print("   驱动位置：D:\\STM32_小车项目\\02_工具\\CH340驱动\\")
        return []
    print(f"发现 {len(ports)} 个串口：")
    for p in ports:
        print(f"  {p.device:<8} {p.description}")
    return [p.device for p in ports]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="串口号，如 COM5")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--list", action="store_true", help="只列出串口")
    args = ap.parse_args()

    ports = list_ports()
    if args.list:
        return 0
    if not ports:
        return 1

    port = args.port or ports[0]
    print(f"\n打开 {port} @ {args.baud} bps ...")

    try:
        with serial.Serial(port, args.baud, timeout=0.1) as ser:
            time.sleep(0.2)
            print("[OK] 串口已打开。按 Ctrl+C 退出。\n")

            # 1) 主动发
            msg = "HELLO FROM PYTHON\n"
            ser.write(msg.encode("ascii"))
            print(f"[发送] {msg.strip()}")

            # 2) 持续收发
            n = 0
            while True:
                n += 1
                ser.write(f"PING {n}\n".encode("ascii"))
                time.sleep(0.5)

                data = ser.read(256)
                if data:
                    print(f"[接收] {data!r}")

    except KeyboardInterrupt:
        print("\n已退出。")
        return 0
    except serial.SerialException as e:
        print(f"[错误] 串口错误：{e}")
        print("   常见原因：串口被 XCOM 占用了（同一个串口只能被一个程序打开）。")
        return 1


if __name__ == "__main__":
    sys.exit(main())
