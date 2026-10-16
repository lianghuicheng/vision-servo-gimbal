"""
串口开启自检（W1）：确认指定串口能否正常打开，并打印它的参数。

用途：在跑回环测试之前，先单独确认"端口本身是好的"，
避免把"端口打不开"和"回环失败"两个问题混在一起排查。

用法：
    python port_check.py --list
    python port_check.py COM4
"""

from __future__ import annotations

import sys

import serial
import serial.tools.list_ports

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

BAUD = 115200


def main() -> int:
    args = sys.argv[1:]

    print("检测到的串口：")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("   (无) —— 检查 USB 线是否插好、是否插在标着 USB_232 的口上")
        return 1
    for p in ports:
        print(f"   {p.device:<8} | {p.description}")
        print(f"            hwid: {p.hwid}")

    if not args or args[0] == "--list":
        return 0

    port = args[0]
    print(f"\n尝试打开 {port} @ {BAUD} ...")
    try:
        with serial.Serial(port, BAUD, timeout=0.3) as s:
            print(f"[OK] {port} 打开成功")
            print(f"     波特率 {s.baudrate} | 数据位 {s.bytesize} | "
                  f"停止位 {s.stopbits} | 校验 {s.parity}")

            payload = b"PING\r\n"
            n = s.write(payload)
            s.flush()
            print(f"[OK] 写入 {n} 字节成功")

            back = s.read(64)
            if back:
                print(f"[收到] {back!r}")
            else:
                print("[收到] (空)")
                print("      这是正常的 —— 还没做 TX/RX 短接，所以没有回音。")
                print("      下一步做回环：用杜邦线短接 TXD 和 RXD（或板上 PA9 与 PA10），")
                print("      然后运行： python loopback_test.py --port " + port)
        print(f"[OK] {port} 已正常关闭")
        return 0

    except serial.SerialException as e:
        print(f"[错误] {port} 打不开：{e}")
        print("       最常见原因：XCOM / 串口助手还开着占用了端口。")
        print("       同一个串口同一时刻只能被一个程序打开。")
        return 1


if __name__ == "__main__":
    sys.exit(main())
