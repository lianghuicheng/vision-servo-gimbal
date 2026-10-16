"""
串口回环自测（W1）：把 TXD 和 RXD 短接，验证「Python + 驱动 + 线 + 波特率」这条链路。

特点：
  - 用真实的协议帧（不是随便发字符串），所以它同时验证了 protocol.py 的正确性
  - 逐帧比对序号，能算出真实的丢包率
  - 每帧都过 CRC，能算出误码率
  - 不依赖任何 STM32 固件

用法：
    python loopback_test.py --list
    python loopback_test.py --port COM5
    python loopback_test.py --port COM5 -n 1000 --baud 460800
    python loopback_test.py --port COM5 --text        # 改用纯文本帧（排查更直观）
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

import serial
import serial.tools.list_ports

from protocol import StreamDecoder, pack_target, crc16_ccitt

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def build_text_frame(seq: int) -> bytes:
    """纯文本帧，方便用串口助手肉眼确认。格式：<SEQ:000123>"""
    return f"<SEQ:{seq:06d}>".encode("ascii")


def parse_text_frames(buf: bytearray) -> list[int]:
    """从字节流里抠出所有 <SEQ:000123> 的序号。"""
    out: list[int] = []
    while True:
        i = buf.find(b"<SEQ:")
        if i < 0:
            if len(buf) > 5:
                del buf[:-5]
            break
        j = buf.find(b">", i)
        if j < 0:
            del buf[:i]
            break
        try:
            out.append(int(buf[i + 5:j]))
        except ValueError:
            pass
        del buf[:j + 1]
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="串口号，如 COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("-n", type=int, default=200, help="发送帧数")
    ap.add_argument("--text", action="store_true", help="用纯文本帧代替二进制协议帧")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()

    ports = list(serial.tools.list_ports.comports())
    if args.list or not ports:
        if not ports:
            print("[错误] 没有找到任何串口设备。")
            print("   检查：USB 线插好了吗？CH340 驱动装了吗？")
            print("   驱动位置：D:\\STM32_小车项目\\02_工具\\CH340驱动\\")
            return 1
        print(f"发现 {len(ports)} 个串口：")
        for p in ports:
            print(f"  {p.device:<8} {p.description}")
        return 0

    port = args.port or ports[0].device
    print("=" * 62)
    print(f"串口回环测试  端口={port}  波特率={args.baud}  帧数={args.n}  "
          f"帧类型={'文本' if args.text else '二进制协议'}")
    print("=" * 62)
    print("提示：请确认 TXD 与 RXD 已经短接（或 PA9 与 PA10 已短接）。\n")

    try:
        with serial.Serial(port, args.baud, timeout=0.05) as ser:
            time.sleep(0.3)
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            t_start = time.perf_counter()
            sent_bytes = 0

            for seq in range(1, args.n + 1):
                if args.text:
                    frame = build_text_frame(seq)
                else:
                    # 用序号编造坐标，这样每一帧的载荷都不同，能检出"粘帧"问题
                    frame = pack_target(cx=seq % 640, cy=seq % 480, w=seq, h=seq, angle_deg=seq % 360)
                ser.write(frame)
                sent_bytes += len(frame)

            ser.flush()
            # 等数据回环回来
            deadline = time.perf_counter() + max(1.0, args.n * 0.01)
            raw = bytearray()
            while time.perf_counter() < deadline:
                chunk = ser.read(4096)
                if chunk:
                    raw.extend(chunk)
                elif len(raw) > 0 and time.perf_counter() > deadline - 0.5:
                    break

            t_end = time.perf_counter()
            elapsed = t_end - t_start

        # ---------------- 统计 ----------------
        print(f"发送 {args.n} 帧 / {sent_bytes} 字节，用时 {elapsed:.2f}s "
              f"（约 {sent_bytes / max(elapsed, 1e-6) / 1024:.1f} KB/s）")
        print(f"收到 {len(raw)} 字节\n")

        if args.text:
            got = parse_text_frames(raw)
            uniq = sorted(set(got))
            received = len(uniq)
        else:
            dec = StreamDecoder()
            payloads = dec.feed(bytes(raw))
            got = []
            bad_payload = 0
            for p in payloads:
                d = StreamDecoder.decode_target(p)
                if d is None:
                    bad_payload += 1
                    continue
                got.append(round(d["cx"]))     # cx 就是序号
            received = len(got)
            print(f"解析统计：合法帧 {dec.stats['ok']}  CRC 错 {dec.stats['crc_error']}  "
                  f"长度错 {dec.stats['len_error']}  帧尾错 {dec.stats['tail_error']}  "
                  f"载荷异常 {bad_payload}")

        # 序号连续性：找出丢掉的帧
        if got:
            expected = set(range(1, args.n + 1))
            actual = set(got)
            missing = sorted(expected - actual)
            dup = len(got) - len(actual)
        else:
            missing = list(range(1, args.n + 1))
            dup = 0

        rate = 100.0 * received / args.n
        print(f"\n回环成功率：{received}/{args.n} = {rate:.2f}%")
        if dup:
            print(f"重复帧：{dup}    （通常是回环线上的回声或有其他程序在发）")
        if missing:
            show = missing[:10]
            print(f"丢失帧：{len(missing)}   前几个：{show}{' ...' if len(missing) > 10 else ''}")

        print("\n" + "-" * 62)
        if rate >= 99.9:
            print("[通过] 链路完全正常，可以进入下一步：连 STM32 并烧固件。")
        elif rate >= 95.0:
            print("[勉强] 有丢帧。建议：降到 115200、换短一点的杜邦线、检查共地。")
        elif rate > 0:
            print("[不达标] 丢帧过多。检查：波特率是否一致？线是否太长？是否共地？")
        else:
            print("[失败] 一帧都没回来。最可能的原因：")
            print("   1) TXD 和 RXD 没有短接（或 PA9/PA10 没短接）")
            print("   2) 串口被 XCOM 等程序占用了 —— 同一个串口只能被一个程序打开")
            print("   3) 端口选错了，用 --list 重新确认")
        print("-" * 62)

        # CRC 实现自检，确保协议端是对的
        assert crc16_ccitt(b"123456789") == 0x29B1
        return 0 if rate >= 99.9 else 2

    except serial.SerialException as e:
        print(f"[错误] 串口错误：{e}")
        print("   最常见原因：串口被 XCOM / 串口助手 占用了，先关掉它。")
        return 1
    except KeyboardInterrupt:
        print("\n已中断。")
        return 130


if __name__ == "__main__":
    sys.exit(main())
