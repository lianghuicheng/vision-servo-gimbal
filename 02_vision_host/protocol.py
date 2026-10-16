"""
串口通信协议 v1 —— Python 端实现（可直接被视觉脚本 import）

帧结构：
    ┌────────┬────────┬──────────────┬──────────┬────────┐
    │ 0xAA   │ 0x55   │ LEN (1 字节) │ PAYLOAD  │ CRC16  │ 尾部 0x0D 0x0A
    └────────┴────────┴──────────────┴──────────┴────────┘
    LEN     = PAYLOAD 的字节数
    CRC16   = CRC16-CCITT(多项式 0x1021, 初值 0xFFFF)，对 [LEN + PAYLOAD] 计算，小端发送

PAYLOAD 类型：
    0x01 TARGET    目标：cx, cy, w, h 为 uint16（像素），angle 为 int16（单位 0.1°，顺时针为正）
    0x02 LOST      目标丢失：字段全 0
    0x03 HEARTBEAT 心跳：无字段（LEN = 1）

用法：
    from protocol import pack_frame, pack_target, StreamDecoder

    data = pack_target(cx=120, cy=80, w=45, h=60, angle_deg=12.3)   # 打包
    ser.write(data)                                                  # 发送

    dec = StreamDecoder()
    for payload in dec.feed(ser.read(64)):                           # 接收解析
        print(payload)
"""

from __future__ import annotations

import struct
import sys

# Windows 控制台默认编码是 GBK，直接打印 UTF-8 字符会抛 UnicodeEncodeError。
# 统一改成 UTF-8 输出，避免"程序跑一半崩在 print 上"。
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

# ---------------------------------------------------------------- 常量
FRAME_HEAD = b"\xAA\x55"
FRAME_TAIL = b"\x0D\x0A"

TYPE_TARGET = 0x01
TYPE_LOST = 0x02
TYPE_HEARTBEAT = 0x03

# 各类型的 payload 长度（不含 type 字节的字段长度 + 1）
PAYLOAD_LEN = {
    TYPE_TARGET: 11,
    TYPE_LOST: 11,
    TYPE_HEARTBEAT: 1,
}

MAX_PAYLOAD = 64


# ---------------------------------------------------------------- CRC
def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE：poly=0x1021, init=0xFFFF, 输入输出不反转, 不异或输出。"""
    crc = init
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ---------------------------------------------------------------- 打包
def pack_frame(payload: bytes) -> bytes:
    """把 payload 封装成完整帧（含帧头、长度、CRC、帧尾）。"""
    if not 0 < len(payload) <= MAX_PAYLOAD:
        raise ValueError(f"payload 长度必须在 1~{MAX_PAYLOAD} 之间，当前 {len(payload)}")

    body = bytes([len(payload)]) + payload
    crc = crc16_ccitt(body)
    return FRAME_HEAD + body + struct.pack("<H", crc) + FRAME_TAIL


def pack_target(cx: int, cy: int, w: int, h: int, angle_deg: float = 0.0) -> bytes:
    """打包一个目标帧（0x01）。angle_deg 会被转成 0.1° 单位的 int16。"""
    angle_raw = int(round(angle_deg * 10.0))
    angle_raw = max(-32768, min(32767, angle_raw))
    payload = struct.pack(
        "<BHHHHh",
        TYPE_TARGET,
        _clamp_u16(cx), _clamp_u16(cy), _clamp_u16(w), _clamp_u16(h),
        angle_raw,
    )
    return pack_frame(payload)


def pack_lost() -> bytes:
    """打包一个"目标丢失"帧（0x02）。"""
    payload = struct.pack("<BHHHHh", TYPE_LOST, 0, 0, 0, 0, 0)
    return pack_frame(payload)


def pack_heartbeat() -> bytes:
    """打包一个心跳帧（0x03）。"""
    return pack_frame(bytes([TYPE_HEARTBEAT]))


def _clamp_u16(v: float) -> int:
    return max(0, min(65535, int(round(v))))


# ---------------------------------------------------------------- 解析
class StreamDecoder:
    """
    流式解析器（解决串口粘包/断包问题）。

    用法：
        dec = StreamDecoder()
        payloads = dec.feed(ser.read(128))    # 返回本次新解析出的 payload 列表
    """

    def __init__(self, strict: bool = True):
        self._buf = bytearray()
        self.strict = strict
        self.stats = {"ok": 0, "crc_error": 0, "len_error": 0, "tail_error": 0}

    def feed(self, chunk: bytes) -> list[bytes]:
        self._buf.extend(chunk)
        out: list[bytes] = []

        while True:
            # 1) 找帧头
            idx = self._buf.find(FRAME_HEAD)
            if idx < 0:
                # 没有帧头，保留最后一个字节（可能是半个帧头）
                if len(self._buf) > 1:
                    del self._buf[:-1]
                break
            if idx > 0:
                del self._buf[:idx]          # 丢弃帧头之前的垃圾数据
            if len(self._buf) < 3:
                break                        # 长度字节还没到

            # 2) 长度检查
            plen = self._buf[2]
            if plen == 0 or plen > MAX_PAYLOAD:
                self.stats["len_error"] += 1
                del self._buf[:2]            # 丢掉这个假帧头，继续找
                continue

            total = 2 + 1 + plen + 2 + 2     # 头 + 长度 + 载荷 + CRC + 尾
            if len(self._buf) < total:
                break                        # 数据还没收全，等下一次

            frame = bytes(self._buf[:total])

            # 3) 帧尾检查
            if frame[-2:] != FRAME_TAIL:
                self.stats["tail_error"] += 1
                del self._buf[:2]
                continue

            # 4) CRC 检查
            body = frame[2:3 + plen]
            crc_recv = struct.unpack("<H", frame[3 + plen:5 + plen])[0]
            crc_calc = crc16_ccitt(body)
            if crc_recv != crc_calc:
                self.stats["crc_error"] += 1
                del self._buf[:2]
                continue

            # 5) 通过校验
            payload = frame[3:3 + plen]
            self.stats["ok"] += 1
            out.append(payload)
            del self._buf[:total]

        return out

    @staticmethod
    def decode_target(payload: bytes) -> dict | None:
        """把 TARGET/LOST 类型的 payload 解成字典；不是目标帧则返回 None。"""
        if len(payload) != PAYLOAD_LEN[TYPE_TARGET]:
            return None
        type_, cx, cy, w, h, angle_raw = struct.unpack("<BHHHHh", payload)
        if type_ not in (TYPE_TARGET, TYPE_LOST):
            return None
        return {
            "type": "target" if type_ == TYPE_TARGET else "lost",
            "cx": cx, "cy": cy, "w": w, "h": h,
            "angle": angle_raw / 10.0,
        }


# ---------------------------------------------------------------- 自测
if __name__ == "__main__":
    print("=== CRC16 自测 ===")
    # 本实现是 CRC-16/CCITT-FALSE：poly=0x1021, init=0xFFFF, 不反转, 不异或输出
    # 对 ASCII "123456789" 的标准校验值是 0x29B1
    # （注意：0x31C3 是 CRC-16/XMODEM，它的 init=0x0000，不要搞混）
    got = crc16_ccitt(b"123456789")
    print(f'crc16_ccitt("123456789") = 0x{got:04X}  (期望 0x29B1)  {"[OK]" if got == 0x29B1 else "[FAIL]"}')

    print("\n=== 打包/解析往返测试 ===")
    frame = pack_target(120, 80, 45, 60, 12.3)
    print("帧字节:", " ".join(f"{b:02X}" for b in frame))

    dec = StreamDecoder()
    # 模拟粘包：两帧连在一起 + 前面有垃圾字节
    stream = b"\x00\xFF" + frame + pack_lost() + frame[:5]
    got_payloads = dec.feed(stream)
    for p in got_payloads:
        print("解析出:", StreamDecoder.decode_target(p))

    # 再来一次，补上剩余的半个帧
    got_payloads = dec.feed(frame[5:])
    for p in got_payloads:
        print("解析出(续):", StreamDecoder.decode_target(p))

    print("\n统计:", dec.stats)
    print("\n=== 纠错测试：故意翻转一个字节，应被 CRC 拦截 ===")
    bad = bytearray(frame)
    bad[4] ^= 0xFF
    dec2 = StreamDecoder()
    got_payloads = dec2.feed(bytes(bad) + frame)
    print("被 CRC 拦截数:", dec2.stats["crc_error"], "| 正常解析数:", len(got_payloads))
