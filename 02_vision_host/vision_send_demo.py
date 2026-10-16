"""
视觉端主程序骨架（W2–W4）：摄像头识别目标 → 打包成协议帧 → 串口发出。

先用合成数据把整条链路跑通，再接摄像头 —— 这样你能把"视觉的问题"和"通信/控制的问题"分开排查。

三种运行模式：
    python vision_send_demo.py --mode sweep     # 不用摄像头，发送正弦扫动的假目标（先调 STM32 端）
    python vision_send_demo.py --mode camera    # 用真实摄像头识别红色物体
    python vision_send_demo.py --mode file      # 用一张图片测试（无需摄像头）

调参小抄：
    摄像头参数（曝光/增益）就是抗光的关键。W7 会专门做这个实验。
    如果画面偏暗/闪烁：先调 CAP_PROP_AUTO_EXPOSURE，再手动锁定 EXPOSURE 和 GAIN。
"""

from __future__ import annotations

import argparse
import math
import sys
import time

# Windows 控制台默认编码是 GBK，直接打印 UTF-8 字符会抛 UnicodeEncodeError
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

import cv2
import numpy as np
import serial
import serial.tools.list_ports

from protocol import StreamDecoder, pack_heartbeat, pack_lost, pack_target

# ---------------------------------------------------------------- 可调参数
BAUD = 115200
FRAME_W, FRAME_H = 640, 480      # 分辨率越低延迟越小（W4 优化点之一）

# 红色在 HSV 里跨了两段（0° 附近），所以要两个区间取并集
RED_RANGES = [
    ((0, 90, 60), (10, 255, 255)),
    ((170, 90, 60), (180, 255, 255)),
]
MIN_AREA = 500                    # 面积小于这个值当作噪声丢掉

# 摄像头参数：手动锁定曝光/增益是抗光的第一步（不同摄像头数值范围不同，需要试）
CAP_AUTO_EXPOSURE_OFF = 0.25      # 值因驱动而异，常见 0.25=手动 / 0.75=自动
CAP_EXPOSURE = -6                 # 曝光值（负值通常是更短曝光）
CAP_GAIN = 0                      # 增益越小噪声越低


def open_serial(port: str | None, baud: int) -> serial.Serial:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise SystemExit("[错误] 没找到串口。请插好 USB 线，或用 --dry-run 只在屏幕上看结果。")
    dev = port or ports[0].device
    print(f"串口：{dev} @ {baud}")
    return serial.Serial(dev, baud, timeout=0)


def open_camera(index: int = 0) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)   # Windows 下用 DSHOW 打开更快、参数更可控
    if not cap.isOpened():
        raise SystemExit("[错误] 摄像头打不开。换个 index（--camera 1）或检查是否被其他程序占用。")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, 60)
    # 锁定曝光与增益 —— 这一步非常重要，自动曝光会让阈值忽明忽暗
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, CAP_AUTO_EXPOSURE_OFF)
    cap.set(cv2.CAP_PROP_EXPOSURE, CAP_EXPOSURE)
    cap.set(cv2.CAP_PROP_GAIN, CAP_GAIN)
    return cap


def find_target(frame: np.ndarray) -> dict | None:
    """
    在画面里找最大的红色色块，返回 {cx, cy, w, h, angle, area}；找不到返回 None。

    这是 W2 的核心函数，后面会逐步加：形态学去噪、LAB 空间、动态阈值。
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = None
    for lo, hi in RED_RANGES:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)

    # 形态学：先开后闭 —— 开运算去孤立噪点，闭运算补内部空洞
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_AREA:
        return None

    # minAreaRect 能给出带角度的最小外接矩形（比 boundingRect 信息更多）
    (cx, cy), (w, h), angle = cv2.minAreaRect(c)
    return {"cx": cx, "cy": cy, "w": w, "h": h, "angle": angle, "area": cv2.contourArea(c),
            "contour": c, "mask": mask}


def draw_overlay(frame: np.ndarray, t: dict | None, fps: float) -> np.ndarray:
    """把识别结果画到画面上，方便肉眼确认（W20 会升级成独立上位机）。"""
    h, w = frame.shape[:2]
    cv2.line(frame, (w // 2, 0), (w // 2, h), (128, 128, 128), 1)
    cv2.line(frame, (0, h // 2), (w, h // 2), (128, 128, 128), 1)

    if t:
        box = cv2.boxPoints(((t["cx"], t["cy"]), (t["w"], t["h"]), t["angle"]))
        cv2.drawContours(frame, [np.intp(box)], 0, (0, 255, 0), 2)
        cv2.circle(frame, (int(t["cx"]), int(t["cy"])), 4, (0, 0, 255), -1)
        cv2.putText(frame,
                    f"({t['cx']:.0f},{t['cy']:.0f}) {t['w']:.0f}x{t['h']:.0f} {t['angle']:.1f}deg",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        # 误差：目标中心相对画面中心的偏移（这就是 PID 的输入！）
        ex, ey = t["cx"] - w / 2, t["cy"] - h / 2
        cv2.putText(frame, f"err=({ex:+.0f},{ey:+.0f})", (10, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    else:
        cv2.putText(frame, "NO TARGET", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    cv2.putText(frame, f"FPS {fps:.1f}", (w - 130, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    return frame


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["sweep", "camera", "file"], default="camera")
    ap.add_argument("--port", help="串口号，如 COM5")
    ap.add_argument("--camera", type=int, default=0, help="摄像头序号")
    ap.add_argument("--file", default="test.jpg", help="--mode file 时读的图片")
    ap.add_argument("--dry-run", action="store_true", help="不发串口，只在屏幕上看")
    ap.add_argument("--fps", type=float, default=30.0, help="sweep 模式发包频率")
    args = ap.parse_args()

    ser = None if args.dry_run else open_serial(args.port, BAUD)
    dec = StreamDecoder()          # 预留：接收 STM32 回传的状态

    # ---------------- 模式 1：合成扫动目标（先调 STM32 端，不用摄像头）
    if args.mode == "sweep":
        print("sweep 模式：发送正弦扫动的假目标。Ctrl+C 退出。")
        start = time.time()
        dt = 1.0 / args.fps
        while True:
            t = time.time() - start
            cx = 320 + 240 * math.sin(2 * math.pi * 0.25 * t)      # 0.25 Hz 扫动
            cy = 240 + 120 * math.sin(2 * math.pi * 0.5 * t)
            frame = pack_target(cx, cy, 60, 80, 20 * math.sin(2 * math.pi * 0.3 * t))
            if ser:
                ser.write(frame)
            print(f"[{t:6.2f}s] cx={cx:6.1f} cy={cy:6.1f}  帧={len(frame)}B  {frame[:6].hex(' ')}")
            time.sleep(dt)

    # ---------------- 模式 3：单张图片
    if args.mode == "file":
        img = cv2.imread(args.file)
        if img is None:
            raise SystemExit(f"[错误] 读不到图片：{args.file}")
        t = find_target(img)
        print("识别结果:", {k: round(v, 2) for k, v in (t or {}).items() if k in ("cx", "cy", "w", "h", "angle", "area")})
        if t:
            cv2.imshow("result", draw_overlay(img, t, 0))
            cv2.waitKey(0)
        return 0

    # ---------------- 模式 2：真实摄像头（主循环）
    cap = open_camera(args.camera)
    print("camera 模式：按 q 退出。")

    t_prev = time.time()
    fps = 0.0
    sent = 0
    lost = 0
    t_start = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[警告] 读帧失败，跳过")
            continue

        t = find_target(frame)

        # ---- 发帧
        if t:
            data = pack_target(t["cx"], t["cy"], t["w"], t["h"], t["angle"])
            sent += 1
        else:
            data = pack_lost()
            lost += 1
        if ser:
            ser.write(data)

        # ---- 收帧（预留：把 STM32 的状态读回来显示）
        if ser:
            incoming = ser.read(64)
            if incoming:
                for p in dec.feed(incoming):
                    print("STM32 →", StreamDecoder.decode_target(p))

        # ---- 显示
        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-6)
        t_prev = now
        cv2.imshow("vision_host  (q to quit)", draw_overlay(frame, t, fps))

        # ---- 每 5 秒打印一次统计（W4 要的延迟/成功率数据就从这里来）
        if now - t_start > 5.0:
            total = sent + lost
            print(f"[统计] 帧率 {fps:.1f} | 识别 {sent}/{total} "
                  f"({100 * sent / max(total, 1):.1f}%) | 丢失 {lost}")
            t_start = now
            sent = lost = 0

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n已退出。")
