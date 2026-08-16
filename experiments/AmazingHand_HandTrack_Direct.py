# AmazingHand_HandTrack_Direct.py
# 轻量版手部追踪控制：摄像头 -> MediaPipe Hands -> AmazingHand 8 个 SCS0009 舵机
#
# 适合你当前阶段：
# - 已完成舵机 ID 设置
# - 已完成调零
# - 已经能运行官方 demo
# - 想直接用摄像头识别人手开合来控制机械手
#
# 安装依赖：
#   python -m pip install opencv-python mediapipe numpy rustypot pyserial
#
# 注意：
# - MediaPipe 通常要求 Python 3.9~3.12；如果你现在是 Python 3.14，建议新建 Python 3.12 虚拟环境。
# - 第一次运行时，请不要让机械手抓住硬物，也不要让手指顶死。
# - 先把 CLOSE_OFFSET_DEG 调小，例如 60/-60，确认安全后再加大。

import time
import math
from dataclasses import dataclass

import cv2
import numpy as np
import mediapipe as mp


# ===================== 你主要需要改这里 =====================

SERIAL_PORT = "COM3"       # 改成你的实际端口，例如 "COM3" 或 "COM11"
BAUDRATE = 1_000_000
TIMEOUT = 1.0

SERVO_IDS = [1, 2, 3, 4, 5, 6, 7, 8]

# 填入你调零后的真实中位值，顺序对应 ID 1~8。
# 官方 demo 示例：[3, 0, -5, -8, -2, 5, -12, 0]
MIDDLE_POS_DEG = [-5, 8, 0, 0, 5, 0, 3, 0]

# 开手和握拳的舵机相对角度。
# 如果你发现机械结构会卡，优先减小 CLOSE_OFFSET_DEG，例如改成 60/-60。
OPEN_OFFSET_DEG  = [-35,  35, -35,  35, -35,  35, -35,  35]
CLOSE_OFFSET_DEG = [ 80, -80,  80, -80,  80, -80,  80, -80]

# 每个舵机最终允许的相对角度范围，防止误识别导致撞限位。
# 格式：[(min1,max1), (min2,max2), ...]
ANGLE_LIMITS_DEG = [
    (-45, 90), (-90, 45),
    (-45, 90), (-90, 45),
    (-45, 90), (-90, 45),
    (-45, 90), (-90, 45),
]

# True：只打开摄像头看识别效果，不连接舵机；False：真实控制机械手。
# 建议第一次先设为 True，确认识别稳定后再改 False。
SIMULATE_ONLY = True

CAMERA_INDEX = 0
CONTROL_HZ = 20             # 控制频率，不建议太高
SPEED = 3                   # 1~7；初次建议 2 或 3
SMOOTHING_ALPHA = 0.25      # 越小越平滑，但延迟越大
COMMAND_GAP = 0.002         # 通信不稳时可增大到 0.01 或 0.02

# 手指弯曲角度标定：
# PIP/IP 关节角接近 175° 视为张开，接近 70° 视为闭合。
# 不同摄像头角度会影响识别，可以按实际效果微调。
OPEN_JOINT_ANGLE = 170.0
CLOSE_JOINT_ANGLE = 75.0

# ============================================================


mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles


@dataclass
class FingerDef:
    name: str
    mcp: int
    pip: int
    dip: int
    tip: int
    servo_slice: slice


FINGERS = [
    FingerDef("index",  5,  6,  7,  8, slice(0, 2)),
    FingerDef("middle", 9, 10, 11, 12, slice(2, 4)),
    FingerDef("ring",  13, 14, 15, 16, slice(4, 6)),
    # 拇指没有标准 PIP/DIP 命名，这里用 CMC/MCP/IP/TIP 近似处理
    FingerDef("thumb", 1, 2, 3, 4, slice(6, 8)),
]


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def landmark_xyz(lm):
    return np.array([lm.x, lm.y, lm.z], dtype=float)


def angle_deg(a, b, c):
    """返回 abc 中以 b 为顶点的夹角，单位 degree。"""
    va = a - b
    vc = c - b
    na = np.linalg.norm(va)
    nc = np.linalg.norm(vc)
    if na < 1e-9 or nc < 1e-9:
        return 180.0
    cosv = np.dot(va, vc) / (na * nc)
    cosv = clamp(cosv, -1.0, 1.0)
    return math.degrees(math.acos(cosv))


def curl_from_angle(theta):
    """
    将人手关节角映射为弯曲程度：
    0 = 张开，1 = 闭合。
    """
    curl = (OPEN_JOINT_ANGLE - theta) / (OPEN_JOINT_ANGLE - CLOSE_JOINT_ANGLE)
    return clamp(curl, 0.0, 1.0)


def estimate_finger_curls(hand_landmarks):
    """
    估计四根手指的弯曲程度。
    返回 dict: index/middle/ring/thumb -> 0~1
    """
    lms = hand_landmarks.landmark
    curls = {}

    for f in FINGERS:
        p_mcp = landmark_xyz(lms[f.mcp])
        p_pip = landmark_xyz(lms[f.pip])
        p_dip = landmark_xyz(lms[f.dip])
        p_tip = landmark_xyz(lms[f.tip])

        # 主指标：PIP/IP 关节角
        theta1 = angle_deg(p_mcp, p_pip, p_dip)

        # 辅助指标：DIP/TIP 关节角，降低单点抖动
        theta2 = angle_deg(p_pip, p_dip, p_tip)

        # 拇指结构不同，降低 DIP 辅助权重
        if f.name == "thumb":
            theta = 0.8 * theta1 + 0.2 * theta2
        else:
            theta = 0.65 * theta1 + 0.35 * theta2

        curls[f.name] = curl_from_angle(theta)

    return curls


def curls_to_servo_offsets(curls):
    """
    将四根手指弯曲程度映射为 8 个舵机的相对角度。
    每根手指两个舵机：open -> close 做线性插值。
    """
    target = np.array(OPEN_OFFSET_DEG, dtype=float)
    close = np.array(CLOSE_OFFSET_DEG, dtype=float)

    for f in FINGERS:
        curl = curls[f.name]
        s = f.servo_slice
        target[s] = (1.0 - curl) * target[s] + curl * close[s]

    for i, (lo, hi) in enumerate(ANGLE_LIMITS_DEG):
        target[i] = clamp(target[i], lo, hi)

    return target


def connect_controller():
    if SIMULATE_ONLY:
        print("SIMULATE_ONLY=True: camera tracking only, servos will not move.")
        return None

    from rustypot import Scs0009PyController

    print(f"Connecting to {SERIAL_PORT} ...")
    c = Scs0009PyController(
        serial_port=SERIAL_PORT,
        baudrate=BAUDRATE,
        timeout=TIMEOUT,
    )

    print("Enabling torque and setting speed...")
    for sid in SERVO_IDS:
        c.write_torque_enable(sid, 1)
        time.sleep(0.02)
    for sid in SERVO_IDS:
        c.write_goal_speed(sid, SPEED)
        time.sleep(0.02)

    return c


def move_all(c, offsets_deg):
    """
    发送 8 个舵机目标角度。
    offsets_deg 是相对中位值的角度。
    """
    if c is None:
        return

    for i, sid in enumerate(SERVO_IDS):
        target_deg = MIDDLE_POS_DEG[i] + float(offsets_deg[i])
        c.write_goal_position(sid, np.deg2rad(target_deg))
        time.sleep(COMMAND_GAP)


def draw_status(frame, curls, paused, has_hand):
    y = 30
    cv2.putText(
        frame,
        "q: quit | space: open | c: close | p: pause/resume",
        (10, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    y += 28

    cv2.putText(
        frame,
        f"status: {'PAUSED' if paused else 'RUNNING'} | hand: {'YES' if has_hand else 'NO'}",
        (10, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    y += 28

    for name in ["index", "middle", "ring", "thumb"]:
        val = curls.get(name, 0.0)
        cv2.putText(
            frame,
            f"{name:6s}: {val:.2f}",
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )
        y += 24


def main():
    c = connect_controller()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {CAMERA_INDEX}")

    current_offsets = np.array(OPEN_OFFSET_DEG, dtype=float)
    target_offsets = np.array(OPEN_OFFSET_DEG, dtype=float)
    move_all(c, current_offsets)

    paused = False
    last_control_t = 0.0
    last_curls = {"index": 0.0, "middle": 0.0, "ring": 0.0, "thumb": 0.0}

    print("Camera hand tracking started.")
    print("Keys: q=quit, space=open, c=close, p=pause/resume")

    try:
        with mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=0,
            min_detection_confidence=0.55,
            min_tracking_confidence=0.55,
        ) as hands:
            while True:
                ok, frame = cap.read()
                if not ok:
                    continue

                # 镜像显示更符合直觉
                frame = cv2.flip(frame, 1)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                rgb.flags.writeable = False
                results = hands.process(rgb)
                rgb.flags.writeable = True

                has_hand = False

                if results.multi_hand_landmarks:
                    has_hand = True
                    hand_lms = results.multi_hand_landmarks[0]

                    mp_drawing.draw_landmarks(
                        frame,
                        hand_lms,
                        mp_hands.HAND_CONNECTIONS,
                        mp_styles.get_default_hand_landmarks_style(),
                        mp_styles.get_default_hand_connections_style(),
                    )

                    last_curls = estimate_finger_curls(hand_lms)
                    target_offsets = curls_to_servo_offsets(last_curls)

                now = time.time()
                if not paused and now - last_control_t >= 1.0 / CONTROL_HZ:
                    current_offsets = (
                        (1.0 - SMOOTHING_ALPHA) * current_offsets
                        + SMOOTHING_ALPHA * target_offsets
                    )
                    move_all(c, current_offsets)
                    last_control_t = now

                draw_status(frame, last_curls, paused, has_hand)
                cv2.imshow("AmazingHand HandTrack Direct", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("p"):
                    paused = not paused
                elif key == ord(" "):
                    target_offsets = np.array(OPEN_OFFSET_DEG, dtype=float)
                    current_offsets = target_offsets.copy()
                    move_all(c, current_offsets)
                elif key == ord("c"):
                    target_offsets = np.array(CLOSE_OFFSET_DEG, dtype=float)
                    current_offsets = target_offsets.copy()
                    move_all(c, current_offsets)

    except KeyboardInterrupt:
        pass
    finally:
        print("Returning to open pose...")
        try:
            move_all(c, np.array(OPEN_OFFSET_DEG, dtype=float))
        except Exception as e:
            print("Return-to-open failed:", repr(e))
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
