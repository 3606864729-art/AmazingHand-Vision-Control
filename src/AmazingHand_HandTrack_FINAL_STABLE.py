# AmazingHand_HandTrack_FINAL_STABLE.py
# AmazingHand hand tracking：左右摆动标定增强版（已合并你的 TUNED_THUMB_FIX 参数，并加入半弯曲友好曲线、单指 splay 衰减、左右偏置微调、开手限速防误展开）
#
# 核心改进：
# 1. 四指左右摆动不再用 fingertip - MCP，而是用 MCP -> PIP 的近节指骨方向，减少屈伸污染。
# 2. 拇指左右摆动单独处理，用 CMC->MCP 与 CMC->TIP 的混合指标，提高可见幅度。
# 3. 增加中立姿态标定：张开手、保持自然中立，按 n 记录 neutral splay。
# 4. 终端持续输出 raw_splay / cmd_splay，便于判断到底是识别问题还是舵机映射问题。
#
# 运行：
#   python src/AmazingHand_HandTrack_FINAL_STABLE.py
#
# 操作：
#   n     标定当前手势为左右摆动中立位
#   q     退出
#   space 手动张开
#   c     手动闭合
#
# 重要：
# - 运行前关闭所有旧脚本、官方 demo、FD 调试软件，确保没有其他程序占用 COM3。
# - 第一次测试时让机械手空载，手放在电源开关附近，发现顶死立刻断电。

import os
import time
import math
from dataclasses import dataclass

import cv2
import numpy as np
import mediapipe as mp


# ===================== 你主要需要改这里 =====================

SIMULATE_ONLY = os.getenv("AMAZINGHAND_SIMULATE_ONLY", "0").lower() in {"1", "true", "yes", "on"}

SERIAL_PORT = os.getenv("AMAZINGHAND_SERIAL_PORT", "COM3")
BAUDRATE = 1_000_000
TIMEOUT = 1.0

SERVO_IDS = [1, 2, 3, 4, 5, 6, 7, 8]

# 改成你的真实调零值。
# 官方 demo 示例：[3, 0, -5, -8, -2, 5, -12, 0]
MIDDLE_POS_DEG = [-5, 8, 0, 0, 5, 0, 3, 0]

# 屈伸：反向转动
OPEN_OFFSET_DEG  = [-35,  35, -35,  35, -35,  35, -35,  35]
CLOSE_OFFSET_DEG = [ 90, -90,  90, -90,  90, -90,  85, -85]

# 左右摆动：同向叠加
SPLAY_ENABLE = True

# index, middle, ring, thumb
# 如果左右摆幅小，优先调大 SPLAY_GAIN；如果舵机幅度小，再调大 SPLAY_MAX_DEG。
SPLAY_MAX_DEG = [80, 80, 80, 80]
SPLAY_GAIN    = [3.2, 3.0, 3.2, 9.0]  # 单指独立衰减：伸展手指保留摆动，弯曲手指削弱摆动
SPLAY_SIGN    = [-1, -1, -1,  -1]   # 某根方向反了，就把对应值改成 -1

# 每根手指左右摆动零点微调，单位：度。
# 顺序：index, middle, ring, thumb
# 你现在的问题是中指偏向食指，所以先给 middle 一个 +8° 偏置。
# 如果更偏，就改成 [0, -8, 0, 0]。
SPLAY_TRIM_DEG = [0, 10, 0, 0]

# 安全限位
ANGLE_LIMITS_DEG = [
    (-90, 95), (-95, 90),
    (-90, 95), (-95, 90),
    (-90, 95), (-95, 90),
    (-90, 95), (-95, 90),
]

CAMERA_INDEX = int(os.getenv("AMAZINGHAND_CAMERA_INDEX", "0"))

CONTROL_HZ = 14
SMOOTHING_ALPHA = 0.25

SPEED = 4
COMMAND_GAP = 0.006

# 半弯曲友好参数说明：
# - CURL_FULL 提高到 0.82：需要更明显弯曲才会满闭合；
# - CURL_GAMMA 改到 1.10：曲线更线性，半弯曲更容易停住；
# - CURL_LOCK_ON 提高到 0.82：避免半弯曲时过早锁死；
# - SMOOTHING_ALPHA 降到 0.25：减少跳变。

# 弯曲灵敏度
CURL_START = 0.12
CURL_FULL = 0.82
CURL_GAMMA = 1.10

# 拇指弯曲单独放大
THUMB_CURL_START = 0.03
THUMB_CURL_FULL = 0.68
THUMB_CURL_GAMMA = 0.85
THUMB_CLOSE_GAIN = 1.10

OPEN_JOINT_ANGLE = 165.0
CLOSE_JOINT_ANGLE = 105.0

HOLD_WHEN_NO_HAND = True

# 防止“中指/无名指握拢后被视觉误判为逐渐张开”
# 原理：
# 1. 某根手指 cmd curl 超过 CURL_LOCK_ON 后，认为它已经握拢；
# 2. 之后即使 MediaPipe 因遮挡导致 curl 短暂变小，也保持至少 CURL_LOCK_MIN 的闭合；
# 3. 只有连续多帧低于 CURL_LOCK_OFF，才解除锁定。
CURL_LOCK_ENABLE = True
CURL_LOCK_ON = 0.72
CURL_LOCK_OFF = 0.30
CURL_LOCK_MIN = 0.72
CURL_LOCK_RELEASE_FRAMES = 8

# 开手限速 / 防误展开：
# 视觉误识别时，某根已经弯曲的手指会突然被估计为张开。
# 这里允许“闭合”快速响应，但限制“张开”的速度，从而保留半弯曲，又避免其他手指被误带开。
CURL_TEMPORAL_FILTER_ENABLE = True
CURL_CLOSE_ALPHA = 0.70       # 新识别更弯曲时，快速跟随
CURL_OPEN_ALPHA = 0.45        # 新识别更张开时，慢速释放
CURL_MAX_OPEN_STEP = 0.045    # 每个控制周期最多张开这么多
CURL_DROP_GUARD_THRESHOLD = 0.70  # 如果单帧下降超过该值，认为可能是误识别，进一步限制


# 防止左右摆动把握拢手指“拉开”
# 手指越弯曲，splay 权重越小；完全握拢时几乎不叠加左右摆。
SPLAY_FADE_WITH_CURL = True
SPLAY_FADE_START = 0.12
SPLAY_FADE_FULL = 0.62

# 单指独立握拢优先：
# 只关闭“当前这根弯曲手指”的 splay，不影响其他伸展手指。
# 例如：食指伸展、中指/无名指握拢时，食指仍然可以左右摆，中指/无名指的 splay 被关闭。
SPLAY_CUTOFF_CURL = 0.74


# 对“食指伸展，其他手指握拢”的演示手势特别有用：
# index 可以保持左右摆，其余握拢手指不被 splay 干扰。

# 是否在识别到手后的前若干帧自动标定中立位。
# 推荐 True：程序启动后保持手自然张开 1 秒左右，会自动标定。
AUTO_CALIBRATE_NEUTRAL = True
AUTO_CALIBRATE_FRAMES = 25

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
    splay_index: int


FINGERS = [
    FingerDef("index",  5,  6,  7,  8, slice(0, 2), 0),
    FingerDef("middle", 9, 10, 11, 12, slice(2, 4), 1),
    FingerDef("ring",  13, 14, 15, 16, slice(4, 6), 2),
    FingerDef("thumb", 1, 2, 3, 4, slice(6, 8), 3),
]


FINGER_NAMES = ["index", "middle", "ring", "thumb"]


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def landmark_xyz(lm):
    return np.array([lm.x, lm.y, lm.z], dtype=float)


def norm2(v):
    n = np.linalg.norm(v[:2])
    if n < 1e-9:
        return None
    return v[:2] / n


def angle_deg(a, b, c):
    va = a - b
    vc = c - b
    na = np.linalg.norm(va)
    nc = np.linalg.norm(vc)
    if na < 1e-9 or nc < 1e-9:
        return 180.0
    cosv = np.dot(va, vc) / (na * nc)
    cosv = clamp(cosv, -1.0, 1.0)
    return math.degrees(math.acos(cosv))


def raw_curl_from_angle(theta):
    curl = (OPEN_JOINT_ANGLE - theta) / (OPEN_JOINT_ANGLE - CLOSE_JOINT_ANGLE)
    return clamp(curl, 0.0, 1.0)


def command_curl(raw, finger_name=None):
    if finger_name == "thumb":
        x = (raw - THUMB_CURL_START) / max(1e-6, (THUMB_CURL_FULL - THUMB_CURL_START))
        x = clamp(x, 0.0, 1.0)
        return x ** THUMB_CURL_GAMMA

    x = (raw - CURL_START) / max(1e-6, (CURL_FULL - CURL_START))
    x = clamp(x, 0.0, 1.0)
    return x ** CURL_GAMMA


def estimate_finger_curls(hand_landmarks):
    lms = hand_landmarks.landmark
    raw_curls = {}

    for f in FINGERS:
        p_mcp = landmark_xyz(lms[f.mcp])
        p_pip = landmark_xyz(lms[f.pip])
        p_dip = landmark_xyz(lms[f.dip])
        p_tip = landmark_xyz(lms[f.tip])

        theta1 = angle_deg(p_mcp, p_pip, p_dip)
        theta2 = angle_deg(p_pip, p_dip, p_tip)

        if f.name == "thumb":
            theta = 0.8 * theta1 + 0.2 * theta2
        else:
            theta = 0.65 * theta1 + 0.35 * theta2

        raw_curls[f.name] = raw_curl_from_angle(theta)

    cmd_curls = {k: command_curl(v, k) for k, v in raw_curls.items()}
    return raw_curls, cmd_curls


def get_palm_axis(hand_landmarks):
    """
    返回图像中的手掌横轴。
    使用 index_mcp -> pinky_mcp 作为横向轴。
    """
    lms = hand_landmarks.landmark
    index_mcp = landmark_xyz(lms[5])
    pinky_mcp = landmark_xyz(lms[17])
    axis = pinky_mcp - index_mcp
    axis2 = norm2(axis)
    if axis2 is None:
        return np.array([1.0, 0.0])
    return axis2


def estimate_raw_splay(hand_landmarks):
    """
    返回未标定的 raw_splay。
    四指：用 MCP -> PIP 方向投影到手掌横轴。
    拇指：用 CMC -> MCP 和 CMC -> TIP 混合，增强可见幅度。
    """
    lms = hand_landmarks.landmark
    palm_axis = get_palm_axis(hand_landmarks)

    raw = {}

    # index/middle/ring: use proximal phalanx direction MCP -> PIP
    for f in FINGERS:
        if f.name == "thumb":
            continue

        mcp = landmark_xyz(lms[f.mcp])
        pip = landmark_xyz(lms[f.pip])
        v = norm2(pip - mcp)
        if v is None:
            raw[f.name] = 0.0
        else:
            raw[f.name] = float(np.dot(v, palm_axis))

    # thumb: mixed root-tip measurement
    thumb_cmc = landmark_xyz(lms[1])
    thumb_mcp = landmark_xyz(lms[2])
    thumb_tip = landmark_xyz(lms[4])

    v_root = norm2(thumb_mcp - thumb_cmc)
    v_tip = norm2(thumb_tip - thumb_cmc)

    root_val = float(np.dot(v_root, palm_axis)) if v_root is not None else 0.0
    tip_val = float(np.dot(v_tip, palm_axis)) if v_tip is not None else 0.0

    # tip_val 能更明显地反映大幅度摆动，但也会受屈伸影响；
    # root_val 更纯，但幅度小。这里做折中。
    raw["thumb"] = 0.45 * root_val + 0.55 * tip_val

    return raw


def make_zero_splay():
    return {name: 0.0 for name in FINGER_NAMES}


def make_lock_state():
    return {name: False for name in FINGER_NAMES}


def make_release_count():
    return {name: 0 for name in FINGER_NAMES}


def make_filtered_curls():
    return {name: 0.0 for name in FINGER_NAMES}


def apply_curl_temporal_filter(cmd_curls, filtered_curls):
    """
    时间滤波：闭合快，张开慢。
    目的：防止你动食指时，中指/无名指因 MediaPipe 遮挡误差而突然展开。
    """
    if not CURL_TEMPORAL_FILTER_ENABLE:
        return cmd_curls.copy()

    out = {}

    for name in FINGER_NAMES:
        prev = float(filtered_curls.get(name, 0.0))
        new = float(cmd_curls.get(name, 0.0))

        if new >= prev:
            # 用户正在弯曲，快速跟随
            val = (1.0 - CURL_CLOSE_ALPHA) * prev + CURL_CLOSE_ALPHA * new
        else:
            # 用户正在张开或被误识别为张开，慢速释放
            drop = prev - new
            alpha = CURL_OPEN_ALPHA

            if drop > CURL_DROP_GUARD_THRESHOLD:
                alpha *= 0.45

            val = (1.0 - alpha) * prev + alpha * new
            val = max(val, prev - CURL_MAX_OPEN_STEP)

        val = clamp(val, 0.0, 1.0)
        filtered_curls[name] = val
        out[name] = val

    return out


def apply_curl_lock(cmd_curls, lock_state, release_count):
    """
    防止已握拢手指因为遮挡/误识别而逐渐张开。
    返回 locked_curls，同时原地更新 lock_state 和 release_count。
    """
    if not CURL_LOCK_ENABLE:
        return cmd_curls.copy()

    locked = {}

    for name in FINGER_NAMES:
        curl = float(cmd_curls.get(name, 0.0))

        if lock_state[name]:
            # 只有连续多帧明显低于阈值，才认为用户真的想张开
            if curl < CURL_LOCK_OFF:
                release_count[name] += 1
            else:
                release_count[name] = 0

            if release_count[name] >= CURL_LOCK_RELEASE_FRAMES:
                lock_state[name] = False
                release_count[name] = 0
                locked[name] = curl
            else:
                locked[name] = max(curl, CURL_LOCK_MIN)
        else:
            if curl >= CURL_LOCK_ON:
                lock_state[name] = True
                release_count[name] = 0
                locked[name] = max(curl, CURL_LOCK_MIN)
            else:
                locked[name] = curl

    return locked


def splay_fade_from_curl(curl):
    """
    单指独立 splay 衰减。
    只根据这根手指自己的 curl 决定是否削弱这根手指的 splay。
    不使用全局握拳平均值，因此不会影响仍然伸展的食指/拇指。
    """
    if not SPLAY_FADE_WITH_CURL:
        return 1.0

    if curl >= SPLAY_CUTOFF_CURL:
        return 0.0

    x = (curl - SPLAY_FADE_START) / max(1e-6, (SPLAY_FADE_FULL - SPLAY_FADE_START))
    x = clamp(x, 0.0, 1.0)

    # 二次衰减：轻微弯曲时还能摆，明显弯曲后迅速关闭
    return (1.0 - x) ** 2


def subtract_neutral_and_amplify(raw_splay, neutral_splay, cmd_curls=None):
    """
    只对每根手指独立削弱 splay：
    - 这根手指伸展：splay 保留；
    - 这根手指弯曲：splay 减弱；
    - 其他手指的弯曲状态不影响这根手指。
    """
    cmd = {}
    for i, name in enumerate(FINGER_NAMES):
        delta = raw_splay.get(name, 0.0) - neutral_splay.get(name, 0.0)
        val = clamp(SPLAY_SIGN[i] * SPLAY_GAIN[i] * delta, -1.0, 1.0)

        if cmd_curls is not None:
            val *= splay_fade_from_curl(cmd_curls.get(name, 0.0))

        cmd[name] = clamp(val, -1.0, 1.0)
    return cmd


def curls_splay_to_servo_offsets(cmd_curls, cmd_splay):
    target = np.array(OPEN_OFFSET_DEG, dtype=float)
    close = np.array(CLOSE_OFFSET_DEG, dtype=float)

    for f in FINGERS:
        s = f.servo_slice
        curl = cmd_curls[f.name]

        if f.name == "thumb":
            curl = clamp(curl * THUMB_CLOSE_GAIN, 0.0, 1.0)

        # 反向转动：屈伸
        target[s] = (1.0 - curl) * target[s] + curl * close[s]

        # 同向叠加：左右摆动
        # SPLAY_TRIM_DEG 是每根手指的常量同向偏置，用于修正“某根手指总是偏一边”。
        si = f.splay_index
        trim_deg = SPLAY_TRIM_DEG[si]

        if SPLAY_ENABLE:
            splay_deg = SPLAY_MAX_DEG[si] * cmd_splay[f.name] + trim_deg
            target[s] += splay_deg
        else:
            target[s] += trim_deg

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
        time.sleep(0.03)

    for sid in SERVO_IDS:
        c.write_goal_speed(sid, SPEED)
        time.sleep(0.03)

    return c


def move_all(c, offsets_deg):
    if c is None:
        return

    for i, sid in enumerate(SERVO_IDS):
        target_deg = MIDDLE_POS_DEG[i] + float(offsets_deg[i])
        c.write_goal_position(sid, np.deg2rad(target_deg))
        time.sleep(COMMAND_GAP)


def draw_status(frame, raw_curls, cmd_curls, raw_splay, cmd_splay, neutral_splay, has_hand, calibrated, lock_state):
    y = 30
    cv2.putText(
        frame,
        "FINAL STABLE | n: neutral | r: unlock | q: quit | space: open | c: close",
        (10, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        2,
    )
    y += 27

    cv2.putText(
        frame,
        f"hand: {'YES' if has_hand else 'NO'} | calibrated: {'YES' if calibrated else 'NO'}",
        (10, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        2,
    )
    y += 27

    for name in FINGER_NAMES:
        cv2.putText(
            frame,
            f"{name:6s}: curl {cmd_curls.get(name,0):.2f} | sp {cmd_splay.get(name,0):+.2f} | lock {str(lock_state.get(name, False))[0]}",
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
        )
        y += 23


def main():
    c = connect_controller()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {CAMERA_INDEX}")

    raw_curls = {name: 0.0 for name in FINGER_NAMES}
    cmd_curls = {name: 0.0 for name in FINGER_NAMES}
    raw_splay = make_zero_splay()
    neutral_splay = make_zero_splay()
    cmd_splay = make_zero_splay()
    lock_state = make_lock_state()
    release_count = make_release_count()
    filtered_curls = make_filtered_curls()

    target_offsets = np.array(OPEN_OFFSET_DEG, dtype=float)
    current_offsets = np.array(OPEN_OFFSET_DEG, dtype=float)
    move_all(c, current_offsets)

    last_control_t = 0.0
    last_debug_t = 0.0

    calibrated = not AUTO_CALIBRATE_NEUTRAL
    auto_samples = []

    print("Camera hand tracking FINAL STABLE mode started.")
    print("Hold your hand naturally open and press 'n' to set neutral splay.")
    print("If AUTO_CALIBRATE_NEUTRAL=True, keep your hand naturally open for the first second.")
    print("Keys: n=neutral, r=unlock all finger locks, q=quit, space=open, c=close")

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
                    print("Camera read failed.")
                    time.sleep(0.1)
                    continue

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

                    raw_curls, cmd_curls_raw = estimate_finger_curls(hand_lms)
                    cmd_curls_smooth = apply_curl_temporal_filter(cmd_curls_raw, filtered_curls)
                    cmd_curls = apply_curl_lock(cmd_curls_smooth, lock_state, release_count)
                    raw_splay = estimate_raw_splay(hand_lms)

                    if AUTO_CALIBRATE_NEUTRAL and not calibrated:
                        auto_samples.append(raw_splay.copy())
                        if len(auto_samples) >= AUTO_CALIBRATE_FRAMES:
                            neutral_splay = {
                                name: float(np.mean([s[name] for s in auto_samples]))
                                for name in FINGER_NAMES
                            }
                            calibrated = True
                            print("AUTO neutral_splay:", {k: round(v, 3) for k, v in neutral_splay.items()})

                    cmd_splay = subtract_neutral_and_amplify(raw_splay, neutral_splay, cmd_curls)
                    target_offsets = curls_splay_to_servo_offsets(cmd_curls, cmd_splay)
                else:
                    if not HOLD_WHEN_NO_HAND:
                        target_offsets = np.array(OPEN_OFFSET_DEG, dtype=float)

                now = time.time()

                if now - last_control_t >= 1.0 / CONTROL_HZ:
                    current_offsets = (
                        (1.0 - SMOOTHING_ALPHA) * current_offsets
                        + SMOOTHING_ALPHA * target_offsets
                    )
                    move_all(c, current_offsets)
                    last_control_t = now

                if now - last_debug_t >= 0.5:
                    print(
                        "hand:", has_hand,
                        "curl:", {k: round(v, 2) for k, v in cmd_curls.items()},
                        "raw_sp:", {k: round(v, 3) for k, v in raw_splay.items()},
                        "neutral:", {k: round(v, 3) for k, v in neutral_splay.items()},
                        "cmd_sp:", {k: round(v, 2) for k, v in cmd_splay.items()},
                        "lock:", {k: lock_state[k] for k in FINGER_NAMES},
                        "target:", np.round(target_offsets, 1),
                    )
                    last_debug_t = now

                draw_status(frame, raw_curls, cmd_curls, raw_splay, cmd_splay, neutral_splay, has_hand, calibrated, lock_state)
                cv2.imshow("AmazingHand HandTrack SPLAY CALIB", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("n"):
                    neutral_splay = raw_splay.copy()
                    calibrated = True
                    auto_samples = []
                    print("MANUAL neutral_splay:", {k: round(v, 3) for k, v in neutral_splay.items()})
                elif key == ord("r"):
                    print("Clear curl locks")
                    lock_state = make_lock_state()
                    release_count = make_release_count()
                    filtered_curls = make_filtered_curls()
                elif key == ord(" "):
                    print("Manual OPEN")
                    lock_state = make_lock_state()
                    release_count = make_release_count()
                    filtered_curls = make_filtered_curls()
                    target_offsets = np.array(OPEN_OFFSET_DEG, dtype=float)
                    current_offsets = target_offsets.copy()
                    move_all(c, current_offsets)
                elif key == ord("c"):
                    print("Manual CLOSE")
                    target_offsets = np.array(CLOSE_OFFSET_DEG, dtype=float)
                    current_offsets = target_offsets.copy()
                    move_all(c, current_offsets)

    except KeyboardInterrupt:
        pass
    finally:
        print("Returning to OPEN...")
        try:
            move_all(c, np.array(OPEN_OFFSET_DEG, dtype=float))
        except Exception as e:
            print("Return failed:", repr(e))
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
