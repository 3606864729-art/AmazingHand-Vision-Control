import time
import numpy as np

from rustypot import Scs0009PyController


# =========================
# 1. 基本设置
# =========================

SERIAL_PORT = "COM3"      # 改成你当前实际 COM 口
BAUDRATE = 1000000
TIMEOUT = 1.0

# 这里填你自己的 8 个 MiddlePos 校准值
# 顺序必须是：
# [ID1, ID2, ID3, ID4, ID5, ID6, ID7, ID8]
# 即：
# [食指1, 食指2, 中指1, 中指2, 无名指1, 无名指2, 拇指1, 拇指2]
MiddlePos = [-5, 8, 0, 0, 5, 0, 3, 0]   # 先替换成你自己的值

# 调零测试速度，建议不要太快
SPEED = 3

# 停顿时间
HOLD_TIME = 5

# 最大伸展角度：对应 Demo 里的 OpenHand()
EXTEND_ANGLE_1 = -35
EXTEND_ANGLE_2 = 35

# 最大收缩角度：对应 Demo 里的 CloseHand()
CONTRACT_ANGLE_1 = 90
CONTRACT_ANGLE_2 = -90

# 零位角度
ZERO_ANGLE_1 = 0
ZERO_ANGLE_2 = 0


# =========================
# 2. 控制器初始化
# =========================

c = Scs0009PyController(
    serial_port=SERIAL_PORT,
    baudrate=BAUDRATE,
    timeout=TIMEOUT,
)


# =========================
# 3. 基础函数
# =========================

def move_pair(id1, id2, mid_index1, mid_index2, angle1, angle2, speed=SPEED):
    """
    控制一根手指上的两个舵机。
    id1/id2: 实际舵机 ID
    mid_index1/mid_index2: 在 MiddlePos 列表里的索引
    angle1/angle2: 相对 MiddlePos 的角度偏移，单位：度
    """

    pos1 = np.deg2rad(MiddlePos[mid_index1] + angle1)
    pos2 = np.deg2rad(MiddlePos[mid_index2] + angle2)

    c.write_goal_speed(id1, speed)
    time.sleep(0.002)
    c.write_goal_speed(id2, speed)
    time.sleep(0.002)

    c.write_goal_position(id1, pos1)
    time.sleep(0.002)
    c.write_goal_position(id2, pos2)
    time.sleep(0.002)


def move_all_fingers(angle1, angle2, label):
    """
    所有手指一起运动。
    angle1 控制每组里的第一个舵机：ID 1,3,5,7
    angle2 控制每组里的第二个舵机：ID 2,4,6,8
    """

    print(label)

    # 食指：ID 1, 2
    move_pair(1, 2, 0, 1, angle1, angle2)

    # 中指：ID 3, 4
    move_pair(3, 4, 2, 3, angle1, angle2)

    # 无名指：ID 5, 6
    move_pair(5, 6, 4, 5, angle1, angle2)

    # 拇指：ID 7, 8
    move_pair(7, 8, 6, 7, angle1, angle2)


def zero_position():
    move_all_fingers(ZERO_ANGLE_1, ZERO_ANGLE_2, "Move to ZERO position")


def extend_max():
    move_all_fingers(EXTEND_ANGLE_1, EXTEND_ANGLE_2, "Move to MAX EXTENSION")


def contract_max():
    move_all_fingers(CONTRACT_ANGLE_1, CONTRACT_ANGLE_2, "Move to MAX CONTRACTION")


# =========================
# 4. 主循环
# =========================

def main():
    print("=== Amazing Hand Zero Fine Tuning Test ===")
    print("Press Ctrl + C to stop.")

    # 先回到零位
    zero_position()
    time.sleep(HOLD_TIME)

    while True:
        extend_max()
        time.sleep(HOLD_TIME)

        contract_max()
        time.sleep(HOLD_TIME)

        zero_position()
        time.sleep(HOLD_TIME)


if __name__ == "__main__":
    main()