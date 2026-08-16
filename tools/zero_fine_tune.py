# zero_fine_tune.py
# 适用场景：
# 1. Windows + Waveshare BUS SERVO ADAPTER (A)
# 2. Feetech SCS0009，总线舵机 ID 为 1~8
# 3. 用于整手“零位 -> 最大伸展 -> 最大收缩 -> 回零”的循环测试/精细调零
#
# 使用方法：
# 1. 设置 AMAZINGHAND_SERIAL_PORT，例如 "COM3" 或 "COM11"
# 2. 先空载或确保机械结构不会卡死，再运行：
#       python tools/zero_fine_tune.py
# 3. 按 Ctrl+C 退出；默认退出前会尝试回到零位

import os
import time
import sys

try:
    import numpy as np
    from rustypot import Scs0009PyController
except ImportError as e:
    print("缺少依赖。请先运行：")
    print("  py -m pip install numpy rustypot pyserial")
    raise


# ===================== 硬件与标定参数 =====================

SERIAL_PORT = os.getenv("AMAZINGHAND_SERIAL_PORT", "COM3")
BAUDRATE = 1_000_000
TIMEOUT = 1.0

SERVO_IDS = [1, 2, 3, 4, 5, 6, 7, 8]

# 每个舵机的机械零位补偿，顺序对应 ID 1~8。
# 初始值可设为全 0，精调后记录各舵机中位补偿值。
# 官方 demo 示例是：[3, 0, -5, -8, -2, 5, -12, 0]
MIDDLE_POS_DEG = [5, -3, 0, 2, 0, 0, 0, 0]

# 最大伸展/最大收缩角度。
# 开合方向沿用 AmazingHand 官方 demo 的基础映射：
# 伸展：每对舵机约 -35 / +35 deg
# 收缩：每对舵机约 +90 / -90 deg
OPEN_OFFSET_DEG  = [-35,  35, -35,  35, -35,  35, -35,  35]
CLOSE_OFFSET_DEG = [ 90, -90,  90, -90,  90, -90,  90, -90]
ZERO_OFFSET_DEG  = [  0,   0,   0,   0,   0,   0,   0,   0]

# 安全参数
SPEED = 3                 # 1~7；初次调试建议 2 或 3，不要直接用 7
PAUSE_AFTER_OPEN = 5.0
PAUSE_AFTER_CLOSE = 5.0
PAUSE_AFTER_ZERO = 2.0
COMMAND_GAP = 0.03        # 每条总线命令之间的间隔；通信不稳时可加到 0.05
DISABLE_TORQUE_ON_EXIT = False  # True: 退出时释放舵机；False: 退出时保持力矩

# ============================================================


def print_ports_hint():
    """打印当前可见串口，便于检查 COM 号。"""
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("当前没有检测到任何串口。")
        else:
            print("当前检测到的串口：")
            for p in ports:
                print(f"  {p.device}  {p.description}")
    except Exception:
        print("无法列出串口；这不影响主程序运行。")


def make_controller():
    print(f"正在连接：{SERIAL_PORT}, baudrate={BAUDRATE}, timeout={TIMEOUT}")
    try:
        return Scs0009PyController(
            serial_port=SERIAL_PORT,
            baudrate=BAUDRATE,
            timeout=TIMEOUT,
        )
    except OSError as e:
        print("\n打开串口失败。常见原因：")
        print("1. SERIAL_PORT 写错，例如脚本里是 COM3，但设备实际是 COM11。")
        print("2. FD Debug Software、Arduino IDE 串口监视器、其他 Python 程序正在占用串口。")
        print("3. USB 转串口/总线板状态异常，需要重新插拔 USB。")
        print("4. Windows 设备管理器里端口短暂失效。")
        print()
        print_ports_hint()
        raise e


def enable_torque_all(c):
    print("正在给 8 个舵机开启扭矩...")
    for sid in SERVO_IDS:
        try:
            c.write_torque_enable(sid, 1)  # 1=On, 2=Off, 3=Free，沿用官方注释
            time.sleep(COMMAND_GAP)
            print(f"  ID {sid}: torque ON")
        except Exception as e:
            print(f"\nID {sid} 开启扭矩失败：{repr(e)}")
            print("请优先检查：该 ID 是否真的存在、是否有重复 ID、供电是否足够、总线线序是否正确。")
            raise


def set_speed_all(c, speed):
    for sid in SERVO_IDS:
        c.write_goal_speed(sid, speed)
        time.sleep(COMMAND_GAP)


def move_to_offsets(c, name, offsets_deg):
    """移动到 MIDDLE_POS_DEG + offsets_deg。"""
    print(f"\n移动到：{name}")
    set_speed_all(c, SPEED)

    for i, sid in enumerate(SERVO_IDS):
        target_deg = MIDDLE_POS_DEG[i] + offsets_deg[i]
        target_rad = np.deg2rad(target_deg)
        c.write_goal_position(sid, target_rad)
        time.sleep(COMMAND_GAP)
        print(f"  ID {sid}: {target_deg:+.1f} deg")


def try_return_to_zero(c):
    try:
        move_to_offsets(c, "零位", ZERO_OFFSET_DEG)
        time.sleep(0.8)
    except Exception as e:
        print(f"回零失败：{repr(e)}")


def maybe_disable_torque(c):
    if not DISABLE_TORQUE_ON_EXIT:
        return
    print("正在释放舵机扭矩...")
    for sid in SERVO_IDS:
        try:
            c.write_torque_enable(sid, 3)
            time.sleep(COMMAND_GAP)
        except Exception as e:
            print(f"  ID {sid} 释放扭矩失败：{repr(e)}")


def main():
    print("AmazingHand 全舵机调零循环脚本")
    print("按 Ctrl+C 可退出。")
    print_ports_hint()

    c = make_controller()

    try:
        enable_torque_all(c)

        # 启动时先回到零位，避免直接冲到极限位置
        move_to_offsets(c, "启动零位", ZERO_OFFSET_DEG)
        time.sleep(PAUSE_AFTER_ZERO)

        cycle = 1
        while True:
            print(f"\n========== Cycle {cycle} ==========")

            move_to_offsets(c, "最大伸展 / Open", OPEN_OFFSET_DEG)
            time.sleep(PAUSE_AFTER_OPEN)

            move_to_offsets(c, "最大收缩 / Close", CLOSE_OFFSET_DEG)
            time.sleep(PAUSE_AFTER_CLOSE)

            move_to_offsets(c, "回到零位 / Zero", ZERO_OFFSET_DEG)
            time.sleep(PAUSE_AFTER_ZERO)

            cycle += 1

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，准备退出。")
        try_return_to_zero(c)
        maybe_disable_torque(c)
        print("已退出。")

    except RuntimeError as e:
        print("\n运行时通信失败：", repr(e))
        print("这通常对应 Operation timed out。建议按以下顺序排查：")
        print("1. 只接一个舵机测试，确认该 ID 能被 FD 搜到。")
        print("2. 确认 FD Debug Software 已关闭，不能和 Python 同时占用总线。")
        print("3. 检查电源：8 个舵机同时动作时，5V 2A 可能偏紧，建议更大电流余量。")
        print("4. 把 COMMAND_GAP 改大到 0.05，TIMEOUT 改到 2.0。")
        print("5. 如果某个固定 ID 失败，重点查那个舵机的 ID、线序、插头和供电。")
        try_return_to_zero(c)
        maybe_disable_torque(c)
        sys.exit(1)

    except Exception as e:
        print("\n发生其他错误：", repr(e))
        try_return_to_zero(c)
        maybe_disable_torque(c)
        raise


if __name__ == "__main__":
    main()
