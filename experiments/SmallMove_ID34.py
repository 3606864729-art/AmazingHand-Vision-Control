import time
import numpy as np
from rustypot import Scs0009PyController

ID_1 = 1
ID_2 = 2
MiddlePos_1 = 0
MiddlePos_2 = -3

c = Scs0009PyController(
    serial_port="COM3",
    baudrate=1000000,
    timeout=1.0,
)

print("=== Small move test for ID 3 and 4 ===")

while True:
    print("Move A")
    c.write_goal_speed(ID_1, 3)
    c.write_goal_speed(ID_2, 3)
    c.write_goal_position(ID_1, np.deg2rad(MiddlePos_1 + 90))
    c.write_goal_position(ID_2, np.deg2rad(MiddlePos_2 - 90))
    time.sleep(2)

    print("Move B")
    c.write_goal_speed(ID_1, 3)
    c.write_goal_speed(ID_2, 3)
    c.write_goal_position(ID_1, np.deg2rad(MiddlePos_1 - 30))
    c.write_goal_position(ID_2, np.deg2rad(MiddlePos_2 + 30))
    time.sleep(2)
