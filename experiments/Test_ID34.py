import numpy as np
from rustypot import Scs0009PyController

c = Scs0009PyController(
    serial_port="COM3",
    baudrate=1000000,
    timeout=1.0,
)

print("=== Test_ID34 new version ===")

for sid in [3, 4]:
    print("Testing ID", sid)
    try:
        pos = c.read_present_position(sid)
        print("raw position:", pos)
        print("degree position:", np.rad2deg(pos))
        print("ID", sid, "READ OK")
    except Exception as e:
        print("ID", sid, "READ FAILED")
        print(type(e).__name__, e)

print("Done.")
