"""Wiggle ONE servo to check wiring. No calibration needed.

    sudo python3 Hardware/test_one_servo.py            # channel 0 on the 8.4 V board (hv, 0x40)
    sudo python3 Hardware/test_one_servo.py 13         # channel 13 on the 8.4 V board
    sudo python3 Hardware/test_one_servo.py 3 lv       # channel 3 on the 5 V board (lv, 0x41)
    sudo python3 Hardware/test_one_servo.py 3 0x41     # same, by I2C address

Goes through the electrical box like everything else (REV 3): press RESET on
the box so the SERVO LED is on, then press Enter to arm (servo signals on).
No keyboard? Run with: sudo env POWERBOX_AUTO_ARM=5 python3 ...
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from power_box import EStop, PowerBox  # noqa: E402
from servo_output import BOARDS, ServoBus  # noqa: E402

HOME = 4900                         # ~1.5 ms at 50 Hz: centre of travel
WIGGLE = (4900, 4500, 5300, 4900)   # small swing either side of centre
STEP_S = 1.0

channel = int(sys.argv[1]) if len(sys.argv) > 1 else 0
board = sys.argv[2] if len(sys.argv) > 2 else "hv"
if board.lower().startswith("0x"):
    by_addr = {hex(a): n for n, a in BOARDS.items()}
    board = by_addr.get(hex(int(board, 16)), board)
if board not in BOARDS or not 0 <= channel <= 15:
    boards = ", ".join(f"{n} = {hex(a)}" for n, a in BOARDS.items())
    sys.exit(f"Unknown board '{sys.argv[2] if len(sys.argv) > 2 else board}' or channel {channel}. "
             f"Use a board name or address ({boards}) and a channel 0-15.\n{__doc__}")

bus = ServoBus(dry_run=False, boards=(board,))
power = PowerBox(on_safe=bus.release)
print(f"channel {channel} on the '{board}' board ({hex(BOARDS[board])})")
try:
    power.arm(lambda: bus.set(board, channel, HOME))
    for p in WIGGLE:
        bus.set(board, channel, p)
        print("  sent", p)
        power.sleep(STEP_S)
except EStop:
    print("stopped by the E-stop.")
except KeyboardInterrupt:
    print()
finally:
    power.close()
    bus.release()
    print("done -- signals off (OE HIGH), servo released.")
