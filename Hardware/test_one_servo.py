"""Wiggle ONE servo to check wiring. No calibration needed.

    sudo python3 Hardware/test_one_servo.py            # channel 0 on the 8.4 V board (0x42)
    sudo python3 Hardware/test_one_servo.py 13         # channel 13 on the 8.4 V board
    sudo python3 Hardware/test_one_servo.py 3 lv       # channel 3 on the 5 V board (0x41)

Goes through the electrical box like everything else: relays start OFF,
press Enter to arm, press the box's RESET button, then press Enter again.
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
if board not in BOARDS or not 0 <= channel <= 15:
    sys.exit(__doc__)

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
    print("done -- relays off, servo released.")
