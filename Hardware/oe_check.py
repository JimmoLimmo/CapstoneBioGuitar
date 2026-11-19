"""Check that OE (header pin 31) really turns the servo signals off.

    servo Hardware/oe_check.py              # channel 10 on the 'lv' board (0x41)
    servo Hardware/oe_check.py 0 hv         # channel 0 on the 'hv' board (0x40)

Both boards share pin 31, so run it once for a servo on each board.
Box powered, RESET pressed (SERVO LED on). The servo only moves a few degrees,
and it should only move in the steps marked OE LOW.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from power_box import EStop, PowerBox  # noqa: E402
from servo_output import BOARDS, ServoBus  # noqa: E402

PULSE_A = 4650
PULSE_B = 5150

channel = int(sys.argv[1]) if len(sys.argv) > 1 else 10
board = sys.argv[2] if len(sys.argv) > 2 else "lv"
if board not in BOARDS or not 0 <= channel <= 15:
    sys.exit(f"Use a channel 0-15 and a board name ({', '.join(BOARDS)}).\n{__doc__}")

bus = ServoBus(dry_run=False, boards=(board,))
power = PowerBox(on_safe=bus.release)
GPIO = power.GPIO


def set_oe(low):
    power.check()
    GPIO.output(power.oe, GPIO.LOW if low else GPIO.HIGH)


def moved(question):
    while True:
        a = input(f"  {question} [y/n] ").strip().lower()
        if a in ("y", "n"):
            return a == "y"


print(f"[oe] channel {channel} on the '{board}' board ({hex(BOARDS[board])}).")
print("[oe] Meter on pin 31 to pin 14 if you have one: 3.3 V when OE HIGH, about 0 V when OE LOW.")
answers = {}
try:
    input("[oe] Hands clear. Press Enter to start (Ctrl-C to quit)... ")

    print("\n1) OE HIGH, sending a pulse. The servo should NOT move.")
    set_oe(False)
    bus.set(board, channel, PULSE_A)
    power.sleep(1.5)
    answers["high_1"] = moved("Did it move?")

    print("\n2) OE LOW. The servo should move now (or hold firm if it was already there).")
    set_oe(True)
    power.sleep(1.5)
    answers["low_1"] = moved("Did it move or stiffen?")

    print("\n3) OE HIGH, sending a different pulse. The servo should NOT move.")
    set_oe(False)
    bus.set(board, channel, PULSE_B)
    power.sleep(1.5)
    answers["high_2"] = moved("Did it move?")

    print("\n4) OE LOW. The servo should move to the new spot now.")
    set_oe(True)
    power.sleep(1.5)
    answers["low_2"] = moved("Did it move?")
except EStop:
    print("[oe] stopped by the E-stop.")
finally:
    power.close()

if len(answers) == 4:
    print()
    if not answers["high_1"] and not answers["high_2"] and answers["low_2"]:
        print("[oe] OE works: signals stay off on HIGH and come on with LOW.")
    elif answers["high_1"] or answers["high_2"]:
        print("[oe] OE is NOT working: the servo moved with OE HIGH. Check the wire from pin 31 to")
        print("     this board's OE pin, and that R5 (10k) goes from pin 31 to 3.3 V, not to ground.")
    else:
        print("[oe] The servo never moved, even with OE LOW. Check servo power (SERVO LED, V+ on")
        print("     the board), the channel number, and that OE isn't tied HIGH somewhere else.")
