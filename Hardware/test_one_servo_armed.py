"""Wiggle ONE servo through the REV 3 electrical box.

Follows the box's software rules through power_box.py: servo signals start
OFF (PCA9685 OE, pin 31, HIGH), nothing arms until you press Enter, any E-stop
drops straight to SAFE, no automatic re-arm, and every exit path sets OE HIGH
and turns the PWM off. Servo POWER is hardware: press RESET on the box.

    sudo python3 test_one_servo_armed.py                 # channel 0 on 0x41 (8.4 V wrist board)
    sudo python3 test_one_servo_armed.py 13 0x41         # channel 13 on 0x41
    sudo python3 test_one_servo_armed.py 3 0x40          # channel 3 on 0x40 (5 V finger board)

Check which board is at which address first: sudo i2cdetect -y -r 7
"""
import sys

from adafruit_pca9685 import PCA9685

import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from i2c_bus import open_bus  # noqa: E402  (falls back to Blinka if adafruit_extended_bus is missing)
from power_box import EStop, PowerBox  # noqa: E402

I2C_BUS = 7
HOME = 4900                     # ~1.5 ms at 50 Hz: centre
WIGGLE = (4900, 4500, 5300, 4900)
STEP_S = 1.0

channel = int(sys.argv[1]) if len(sys.argv) > 1 else 0
address = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0x41

pca = PCA9685(open_bus(I2C_BUS), address=address)
pca.frequency = 50


def pwm_off():
    pca.channels[channel].duty_cycle = 0


power = PowerBox(on_safe=pwm_off)
print(f"[WAIT] channel {channel} on board {hex(address)}. Servo signals are OFF.")

try:
    power.arm(lambda: setattr(pca.channels[channel], "duty_cycle", HOME))
    for p in WIGGLE:
        power.check()
        pca.channels[channel].duty_cycle = p
        print("  sent", p)
        power.sleep(STEP_S)
    print("[DONE] Signals off.")
except EStop:
    pass
finally:
    power.close()
