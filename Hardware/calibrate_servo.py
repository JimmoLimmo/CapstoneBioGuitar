"""Jog one PCA9685 channel by hand to find its real pulse bounds.

The hardware equivalent of Sim/mvmnt_demo.py's CALIBRATE mode: instead of
guessing the deg_lo/deg_hi -> pulse_lo/pulse_hi numbers in
servo_output.py's CHANNEL_MAP, move the actual servo to two positions you
care about (e.g. wrist_UDEV all the way to its down-strum limit, then all
the way to its up-strum limit) and read off the duty_cycle value that put it
there.

Run from the repo root, on the Jetson with the PCA9685 wired up:
    python Hardware/calibrate_servo.py 0

Type a pulse value (1638-8191) and Enter to move channel 0 there; Ctrl+C to
quit. Start near the middle (~4900) and nudge in from there -- don't jump
straight to the extremes on an untested channel.
"""

import sys

from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_pca9685 import PCA9685

PULSE_ABS_MIN = 1638
PULSE_ABS_MAX = 8191
I2C_BUS = 7
I2C_ADDRESS = 0x40


def main():
    if len(sys.argv) != 2 or not sys.argv[1].isdigit():
        print(f"usage: python {sys.argv[0]} <channel 0-15>")
        return

    channel = int(sys.argv[1])
    i2c = I2C(I2C_BUS)
    pca = PCA9685(i2c, address=I2C_ADDRESS)
    pca.frequency = 50

    print(f"[calibrate] channel {channel}. Enter a pulse ({PULSE_ABS_MIN}-{PULSE_ABS_MAX}), or 'q' to quit.")
    print("[calibrate] start near the middle and nudge in -- don't jump straight to the extremes.")
    while True:
        raw = input(f"channel {channel} pulse> ").strip()
        if raw.lower() in ("q", "quit", "exit"):
            break
        try:
            pulse = int(raw)
        except ValueError:
            print("  not a number")
            continue
        if not (PULSE_ABS_MIN <= pulse <= PULSE_ABS_MAX):
            print(f"  out of the safe range ({PULSE_ABS_MIN}-{PULSE_ABS_MAX}), ignored")
            continue
        pca.channels[channel].duty_cycle = pulse
        print(f"  -> sent {pulse}. Note this down once the servo is where you want it.")


if __name__ == "__main__":
    main()
