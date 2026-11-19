"""Watch the E-stop sense pin (header pin 15) live. Nothing is driven.

    servo estop_check.py

Press and release the E-stop a few times, then Ctrl-C for a summary.
Expected with REV 3 wiring: LOW released, HIGH pressed.
"""
import glob
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import power_box  # noqa: E402

import Jetson.GPIO as GPIO

PIN_NAMES = ("pn.01", "pn1", "gp88")


# Print the kernel's pinmux / gpio lines for pin 15 (needs sudo for debugfs).
def show_pinmux():
    files = glob.glob("/sys/kernel/debug/pinctrl/*/pinconf-pins")
    files += glob.glob("/sys/kernel/debug/pinctrl/*/pinmux-pins")
    files += ["/sys/kernel/debug/gpio"]
    found = False
    for path in files:
        try:
            with open(path) as f:
                for line in f:
                    if any(n in line.lower() for n in PIN_NAMES):
                        print(f"  {os.path.basename(path)}: {line.strip()}")
                        found = True
        except OSError:
            pass
    if not found:
        print("  (couldn't read debugfs; run with sudo / the servo alias to see the pin config)")


GPIO.setwarnings(False)
pins = power_box.resolve_pins(GPIO)
pin = pins["estop"]
GPIO.setup(pin, GPIO.IN)
pressed = GPIO.LOW if power_box.ESTOP_PRESSED_LEVEL == "low" else GPIO.HIGH

print(f"[estop] GPIO mode {GPIO.getmode()}, header pin 15 = GPIO {pin!r}")
print(f"[estop] pressed level in power_box.py: {power_box.ESTOP_PRESSED_LEVEL.upper()}")
print("[estop] pin config:")
show_pinmux()
print("[estop] watching pin 15. Press / release the E-stop. Ctrl-C to stop.\n")

seen = {GPIO.LOW: 0, GPIO.HIGH: 0}
changes = 0
last = None
start = time.monotonic()
try:
    while True:
        level = GPIO.input(pin)
        seen[level] += 1
        if level != last:
            state = "PRESSED" if level == pressed else "released"
            name = "HIGH" if level == GPIO.HIGH else "LOW"
            print(f"  {time.monotonic() - start:7.2f} s  pin 15 = {name:4s} script sees: {state}")
            if last is not None:
                changes += 1
            last = level
        time.sleep(0.005)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup(pin)

print(f"\n[estop] {changes} changes. LOW reads {seen[GPIO.LOW]}, HIGH reads {seen[GPIO.HIGH]}.")
if changes == 0:
    stuck = "HIGH" if last == GPIO.HIGH else "LOW"
    print(f"[estop] pin 15 never changed (stuck {stuck}). Measure pin 15 to pin 14 (GND) with a meter")
    print("        while pressing: 0 V released, 3.3 V pressed.")
    print("        Meter changes but this doesn't: pin 15 isn't set up as a GPIO input (pinmux / Jetson-IO).")
    print("        Meter doesn't change either: wiring. Check R4 to pin 17, NC2 to pin 14,")
    print("        and that the NC2 contact block is clipped onto the E-stop.")
elif changes > 20:
    print("[estop] lots of changes: pin 15 may be floating (R4 pull-up missing or loose).")
