"""Find out which Jetson pin state turns the relays on with the current wiring.

Tries three states on header pins 7 and 11 (IN1 / IN2) and asks you to press
RESET after each one. At the end it tells you which RELAY_MODE to set in
power_box.py.

    sudo python3 Hardware/relay_polarity_test.py

Before running: unplug the servo connectors (or at least keep hands clear of
the hand), MAIN switch on, E-stop released. No PWM is sent, so servos should
not move, but servo power WILL come on during this test.
"""

import sys
import time

from power_box import _resolve_pins

import Jetson.GPIO as GPIO

GPIO.setwarnings(False)
k1, k2, estop = _resolve_pins(GPIO)
PINS = (k1, k2)


def drive(level):
    for p in PINS:
        try:
            GPIO.output(p, level)
        except Exception:
            GPIO.setup(p, GPIO.OUT, initial=level)


def release():
    for p in PINS:
        try:
            GPIO.cleanup(p)
        except Exception:
            pass
        GPIO.setup(p, GPIO.IN)


def ask(q):
    while True:
        a = input(q + " [y/n] ").strip().lower()
        if a in ("y", "n"):
            return a == "y"


STATES = [
    ("HIGH", lambda: drive(GPIO.HIGH)),
    ("LOW", lambda: drive(GPIO.LOW)),
    ("RELEASED (input, not driven)", release),
]

print("Relay polarity test. Servo connectors unplugged or hands clear?")
input("Press Enter to start (Ctrl-C to quit)... ")

results = {}
try:
    for name, apply in STATES:
        print(f"\n--- pins 7 and 11 now {name} ---")
        apply()
        time.sleep(0.2)
        already = ask("Are the relay LEDs ALREADY on (before pressing RESET)?")
        if already:
            results[name] = True
        else:
            input("Press and release RESET, then press Enter... ")
            results[name] = ask("Did the relays click / LEDs come on and STAY on?")
        if results[name]:
            input("Now press the E-stop to drop it, release the E-stop, then press Enter... ")
finally:
    # We don't know the safe state yet, so let go of the pins.
    for p in PINS:
        try:
            GPIO.cleanup(p)
        except Exception:
            pass

print("\nResults:")
for name, on in results.items():
    print(f"  {name:30s} -> {'relay ON' if on else 'no'}")

hi, lo, rel = results.get("HIGH"), results.get("LOW"), results.get("RELEASED (input, not driven)")
print()
if lo and not hi:
    print('Set RELAY_MODE = "active_low" in power_box.py (the original design).')
elif hi and not lo:
    print('Set RELAY_MODE = "active_high" in power_box.py.')
elif rel and not hi and not lo:
    print('Set RELAY_MODE = "release_on" in power_box.py.')
    print("The Jetson can't pull the inputs low hard enough. This works, but the relays")
    print("can also turn on whenever no script holds the pins (boot, after exit).")
    print("Add a pull-up or a transistor to make that safe.")
elif hi and lo:
    print("The relays came on in both HIGH and LOW: the Jetson pins aren't controlling")
    print("them at all. Check where IN1/IN2 and VCC on the relay board are wired.")
else:
    print("The relays never came on: check the RESET/E-stop loop and relay board power.")
sys.exit(0)
