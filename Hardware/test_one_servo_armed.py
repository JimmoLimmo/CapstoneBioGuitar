"""Wiggle ONE servo through the new relay/E-stop box.

Follows the box's software rules: relays start OFF (see RELAY_MODE in power_box.py),
nothing arms until you press Enter, any E-stop drops straight to SAFE,
no automatic re-arm, and every exit path turns both relays OFF.

    sudo python3 test_one_servo_armed.py                 # channel 0 on 0x42 (8.4 V board)
    sudo python3 test_one_servo_armed.py 13 0x42         # channel 13 on 0x42
    sudo python3 test_one_servo_armed.py 3 0x41          # channel 3 on the 5 V board
"""
import atexit, signal, sys, threading, time

import Jetson.GPIO as GPIO
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_pca9685 import PCA9685
from power_box import relay_on, relay_off, relay_off_level

# --- pins (physical BOARD numbering) -----------------------------------------
PIN_K1 = 7      # relay IN1, 5 V servo power, master hold. Polarity: RELAY_MODE in power_box.py
PIN_K2 = 11     # relay IN2, 8.4 V servo power.
PIN_ESTOP = 15  # LOW = E-stop released, HIGH = pressed or cable cut
I2C_BUS = 7

HOME = 4900                     # ~1.5 ms at 50 Hz: centre
WIGGLE = (4900, 4500, 5300, 4900)
STEP_S = 1.0

channel = int(sys.argv[1]) if len(sys.argv) > 1 else 0
address = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0x42

# --- relays OFF before anything else -----------------------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN_K1, GPIO.OUT, initial=relay_off_level(GPIO))
GPIO.setup(PIN_K2, GPIO.OUT, initial=relay_off_level(GPIO))
GPIO.setup(PIN_ESTOP, GPIO.IN)   # Jetson.GPIO has no internal pull-ups; relies on the 10k to pin 17

pca = None
tripped = threading.Event()


def relays_off():
    relay_off(GPIO, PIN_K1)
    relay_off(GPIO, PIN_K2)


def pwm_off():
    if pca is not None:
        try:
            pca.channels[channel].duty_cycle = 0
        except Exception:
            pass


def safe(reason=""):
    relays_off()
    pwm_off()
    if reason:
        print(f"\n[SAFE] {reason}")


def cleanup():
    safe()
    GPIO.cleanup()


atexit.register(cleanup)
for sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(sig, lambda *_: sys.exit(1))


def estop_pressed():
    """Debounced read: HIGH for ~50 ms counts as pressed."""
    for _ in range(5):
        if GPIO.input(PIN_ESTOP) == GPIO.LOW:
            return False
        time.sleep(0.01)
    return True


def watch_estop():
    while not tripped.is_set():
        if estop_pressed():
            tripped.set()
            safe("E-stop pressed (pin 15 HIGH). Relays off, PWM off. Restart the script to try again.")
            return
        time.sleep(0.01)


def wait(seconds):
    end = time.time() + seconds
    while time.time() < end:
        if tripped.is_set():
            sys.exit(1)
        time.sleep(0.01)


# --- run ---------------------------------------------------------------------
pca = PCA9685(I2C(I2C_BUS), address=address)
pca.frequency = 50
print(f"[WAIT] channel {channel} on board {hex(address)}. Relays are OFF.")

if estop_pressed():
    print("[SAFE] E-stop is pressed (pin 15 HIGH). Release it and run again.")
    sys.exit(1)

threading.Thread(target=watch_estop, daemon=True).start()
input("[WAIT] Strings slack? Hands clear? Press Enter to ARM (Ctrl-C to quit)... ")
if tripped.is_set():
    sys.exit(1)

pca.channels[channel].duty_cycle = HOME          # home pulse first
relay_on(GPIO, PIN_K1)                            # then both relays allowed
relay_on(GPIO, PIN_K2)
print("[ARMED] Now press the physical RESET button on the box (servo power only comes on then).")
input("[ARMED] Press Enter here once you've pressed RESET and heard the relays click... ")

for p in WIGGLE:
    pca.channels[channel].duty_cycle = p
    print("  sent", p)
    wait(STEP_S)

print("[DONE] Turning relays off.")
