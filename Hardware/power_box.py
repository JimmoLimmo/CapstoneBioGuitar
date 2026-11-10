"""Servo power control for the right-hand electrical box (relays + E-stop).

Every script that moves a servo goes through this module, so the box's
software rules live in one place:

  * Relays start OFF: pins 7 (IN1, K1) and 11 (IN2, K2) are put in the OFF
    state the moment a PowerBox is created. Which pin state means ON is set by
    RELAY_MODE below (run relay_polarity_test.py to find it).
  * Pin 15 reads the E-stop: LOW = released, HIGH = pressed or cable cut.
    Debounced ~50 ms. A watcher thread drops to SAFE the instant it goes HIGH.
  * States: SAFE (E-stop pressed) -> WAIT (released, not armed) -> ARMED
    (operator pressed Enter; home pulses written FIRST, then both relays ON).
    Servo power still only appears when someone presses the physical RESET.
  * No automatic re-arm after an E-stop: the script has to be started again.
  * Any exit -- normal, Ctrl-C, SIGTERM, exception -- turns both relays OFF
    and turns the PWM outputs off.

The hardware (E-stop, thermostat, fuse) is still what actually cuts power.
This module only makes sure the software never fights it.

With dry_run=True nothing is touched and no Jetson libraries are needed.
"""

import atexit
import signal
import sys
import threading
import time

PIN_K1 = 7        # relay IN1: 5 V servo power + master hold (seal-in).
PIN_K2 = 11       # relay IN2: 8.4 V servo power.
PIN_ESTOP = 15    # LOW = released, HIGH = pressed / cable cut (10k pull-up to pin 17)
DEBOUNCE_S = 0.05

# Which Jetson pin state turns a relay ON with the box's current wiring.
# Run `sudo python3 Hardware/relay_polarity_test.py` to find out, then set this.
#   "active_low"  : pin LOW = ON,  pin HIGH = OFF  (original design)
#   "active_high" : pin HIGH = ON, pin LOW = OFF
#   "release_on"  : pin released (input) = ON, pin HIGH = OFF. Used when the
#                   Jetson can't pull the inputs low hard enough. The relays can
#                   then also come on whenever no script holds the pins (boot,
#                   after exit) until a pull-up or transistor is added.
RELAY_MODE = "active_low"
_MODES = ("active_low", "active_high", "release_on")


def _check_mode():
    if RELAY_MODE not in _MODES:
        raise ValueError(f"RELAY_MODE must be one of {_MODES}, not {RELAY_MODE!r}")


def relay_off_level(GPIO):
    """The level to drive a relay pin to for OFF."""
    _check_mode()
    return GPIO.LOW if RELAY_MODE == "active_high" else GPIO.HIGH


def _drive(GPIO, pin, level):
    try:
        GPIO.output(pin, level)
    except Exception:          # pin was released (input): claim it as an output again
        GPIO.setup(pin, GPIO.OUT, initial=level)


def relay_off(GPIO, pin):
    _drive(GPIO, pin, relay_off_level(GPIO))


def relay_on(GPIO, pin):
    _check_mode()
    if RELAY_MODE == "release_on":
        try:
            GPIO.cleanup(pin)
        except Exception:
            pass
        GPIO.setup(pin, GPIO.IN)
    else:
        _drive(GPIO, pin, GPIO.HIGH if RELAY_MODE == "active_high" else GPIO.LOW)


def _resolve_pins(GPIO):
    """Return (k1, k2, estop) channel ids in whatever numbering mode Jetson.GPIO is in.

    Adafruit Blinka (which the PCA9685 driver uses for I2C) imports Jetson.GPIO
    and locks it to TEGRA_SOC mode, and Jetson.GPIO refuses a second setmode().
    So import Blinka first, then name the same header pins in its mode.
    """
    try:
        import board  # Blinka: sets Jetson.GPIO to its own mode on import
    except Exception:
        board = None
    mode = GPIO.getmode()
    if mode is None:
        GPIO.setmode(GPIO.BOARD)
        mode = GPIO.BOARD
    if mode == GPIO.BOARD:
        return PIN_K1, PIN_K2, PIN_ESTOP
    if mode == GPIO.BCM:
        return 4, 17, 22          # header pins 7, 11, 15
    if board is None:
        raise RuntimeError(f"Jetson.GPIO is in mode {mode} and Blinka isn't available to name the pins")
    # Blinka names header pins like a Raspberry Pi: pin 7 = D4, pin 11 = D17, pin 15 = D22.
    return board.D4.id, board.D17.id, board.D22.id


class EStop(Exception):
    """Raised by PowerBox.check() once the E-stop has tripped."""


class PowerBox:
    def __init__(self, dry_run=False, on_safe=None):
        """on_safe: callable that turns every PWM output off (called on SAFE / exit)."""
        self.dry_run = dry_run
        self.on_safe = on_safe
        self.state = "WAIT"
        self.tripped = threading.Event()
        self._closed = False
        self.GPIO = None
        if dry_run:
            return

        import Jetson.GPIO as GPIO
        self.GPIO = GPIO
        GPIO.setwarnings(False)
        self.k1, self.k2, self.estop = _resolve_pins(GPIO)
        off = relay_off_level(GPIO)
        GPIO.setup(self.k1, GPIO.OUT, initial=off)
        GPIO.setup(self.k2, GPIO.OUT, initial=off)
        # Jetson.GPIO cannot enable internal pull-ups on the Orin; this relies on
        # the external 10k to pin 17. If that resistor is missing, pin 15 floats.
        GPIO.setup(self.estop, GPIO.IN)
        print(f"[power] header pins 7 / 11 / 15 = GPIO {self.k1} / {self.k2} / {self.estop}"
              f"  (RELAY_MODE = {RELAY_MODE})")

        atexit.register(self.close)
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, self._on_signal)

        if self._estop_pressed():
            self._go_safe("E-stop is pressed (pin 15 HIGH). Release it and start again.")
        threading.Thread(target=self._watch, daemon=True).start()

    # --- E-stop -----------------------------------------------------------------
    def _estop_pressed(self):
        """True only if pin 15 stays HIGH for the whole debounce window."""
        end = time.monotonic() + DEBOUNCE_S
        while time.monotonic() < end:
            if self.GPIO.input(self.estop) == self.GPIO.LOW:
                return False
            time.sleep(0.005)
        return True

    def _watch(self):
        while not self.tripped.is_set() and not self._closed:
            if self.GPIO.input(self.estop) == self.GPIO.HIGH and self._estop_pressed():
                self._go_safe("E-stop pressed. Relays off, PWM off. Restart the script to re-arm.")
                return
            time.sleep(0.005)

    # --- states -----------------------------------------------------------------
    def relays_off(self):
        if self.GPIO is not None:
            relay_off(self.GPIO, self.k1)
            relay_off(self.GPIO, self.k2)

    def _pwm_off(self):
        if self.on_safe:
            try:
                self.on_safe()
            except Exception:
                pass

    def _go_safe(self, reason):
        self.relays_off()
        self._pwm_off()
        self.state = "SAFE"
        self.tripped.set()
        print(f"\n[power] SAFE: {reason}", flush=True)

    @property
    def closed(self):
        return self._closed

    def check(self):
        """Call in every control loop. Raises EStop once the E-stop has tripped
        or the box has been shut down (Ctrl-C, SIGTERM)."""
        if self.tripped.is_set() or self._closed:
            raise EStop()

    def sleep(self, seconds):
        """time.sleep that wakes up and raises EStop if the E-stop trips."""
        if self.tripped.wait(seconds):
            raise EStop()

    def arm(self, write_home):
        """WAIT -> ARMED. write_home() must put every servo channel at its safe
        home pulse; it runs BEFORE the relays are allowed to close."""
        self.check()
        if self.dry_run:
            self.state = "ARMED"
            print("[power] (dry run) would arm here.")
            return
        print("[power] WAIT: relays off. Strings OK? Hands clear? E-stop within reach?")
        input("[power] Press Enter to ARM (Ctrl-C to quit)... ")
        self.check()
        write_home()
        relay_on(self.GPIO, self.k1)
        relay_on(self.GPIO, self.k2)
        self.state = "ARMED"
        print("[power] ARMED. Press the physical RESET button on the box; the relays should click.")
        input("[power] Press Enter here once servo power is on... ")
        self.check()

    def close(self):
        """Relays off, PWM off, release the pins. Safe to call more than once."""
        if self._closed:
            return
        self._closed = True
        self.relays_off()
        self._pwm_off()
        if self.GPIO is not None:
            # cleanup() only for the E-stop input; the relay pins are left
            # driven to OFF rather than released to float.
            try:
                self.GPIO.cleanup(self.estop)
            except Exception:
                pass

    def _on_signal(self, signum, _frame):
        self.close()
        sys.exit(128 + signum)
