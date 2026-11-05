"""Servo power control for the right-hand electrical box (relays + E-stop).

Every script that moves a servo goes through this module, so the box's
software rules live in one place:

  * Relays start OFF: pins 7 (IN1, K1) and 11 (IN2, K2) are driven HIGH the
    moment a PowerBox is created. Both are ACTIVE LOW.
  * Pin 15 reads the E-stop: LOW = released, HIGH = pressed or cable cut.
    Debounced ~50 ms. A watcher thread drops to SAFE the instant it goes HIGH.
  * States: SAFE (E-stop pressed) -> WAIT (released, not armed) -> ARMED
    (operator pressed Enter; home pulses written FIRST, then IN1 = IN2 = LOW).
    Servo power still only appears when someone presses the physical RESET.
  * No automatic re-arm after an E-stop: the script has to be started again.
  * Any exit -- normal, Ctrl-C, SIGTERM, exception -- sets IN1 = IN2 = HIGH
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

PIN_K1 = 7        # relay IN1: 5 V servo power + master hold (seal-in). ACTIVE LOW.
PIN_K2 = 11       # relay IN2: 8.4 V servo power. ACTIVE LOW.
PIN_ESTOP = 15    # LOW = released, HIGH = pressed / cable cut (10k pull-up to pin 17)
DEBOUNCE_S = 0.05


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
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(PIN_K1, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(PIN_K2, GPIO.OUT, initial=GPIO.HIGH)
        # Jetson.GPIO cannot enable internal pull-ups on the Orin; this relies on
        # the external 10k to pin 17. If that resistor is missing, pin 15 floats.
        GPIO.setup(PIN_ESTOP, GPIO.IN)

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
            if self.GPIO.input(PIN_ESTOP) == self.GPIO.LOW:
                return False
            time.sleep(0.005)
        return True

    def _watch(self):
        while not self.tripped.is_set() and not self._closed:
            if self.GPIO.input(PIN_ESTOP) == self.GPIO.HIGH and self._estop_pressed():
                self._go_safe("E-stop pressed. Relays off, PWM off. Restart the script to re-arm.")
                return
            time.sleep(0.005)

    # --- states -----------------------------------------------------------------
    def relays_off(self):
        if self.GPIO is not None:
            self.GPIO.output(PIN_K1, self.GPIO.HIGH)
            self.GPIO.output(PIN_K2, self.GPIO.HIGH)

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
        self.GPIO.output(PIN_K1, self.GPIO.LOW)
        self.GPIO.output(PIN_K2, self.GPIO.LOW)
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
            # cleanup() only for the E-stop input; the relay pins are left as
            # driven-HIGH outputs rather than released to float.
            try:
                self.GPIO.cleanup(PIN_ESTOP)
            except Exception:
                pass

    def _on_signal(self, signum, _frame):
        self.close()
        sys.exit(128 + signum)
