"""Servo safety for the right-hand electrical box (REV 3 wiring sheets 0-5).

REV 3: the relays are NOT driven by the Jetson. IN1/IN2 are tied to GND and
the relay coils are fed only through the hardware loop (RESET -> point A ->
F3 -> TH1 -> E-stop NC1 -> point B -> JD-VCC, D1 seal-in from K1 NO). So servo
POWER is switched entirely in hardware: RESET turns it on, the E-stop / TH1 /
F3 / MAIN turn it off, and nothing restarts by itself.

The Jetson's jobs, all in this module so every script follows the same rules:

  * Pin 31 (GPIO11) drives OE on both PCA9685 boards. OE HIGH = every servo
    signal off, LOW = signals on. R5 (10k to 3.3 V) holds it HIGH whenever
    the pin isn't driven (boot, crash, Jetson off). Starts HIGH here.
  * Pin 15 reads the E-stop's second contact NC2 (R4 10k pull-up to 3.3 V,
    NC2 to GND): LOW = released, HIGH = pressed or cable cut. Debounced
    ~50 ms. A watcher thread drops to SAFE the instant it reads pressed.
  * States: SAFE (E-stop pressed) -> WAIT (released, OE HIGH) -> ARMED
    (operator pressed Enter; home pulses written FIRST, then OE LOW).
  * No automatic re-arm after an E-stop: the script has to be started again.
  * Any exit -- normal, Ctrl-C, SIGTERM, exception -- sets OE HIGH and turns
    the PWM outputs off.

The hardware (E-stop NC1, TH1, F3) is what actually cuts servo power. This
module only makes sure the software never fights it.

RELAY_PINS_WIRED is only for the alternative "Jetson-switched relay" rewire
(2N7000 drivers on IN1/IN2); leave it False for REV 3.

With dry_run=True nothing is touched and no Jetson libraries are needed.
"""

import atexit
import os
import signal
import sys
import threading
import time

PIN_OE = 31       # OE of both PCA9685 (GPIO11). HIGH = outputs off. R5 pull-up to 3.3 V.
PIN_ESTOP = 15    # E-stop NC2 sense. R4 10k pull-up to 3.3 V (pin 17).
PIN_K1 = 7        # only used if RELAY_PINS_WIRED (alternative rewire)
PIN_K2 = 11       # only used if RELAY_PINS_WIRED (alternative rewire)
DEBOUNCE_S = 0.05

# Pin 15 level that means "E-stop pressed".
#   "high" : REV 3 (NC2 to GND, 10k pull-up: closed = LOW = released)
#   "low"  : alternative rewire (divider from the latch loop)
ESTOP_PRESSED_LEVEL = "high"

# Alternative rewire only: Jetson switches IN1/IN2 through 2N7000 drivers.
RELAY_PINS_WIRED = False
#   "active_low"  : pin LOW = ON,  pin HIGH = OFF
#   "active_high" : pin HIGH = ON, pin LOW = OFF (2N7000 drivers)
#   "release_on"  : pin released (input) = ON, pin HIGH = OFF
RELAY_MODE = "active_high"
_MODES = ("active_low", "active_high", "release_on")


# --- relay helpers (alternative rewire only) -------------------------------------
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


# --- pins ------------------------------------------------------------------------
def resolve_pins(GPIO):
    """Return a dict of channel ids {oe, estop, k1, k2} in whatever numbering
    mode Jetson.GPIO is in.

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
        return {"oe": PIN_OE, "estop": PIN_ESTOP, "k1": PIN_K1, "k2": PIN_K2}
    if mode == GPIO.BCM:
        # header pins 31, 15, 7, 11
        return {"oe": 6, "estop": 22, "k1": 4, "k2": 17}
    if board is None:
        raise RuntimeError(f"Jetson.GPIO is in mode {mode} and Blinka isn't available to name the pins")
    # Blinka names header pins like a Raspberry Pi: 31 = D6, 15 = D22, 7 = D4, 11 = D17.
    return {"oe": board.D6.id, "estop": board.D22.id, "k1": board.D4.id, "k2": board.D17.id}


def _resolve_pins(GPIO):
    """Old 3-tuple form (k1, k2, estop), kept for relay_polarity_test.py."""
    p = resolve_pins(GPIO)
    return p["k1"], p["k2"], p["estop"]


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
        pins = resolve_pins(GPIO)
        self.oe, self.estop = pins["oe"], pins["estop"]
        self.k1, self.k2 = pins["k1"], pins["k2"]
        # Signals off before anything else.
        GPIO.setup(self.oe, GPIO.OUT, initial=GPIO.HIGH)
        if RELAY_PINS_WIRED:
            off = relay_off_level(GPIO)
            GPIO.setup(self.k1, GPIO.OUT, initial=off)
            GPIO.setup(self.k2, GPIO.OUT, initial=off)
        # Jetson.GPIO cannot enable internal pulls on the Orin; pin 15 relies on
        # the external 10k pull-up (R4). If it is missing, pin 15 floats.
        GPIO.setup(self.estop, GPIO.IN)
        print(f"[power] header pins 31 (OE) / 15 (E-stop) = GPIO {self.oe} / {self.estop}"
              + (f"; relays on pins 7 / 11 ({RELAY_MODE})" if RELAY_PINS_WIRED else ""))

        atexit.register(self.close)
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, self._on_signal)

        if self._estop_pressed():
            self._go_safe("E-stop is pressed (or its cable is cut). Release it and start again.")
        threading.Thread(target=self._watch, daemon=True).start()

    # --- E-stop -----------------------------------------------------------------
    def _pressed_level(self):
        if ESTOP_PRESSED_LEVEL not in ("low", "high"):
            raise ValueError(f"ESTOP_PRESSED_LEVEL must be 'low' or 'high', not {ESTOP_PRESSED_LEVEL!r}")
        return self.GPIO.LOW if ESTOP_PRESSED_LEVEL == "low" else self.GPIO.HIGH

    def _estop_pressed(self):
        """True only if pin 15 stays at the pressed level for the whole debounce window."""
        end = time.monotonic() + DEBOUNCE_S
        while time.monotonic() < end:
            if self.GPIO.input(self.estop) != self._pressed_level():
                return False
            time.sleep(0.005)
        return True

    def _watch(self):
        while not self.tripped.is_set() and not self._closed:
            if self.GPIO.input(self.estop) == self._pressed_level() and self._estop_pressed():
                self._go_safe("E-stop pressed. Signals off (OE HIGH), PWM off. Restart the script to re-arm.")
                return
            time.sleep(0.005)

    # --- states -----------------------------------------------------------------
    def outputs_off(self):
        """OE HIGH (all servo signals off) and, if wired, relays off."""
        if self.GPIO is None:
            return
        try:
            self.GPIO.output(self.oe, self.GPIO.HIGH)
        except Exception:
            self.GPIO.setup(self.oe, self.GPIO.OUT, initial=self.GPIO.HIGH)
        if RELAY_PINS_WIRED:
            relay_off(self.GPIO, self.k1)
            relay_off(self.GPIO, self.k2)

    relays_off = outputs_off   # old name

    def _pwm_off(self):
        if self.on_safe:
            try:
                self.on_safe()
            except Exception:
                pass

    def _go_safe(self, reason):
        self.outputs_off()
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
        home pulse; it runs BEFORE OE goes LOW (signals on)."""
        self.check()
        if self.dry_run:
            self.state = "ARMED"
            print("[power] (dry run) would arm here.")
            return
        print("[power] WAIT: servo signals off. Strings OK? Hands clear? E-stop within reach?")
        print("[power] Servo power is hardware: press RESET on the box if the SERVO LED is off.")
        auto = os.environ.get("POWERBOX_AUTO_ARM")
        if auto:
            # No keyboard needed: count down, E-stop or Ctrl-C still stops it.
            secs = max(3, int(float(auto)))
            for n in range(secs, 0, -1):
                print(f"[power] Arming in {n} s (E-stop or Ctrl-C to cancel)...", flush=True)
                self.sleep(1.0)
        else:
            input("[power] Press Enter to ARM (Ctrl-C to quit)... ")
        self.check()
        write_home()
        self.GPIO.output(self.oe, self.GPIO.LOW)
        if RELAY_PINS_WIRED:
            relay_on(self.GPIO, self.k1)
            relay_on(self.GPIO, self.k2)
        self.state = "ARMED"
        print("[power] ARMED: servo signals on (OE LOW).")
        if RELAY_PINS_WIRED:
            input("[power] Press RESET if needed, then Enter here once servo power is on... ")
            self.check()

    def close(self):
        """Signals off, PWM off, release the E-stop pin. Safe to call more than once."""
        if self._closed:
            return
        self._closed = True
        self.outputs_off()
        self._pwm_off()
        if self.GPIO is not None:
            # cleanup() only for the E-stop input; OE is left driven HIGH
            # (R5 also holds it HIGH once the process is gone).
            try:
                self.GPIO.cleanup(self.estop)
            except Exception:
                pass

    def _on_signal(self, signum, _frame):
        self.close()
        sys.exit(128 + signum)
