"""Open a Linux I2C bus by number (bus 7 = header pins 3/5 on the Orin Nano).

Uses adafruit_extended_bus when it's installed. If it isn't (offline Jetson),
builds the same thing from Adafruit Blinka, which adafruit_pca9685 needs anyway.
"""
import os
import threading


def open_bus(bus_id, frequency=400000):
    try:
        from adafruit_extended_bus import ExtendedI2C
        return ExtendedI2C(bus_id, frequency=frequency)
    except ImportError:
        pass

    if not os.path.exists(f"/dev/i2c-{bus_id}"):
        raise SystemExit(f"[i2c] /dev/i2c-{bus_id} does not exist. Check: ls /dev/i2c-*")
    try:
        import busio
        from adafruit_blinka.microcontroller.generic_linux.i2c import I2C as LinuxI2C
    except ImportError as e:
        raise SystemExit(f"[i2c] neither adafruit_extended_bus nor Adafruit Blinka is installed ({e}).\n"
                         "      Copy adafruit_extended_bus.py over from last year's Jetson, or install "
                         "adafruit-blinka.")

    class _BusI2C(busio.I2C):
        """Same as adafruit_extended_bus.ExtendedI2C: a busio.I2C on /dev/i2c-<bus_id>."""

        def __init__(self, bus, freq):  # skip busio's board-pin lookup
            self._i2c = LinuxI2C(bus, mode=LinuxI2C.MASTER, baudrate=freq)
            self._lock = threading.RLock()

    return _BusI2C(bus_id, frequency)
