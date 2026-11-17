"""Bridge between the PyBullet sim and the real hand's wrist servos.

Any script that has a (dev_deg, flex_deg) wrist pose per tick can feed it to
the hardware through a "sink". Two sinks:

    HardwareSink  -- drives the PCA9685 directly (use when the sim runs on the Jetson)
    UdpSink       -- sends poses over the network to a listener on the Jetson

Same-machine use (add three lines to mvmnt_demo.py / play_learned_strum.py):

    from hand_link import HardwareSink
    hw = HardwareSink(dry_run=True)              # flip to False for real servos
    hw.start(wrist_at(schedule, -1.0)[:2])       # ease in from neutral
    ...inside the loop:   hw.send(now, yaw_deg, flex_deg)
    ...in a finally:      hw.close()

Sim on a laptop, hand on the Jetson:

    laptop:  sink = UdpSink("<jetson-ip>")  ->  sink.send(now, yaw, flex)
    jetson:  python Hardware/hand_link.py --listen [--dry-run]

The listener parks the wrist at neutral if packets stop for WATCHDOG_SECONDS.
All motion goes through servo_output.write_wrist(), so calibration, per-servo
limits and the wrist kinematics are the same as running servo_output.py.
"""

import argparse
import json
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

DEFAULT_PORT = 9871
SEND_HZ = 50            # one update per PWM frame
WATCHDOG_SECONDS = 0.5  # listener parks the wrist if the stream goes quiet


class HardwareSink:
    """Writes wrist poses to the servos at SEND_HZ, whatever rate it's called at."""

    def __init__(self, dry_run=False, hz=SEND_HZ):
        import servo_output as so  # imported here so UdpSink users need no hardware libs
        from power_box import PowerBox

        self.so = so
        self.dry_run = dry_run
        problems = so.missing_calibration(wrist_only=True)
        self.active = not problems
        if problems:
            if not dry_run:
                raise RuntimeError("wrist not calibrated: " + "; ".join(problems))
            print("[link] wrist calibration not filled in -- dry run prints spool angles, not pulses.")
        # only the wrist's board, so the finger board doesn't have to be powered
        boards = tuple(sorted({c["board"] for c in so.WRIST_SERVOS.values()}))
        self.bus = so.ServoBus(dry_run=dry_run, boards=boards)
        # OE (pin 31) stays HIGH until arm(), and the E-stop is watched from here on
        self.power = PowerBox(dry_run=dry_run, on_safe=self.bus.release)
        self.period = 1.0 / hz
        self.next_t = 0.0
        self.next_print = 0.0
        self.pose = (0.0, 0.0)
        self.started = False
        self.stopped = False

    # Waits for Enter (or POWERBOX_AUTO_ARM), writes neutral, then turns the servo signals on.
    def arm(self):
        if self.power.state == "ARMED":
            return

        def write_home():
            if self.active:
                self.so.write_wrist(self.bus, 0.0, 0.0)

        self.power.arm(write_home)

    def live(self):
        return self.power.state == "ARMED" and not self.power.tripped.is_set() and not self.power.closed

    def start(self, ready=(0.0, 0.0)):
        """Ease from neutral to the first pose so the first command is never a jump."""
        self.arm()
        if self.active:
            self.so.ease_wrist(self.bus, (0.0, 0.0), ready, power=self.power)
        self.pose = tuple(ready)
        self.started = True

    def send(self, t, dev_deg, flex_deg):
        if self.stopped:
            return
        if not self.live():
            # E-stop or Ctrl-C: the box already turned the signals off, the sim can keep going
            self.stopped = True
            print("[link] hardware stopped (E-stop or shutdown). Restart the script to re-arm.")
            return
        if not self.started:
            self.start((dev_deg, flex_deg))
        if t < self.next_t:
            return
        self.next_t = t + self.period
        if self.active:
            pulses = self.so.write_wrist(self.bus, dev_deg, flex_deg)
        self.pose = (dev_deg, flex_deg)
        if self.dry_run and t >= self.next_print:
            self.next_print = t + 1.0
            if self.active:
                detail = " ".join(f"{n}={p}" for n, p in pulses.items())
            else:
                detail = " ".join(f"{n}={a:+5.1f}d" for n, a in self.so.wk.servo_angles(dev_deg, flex_deg).items())
            print(f"[link] t={t:6.2f}s dev={dev_deg:6.1f} flex={flex_deg:5.1f} | {detail}")

    def park(self):
        """Ease back to neutral and keep holding."""
        if self.active and self.started and self.live():
            try:
                self.so.ease_wrist(self.bus, self.pose, (0.0, 0.0), power=self.power)
            except Exception as error:  # an E-stop mid-park is fine, the box is already safe
                print(f"[link] could not park the wrist ({error.__class__.__name__})")
        self.pose = (0.0, 0.0)
        self.started = False

    def close(self):
        try:
            self.park()
        finally:
            self.power.close()
            self.bus.release()


class UdpSink:
    """Same send()/close() interface, but ships poses to a listener over UDP."""

    def __init__(self, host, port=DEFAULT_PORT, hz=SEND_HZ):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.period = 1.0 / hz
        self.next_t = 0.0

    def start(self, ready=(0.0, 0.0)):
        pass  # the listener eases in on the first packet

    def send(self, t, dev_deg, flex_deg):
        if t < self.next_t:
            return
        self.next_t = t + self.period
        self.sock.sendto(json.dumps({"t": t, "dev": dev_deg, "flex": flex_deg}).encode(), self.addr)

    def close(self):
        self.sock.close()


def listen(port, dry_run):
    """Run on the Jetson: turn incoming poses into servo commands."""
    sink = HardwareSink(dry_run=dry_run)
    sink.arm()  # arm before listening, so the Enter prompt doesn't wait on the first packet
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.1)
    last_packet = time.perf_counter()
    print(f"[link] listening on UDP {port}{' [DRY RUN]' if dry_run else ''}. Ctrl-C to stop.")
    try:
        while True:
            try:
                data, _ = sock.recvfrom(1024)
                msg = json.loads(data)
                last_packet = time.perf_counter()
                # Receiver sets its own pacing; the sender's t is only used for throttling.
                sink.send(msg["t"], float(msg["dev"]), float(msg["flex"]))
            except socket.timeout:
                if sink.started and time.perf_counter() - last_packet > WATCHDOG_SECONDS:
                    print("[link] stream stopped -- parking wrist at neutral.")
                    sink.park()
                    sink.next_t = 0.0  # the sender's clock restarts with its next run
            except (ValueError, KeyError):
                print("[link] ignored a malformed packet.")
    except KeyboardInterrupt:
        print("\n[link] interrupted.")
    finally:
        sink.close()
        sock.close()
        print("[link] stopped -- wrist parked and released.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--listen", action="store_true", help="receive poses over UDP and drive the servos")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--dry-run", action="store_true", help="no I2C; nothing moves")
    args = parser.parse_args()
    if args.listen:
        listen(args.port, args.dry_run)
    else:
        print(__doc__)


if __name__ == "__main__":
    main()