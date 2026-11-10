"""Drives the whole right hand on the Jetson (wrist + fingers) through power_box.py.

    with Hand(sim_ip="192.168.1.20") as hand:
        hand.arm()
        hand.move(fingers=finger_motion.PICK_GRIP, wrist=(25, 0))
"""

import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "Sim"))
sys.path.insert(0, HERE)

import finger_motion as fm  # noqa: E402
import fingers as fh  # noqa: E402
import servo_output as so  # noqa: E402
from motion_link import DEFAULT_PORT, PoseSender  # noqa: E402
from power_box import EStop, PowerBox  # noqa: E402
from strum_motion import smooth, wrist_at  # noqa: E402

CONTROL_HZ = so.CONTROL_HZ
EASE_SECONDS = so.EASE_SECONDS
# software limits for typed wrist commands (deg)
WRIST_DEV_LIMITS = (-30.0, 30.0)
WRIST_FLEX_LIMITS = (-25.0, 25.0)

__all__ = ["Hand", "EStop", "CONTROL_HZ"]


def _wrist_problems():
    problems = []
    for name, cfg in so.WRIST_SERVOS.items():
        if cfg.get("board") not in so.BOARDS:
            problems.append(f"wrist {name}: board must be one of {sorted(so.BOARDS)}")
        if cfg["channel"] is None:
            problems.append(f"wrist {name}: no channel")
        if cfg["neutral_pulse"] is None:
            problems.append(f"wrist {name}: no neutral_pulse")
    return problems


class Hand:
    def __init__(self, dry_run=False, wrist=True, fingers=fm.FINGERS, sim_ip=None,
                 sim_port=DEFAULT_PORT, realtime=None):
        self.dry_run = dry_run
        self.use_wrist = wrist
        self.fingers = tuple(fingers)
        fh.servos_for(self.fingers)   # raises on a typo'd finger name

        problems = self.calibration_problems()
        if problems and not dry_run:
            raise SystemExit("[hand] not calibrated yet -- refusing to move anything:\n"
                             + "\n".join(f"          {p}" for p in problems)
                             + "\n[hand]   fill these in, drive fewer parts (--no-wrist / --fingers index), or use --dry-run.")
        if problems:
            print(f"[hand] (dry run) {len(problems)} calibration value(s) missing; "
                  "those servos are skipped, the sim still gets every pose.")
        self.wrist_live = wrist and so.is_calibrated()

        boards = set()
        if wrist:
            boards |= {c["board"] for c in so.WRIST_SERVOS.values()}
        if self.fingers:
            boards |= {fh.FINGER_SERVOS[n]["board"] for n in fh.servos_for(self.fingers)}
        if not boards:
            raise SystemExit("[hand] nothing to drive: no wrist and no fingers selected.")
        self._check_shared_address(boards)

        self.bus = so.ServoBus(dry_run=dry_run, boards=tuple(sorted(boards)))
        self.power = PowerBox(dry_run=dry_run, on_safe=self.bus.release)
        if sim_ip:
            try:
                self.bus.sim = PoseSender(sim_ip, sim_port)
                print(f"[hand] streaming poses to the sim at {self.bus.sim.addr[0]}:{sim_port}")
            except OSError as e:
                print(f"[hand] can't reach sim address {sim_ip} ({e}); continuing without the sim.")
        # a dry run only waits when a sim is watching
        self.realtime = (not dry_run or self.bus.sim is not None) if realtime is None else realtime

        self.wrist_pose = (0.0, 0.0)
        self.finger_pose = dict(fm.OPEN)

    # --- setup checks --------------------------------------------------------
    def calibration_problems(self):
        problems = _wrist_problems() if self.use_wrist else []
        problems += fh.missing_calibration(self.fingers)
        used = []
        if self.use_wrist:
            used += [(c["board"], c["channel"]) for c in so.WRIST_SERVOS.values() if c["channel"] is not None]
        used += fh.channels(self.fingers)
        dupes = sorted({c for c in used if used.count(c) > 1})
        if dupes:
            problems.append(f"(board, channel) {dupes} assigned to more than one servo")
        return problems

    def _check_shared_address(self, boards):
        """Refuse if wrist and fingers share a channel number while both boards share an address."""
        by_address = {}
        for b in boards:
            by_address.setdefault(so.BOARDS[b], []).append(b)
        for address, names in by_address.items():
            if len(names) < 2:
                continue
            used = {}
            if self.use_wrist:
                for c in so.WRIST_SERVOS.values():
                    if c["channel"] is not None:
                        used.setdefault(c["channel"], set()).add(c["board"])
            for board, ch in fh.channels(self.fingers):
                used.setdefault(ch, set()).add(board)
            clash = sorted(ch for ch, bs in used.items() if len(bs) > 1)
            msg = (f"[hand] boards {names} are both at {hex(address)}: every command reaches both. "
                   "Bridge the address pads (A0 on the 5 V board, A1 on the 8.4 V board).")
            if clash:
                raise SystemExit(msg + f"\n[hand]   channel(s) {clash} are used on both boards -- refusing to run.")
            print(msg + " Channels don't overlap, so continuing.")

    # --- low level -----------------------------------------------------------
    def _write(self):
        if self.wrist_live:
            so.write_wrist(self.bus, *self.wrist_pose)
        if self.fingers:
            fh.write_fingers(self.bus, self.finger_pose, self.fingers)
        self.bus.publish(*self.wrist_pose, fingers=self.finger_pose)

    def tick(self):
        """Check the E-stop, write the current pose, wait one control period."""
        self.power.check()
        self._write()
        if self.realtime:
            self.power.sleep(1.0 / CONTROL_HZ)

    def pulses(self):
        """What the servos are being sent right now (calibrated servos only)."""
        out = {}
        if self.wrist_live:
            out.update({f"wrist {n}": p for n, p in so.wrist_pulses(*self.wrist_pose)[0].items()})
        out.update(fh.finger_pulses(self.finger_pose, self.fingers))
        return out

    # --- motions ---------------------------------------------------------------
    def arm(self):
        """Servo signals on (after Enter; servo power comes from the box's RESET) with every servo at home."""
        def write_home():
            self.wrist_pose = (0.0, 0.0)
            self.finger_pose = dict(fm.OPEN)
            self._write()
        self.power.arm(write_home)
        self._write()

    def move(self, wrist=None, fingers=None, seconds=EASE_SECONDS):
        """Ease to a new pose. wrist = (deviation, flexion) deg; fingers = {servo name: curl}."""
        start_w, start_f = self.wrist_pose, dict(self.finger_pose)
        end_w = start_w
        if wrist is not None and self.use_wrist:
            end_w = (max(WRIST_DEV_LIMITS[0], min(WRIST_DEV_LIMITS[1], float(wrist[0]))),
                     max(WRIST_FLEX_LIMITS[0], min(WRIST_FLEX_LIMITS[1], float(wrist[1]))))
        end_f = fm.clamp_pose({**start_f, **(fingers or {})})
        steps = max(1, int(round(seconds * CONTROL_HZ)))
        for k in range(1, steps + 1):
            s = smooth(k / steps)
            self.wrist_pose = (start_w[0] + (end_w[0] - start_w[0]) * s,
                               start_w[1] + (end_w[1] - start_w[1]) * s)
            self.finger_pose = fm.blend(start_f, end_f, s)
            self.tick()

    def hold(self, seconds):
        """Stay put, still checking the E-stop and refreshing the sim."""
        for _ in range(max(1, int(round(seconds * CONTROL_HZ)))):
            self.tick()

    def play(self, schedule, speed=1.0, lead_in=EASE_SECONDS):
        """Play a strum schedule, fingers held. Returns per-stroke scheduled vs commanded times (s)."""
        if not schedule:
            return []
        self.move(wrist=wrist_at(schedule, -1.0)[:2], seconds=lead_in)
        end_time = schedule[-1][0] + schedule[-1][2] + schedule[-1][3]
        t0 = schedule[0][0]
        period = 1.0 / CONTROL_HZ
        log, reported = [], -1
        start = time.perf_counter()
        clock = 0.0   # seconds since start: wall clock on hardware, virtual in a fast dry run
        while True:
            song_t = schedule[0][0] + clock * speed
            if song_t > end_time:
                break
            self.power.check()
            dev, flex, i = wrist_at(schedule, song_t)
            if self.use_wrist:
                self.wrist_pose = (dev, flex)
            self._write()
            if i > reported:
                log.append({"stroke": i, "direction": schedule[i][1],
                            "scheduled_s": (schedule[i][0] - t0) / speed,
                            "commanded_s": clock})
                reported = i
            if self.realtime:
                target = clock + period
                wait = target - (time.perf_counter() - start)
                if wait > 0:
                    time.sleep(wait)
                clock = max(target, time.perf_counter() - start)   # fell behind: resync, no burst
            else:
                clock += period
        return log

    def park(self, seconds=EASE_SECONDS):
        self.move(wrist=(0.0, 0.0), fingers=fm.OPEN, seconds=seconds)

    # --- shutdown --------------------------------------------------------------
    def close(self, park=True):
        """Park (only on a clean finish), then signals off and PWM off. Always safe to call."""
        if park and self.power.state == "ARMED" and not self.power.tripped.is_set() and not self.power.closed:
            try:
                self.park()
            except Exception as e:   # never let parking hide the real error
                print(f"[hand] could not park ({e})")
        self.power.close()
        self.bus.release()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close(park=exc_type is None)
        if exc_type is EStop:
            print("[hand] stopped by the E-stop. Signals off. Restart the script to re-arm.")
            return True
        if exc_type is KeyboardInterrupt:
            print("\n[hand] interrupted. Signals off.")
            return True
        return False
