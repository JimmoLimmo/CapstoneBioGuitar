"""Per-tick record of a run: what was commanded and how late each control tick started.

Rows stay in memory during the run (nothing is written to disk inside the
control loop) and go to a CSV in Testing/results/ at the end.
"""

import csv
import os
import re
from datetime import datetime

from calibration import hand_back

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RESULTS_DIR = os.path.join(REPO_ROOT, "Testing", "results")


class RunLog:
    def __init__(self, period, servo_names, keep_rows=False):
        self.period = period
        self.servo_names = tuple(servo_names)
        self.keep_rows = keep_rows
        self.rows = []
        self.ticks = 0
        self.late_total = 0.0
        self.worst = 0.0
        self.missed = 0    # ticks that started more than one period late

    def add(self, wall_s, song_s, stroke, direction, dev_deg, flex_deg, pulses, late_s):
        late = max(0.0, late_s)
        self.ticks += 1
        self.late_total += late
        self.worst = max(self.worst, late)
        if late > self.period:
            self.missed += 1
        if self.keep_rows:
            self.rows.append((wall_s, song_s, stroke, direction, dev_deg, flex_deg, pulses, late))

    def summary(self):
        if not self.ticks:
            return "no ticks ran"
        return (f"{self.ticks} ticks, mean lateness {self.late_total / self.ticks * 1000:.2f} ms, "
                f"worst {self.worst * 1000:.1f} ms, {self.missed} more than one "
                f"{self.period * 1000:.0f} ms period late")

    def save(self, label, dry_run=False, folder=None):
        """Write the CSV and return its path. Dry runs go to results/dry_run/."""
        folder = folder or (os.path.join(RESULTS_DIR, "dry_run") if dry_run else RESULTS_DIR)
        os.makedirs(folder, exist_ok=True)
        slug = re.sub(r"[^A-Za-z0-9]+", "-", label).strip("-")[:40] or "run"
        file_path = os.path.join(folder, f"run_{slug}_{datetime.now():%Y%m%d-%H%M%S}.csv")
        with open(file_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["wall_s", "song_s", "stroke", "direction", "dev_deg", "flex_deg",
                             *(f"{n}_pulse" for n in self.servo_names), "late_ms"])
            for wall_s, song_s, stroke, direction, dev, flex, pulses, late in self.rows:
                writer.writerow([f"{wall_s:.4f}", f"{song_s:.4f}", stroke, direction, f"{dev:.2f}", f"{flex:.2f}",
                                 *(pulses.get(n, "") for n in self.servo_names), f"{late * 1000:.2f}"])
        hand_back(file_path)
        return file_path
