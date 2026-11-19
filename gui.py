# Demo launcher: pick the music demo or the camera mimic, set the options, press Start.
#
#     python3 gui.py               (from a terminal on the Jetson's desktop, without sudo)
#     python3 gui.py --shortcut    (once: puts a launcher icon on the desktop)
#
# It runs run.py with the chosen options and shows its output. Arm answers the
# "Press Enter" prompts the scripts already ask, and Stop (or Esc) is the same as Ctrl-C.
# The E-stop works exactly as it does from the terminal. While the mimic runs, its
# hand, wrist, preview and swap boxes change it live (over a local UDP port).

import codecs
import json
import os
import queue
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox, simpledialog, ttk

ROOT = os.path.dirname(os.path.abspath(__file__))
SONG_DIR = os.path.join(ROOT, "Parsing", "inputs")
SETTINGS = os.path.join(os.path.expanduser("~"), ".config", "bioguitar_gui.json")
NO_SONG = "none (constant tempo)"
STROKES = ("default", "D", "DU", "auto")
CONTROL_PORT = 5006     # hand_mimic.py --control-port; the sim uses 5005
RESEND_S = 1.0          # live settings are sent again this often, in case one went out before the mimic was listening
LOG_LINES = 3000
LARGE = 1.5
FONTS = ("TkDefaultFont", "TkTextFont", "TkFixedFont", "TkHeadingFont", "TkMenuFont", "TkCaptionFont")

# what the scripts print, and the status line to show for it
STATUS = (
    ("Press Enter to ARM", "Waiting to arm: hands clear and E-stop in reach, then press Arm."),
    ("ARMED", "Armed and running. Stop or Esc ends it."),
    ("would arm here", "Running (dry run, nothing moves)."),
    ("settings: following", "Running with the new settings."),
    ("paused, hands open", "Paused: tick Left or Right to carry on."),
    ("Press Enter, then hold still", "Calibrating: get your hand in place, then press Next."),
    ("refusing to move", "Refused to start, see the log."),
    ("stopped by the E-stop", "Stopped by the E-stop."),
)


def song_files():
    try:
        return sorted(f for f in os.listdir(SONG_DIR) if f.lower().endswith((".gp3", ".gp4", ".gp5")))
    except OSError:
        return []


# run.py options for the music demo. Returns (options, problem).
def music_options(song, track, speed, pattern, bpm, strokes, wrist_only, log):
    try:
        speed = float(speed)
    except ValueError:
        return None, "Speed has to be a number."
    if not 0 < speed <= 4:
        return None, "Speed has to be between 0 and 4."
    opts = ["song", "--speed", f"{speed:g}"]

    pattern = pattern.strip().upper()
    if pattern:
        if set(pattern) - set("DU-"):
            return None, "A pattern is only D, U and - (one per eighth note), like D-DU-UDU."
        opts += ["--pattern", pattern]
    elif song == NO_SONG:
        opts += ["--song", "none"]
    else:
        opts += ["--song", song]
        if track.strip():
            if not track.strip().isdigit():
                return None, "Track has to be a whole number."
            opts += ["--track", track.strip()]

    if bpm.strip() and (pattern or song == NO_SONG):
        try:
            if float(bpm) <= 0:
                raise ValueError
        except ValueError:
            return None, "BPM has to be a number above 0."
        opts += ["--bpm", bpm.strip()]

    strokes = strokes.strip()
    if strokes and strokes != "default":
        strokes = "auto" if strokes.lower() == "auto" else strokes.upper()
        if strokes != "auto" and (not strokes or set(strokes) - set("DU")):
            return None, "Strokes is default, auto, or a cycle of D and U like DU or DDU."
        opts += ["--directions", strokes]
    if wrist_only:
        opts.append("--wrist-only")
    if log:
        opts.append("--log")
    return opts, None


# Both hands ticked: "together" follows both at once, otherwise whichever hand the camera sees.
def hand_mode(left, right, together):
    if left and right:
        return "both" if together else "auto"
    return "left" if left else ("right" if right else "none")


# run.py options for the mimic. Returns (options, problem).
def mimic_options(left, right, together, wrist, preview, swap, camera, calibrate=False):
    if not (left or right):
        return None, "Tick Left, Right, or both."
    if wrist and not right and not calibrate:
        return None, "Wrist motion follows the right hand, so tick Right too."
    hand = "both" if calibrate and left and right else hand_mode(left, right, together)
    opts = ["mimic", "--hand", hand, "--camera", str(camera)]
    if calibrate:
        opts += ["--calibrate", "--dry-run", "--no-sim"]
    else:
        opts += ["--control-port", str(CONTROL_PORT)]
        if wrist:
            opts.append("--wrist")
    if preview:
        opts.append("--show")
    if swap:
        opts.append("--swap-hands")
    return opts, None


# What a running mimic is told when a box changes.
def live_message(left, right, together, wrist, preview, swap):
    return {"hand": hand_mode(left, right, together), "wrist": bool(wrist and right),
            "show": bool(preview), "swap": bool(swap)}


# Options for both demos. Returns (options, problem).
def shared_options(dry_run, sim, delay):
    opts = ["--dry-run"] if dry_run else []
    if not sim:
        return opts + ["--no-sim"], None
    try:
        if float(delay or 0) < 0:
            raise ValueError
    except ValueError:
        return None, "Sim delay has to be a number of seconds (0 or more)."
    if float(delay or 0) > 0:
        opts += ["--sim-delay", delay.strip()]
    return opts, None


def sudo_cached():
    return subprocess.call(["sudo", "-n", "true"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0


def sudo_login(password):
    try:
        done = subprocess.run(["sudo", "-S", "-p", "", "-v"], input=(password + "\n").encode(),
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=20)
    except subprocess.TimeoutExpired:
        return False
    return done.returncode == 0


def load_settings():
    try:
        with open(SETTINGS) as f:
            saved = json.load(f)
        return saved if isinstance(saved, dict) else {}
    except (OSError, ValueError):
        return {}


# A desktop icon that opens the GUI in a terminal (the terminal is what lets sudo ask for the password).
def make_shortcut():
    entry = "\n".join((
        "[Desktop Entry]",
        "Type=Application",
        "Name=Guitar Hand Demos",
        "Comment=Music and mimic demos for the guitar hand",
        f'Exec="{sys.executable}" "{os.path.abspath(__file__)}"',
        f"Path={ROOT}",
        "Icon=applications-multimedia",
        "Terminal=true",
        "",
    ))
    home = os.path.expanduser("~")
    targets = [os.path.join(home, ".local", "share", "applications", "guitar-hand-demos.desktop")]
    if os.path.isdir(os.path.join(home, "Desktop")):
        targets.append(os.path.join(home, "Desktop", "guitar-hand-demos.desktop"))
    for path in targets:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(entry)
        os.chmod(path, 0o755)
        subprocess.call(["gio", "set", path, "metadata::trusted", "true"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[gui] made {path}")
    print("[gui] If the desktop icon won't open, right-click it and pick Allow Launching.")


# Runs run.py in its own process group with its output collected in a queue.
class Runner:

    def __init__(self):
        self.proc = None
        self.reader = None
        self.output = queue.Queue()
        self.stops = 0

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    # True once the process has exited and all of its output has been read.
    def finished(self):
        return self.proc is not None and not self.running() and not self.reader.is_alive() and self.output.empty()

    def start(self, opts):
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        group = os.setpgrp if os.name == "posix" else None
        self.proc = subprocess.Popen([sys.executable, "-u", "run.py", *opts], cwd=ROOT, env=env,
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     preexec_fn=group)
        self.stops = 0
        self.reader = threading.Thread(target=self._read, args=(self.proc,), daemon=True)
        self.reader.start()

    def _read(self, proc):
        decoder = codecs.getincrementaldecoder("utf-8")("replace")
        fd = proc.stdout.fileno()
        while True:
            chunk = os.read(fd, 4096)
            if not chunk:
                break
            self.output.put(decoder.decode(chunk))
        proc.stdout.close()

    def enter(self):
        try:
            self.proc.stdin.write(b"\n")
            self.proc.stdin.flush()
        except (OSError, AttributeError):
            pass

    # First press is Ctrl-C (the scripts ease the hand open and shut off). A second press forces it.
    def stop(self):
        if not self.running():
            return
        self.stops += 1
        if os.name != "posix":
            self.proc.terminate()
            return
        sig = signal.SIGINT if self.stops == 1 else signal.SIGTERM
        try:
            os.killpg(self.proc.pid, sig)
        except OSError:
            self.proc.send_signal(sig)


class App:

    def __init__(self, root):
        self.root = root
        self.runner = Runner()
        self.control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mode = None
        self.recent = ""
        self.waiting = False
        self.closing = False
        self.next_send = 0.0
        self.inputs = []
        self.live_inputs = []
        self.vars = {}
        self.base_sizes = {name: tkfont.nametofont(name).cget("size") for name in FONTS}

        root.title("Bio-Inspired Guitarist demos")
        root.minsize(680, 600)
        self.tabs = ttk.Notebook(root)
        self.tabs.pack(fill="x", padx=10, pady=(10, 0))
        self.tabs.add(self.music_tab(), text="  Music demo  ")
        self.tabs.add(self.mimic_tab(), text="  Mimic demo  ")

        shared = ttk.Frame(root)
        shared.pack(fill="x", padx=12, pady=8)
        self.dry_run = self.var("dry_run", False)
        self.sim = self.var("sim", True)
        self.sim_delay = self.var("sim_delay", "0")
        self.large = self.var("large_text", False)
        self.add(ttk.Checkbutton(shared, text="Dry run (servos don't move)", variable=self.dry_run)).pack(side="left")
        self.add(ttk.Checkbutton(shared, text="Show the sim", variable=self.sim,
                                 command=self.refresh)).pack(side="left", padx=(16, 0))
        ttk.Label(shared, text="Sim delay (s)").pack(side="left", padx=(16, 4))
        self.delay_entry = self.add(ttk.Entry(shared, textvariable=self.sim_delay, width=6))
        self.delay_entry.pack(side="left")
        ttk.Checkbutton(shared, text="Large text", variable=self.large, command=self.text_size).pack(side="right")

        controls = ttk.Frame(root)
        controls.pack(fill="x", padx=12, pady=(0, 6))
        self.start_button = ttk.Button(controls, text="Start", command=self.start)
        self.start_button.pack(side="left")
        self.arm_button = ttk.Button(controls, text="Arm", command=self.arm, state="disabled")
        self.arm_button.pack(side="left", padx=6)
        self.stop_button = ttk.Button(controls, text="Stop", command=self.stop, state="disabled")
        self.stop_button.pack(side="left")
        self.status = ttk.Label(controls, text="Idle.")
        self.status.pack(side="left", padx=12)

        box = ttk.Frame(root)
        box.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.text = tk.Text(box, height=16, wrap="word", state="disabled", font="TkFixedFont")
        scroll = ttk.Scrollbar(box, command=self.text.yview)
        self.text.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.text.pack(side="left", fill="both", expand=True)

        self.restore()
        root.bind("<Escape>", lambda _e: self.stop())
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        if os.name == "posix" and not os.isatty(0):
            self.log("[gui] started without a terminal; if sudo complains, use the desktop icon "
                     "(python3 gui.py --shortcut) or start it from a terminal.\n")
        self.text_size()
        self.refresh()
        self.root.after(50, self.poll)

    def var(self, name, default):
        v = tk.BooleanVar(value=default) if isinstance(default, bool) else tk.StringVar(value=default)
        self.vars[name] = v
        return v

    def add(self, widget, live=False):
        self.inputs.append(widget)
        if live:
            self.live_inputs.append(widget)
        return widget

    def music_tab(self):
        tab = ttk.Frame(self.tabs, padding=10)
        self.songs = song_files()
        first = next((s for s in self.songs if s.lower().startswith("apollo")), self.songs[0] if self.songs else NO_SONG)
        self.song = self.var("song", first)
        self.track = self.var("track", "1")
        self.speed = self.var("speed", "1.0")
        self.pattern = self.var("pattern", "")
        self.bpm = self.var("bpm", "")
        self.strokes = self.var("strokes", "default")
        self.wrist_only = self.var("wrist_only", False)
        self.run_log = self.var("run_log", False)

        self.song_box = ttk.Combobox(tab, textvariable=self.song, values=[NO_SONG, *self.songs],
                                     state="readonly", width=44)
        rows = (
            ("Song", self.song_box, ""),
            ("Track", ttk.Spinbox(tab, textvariable=self.track, from_=0, to=20, width=6), "track number in the song"),
            ("Speed", ttk.Spinbox(tab, textvariable=self.speed, from_=0.1, to=2.0, increment=0.05, width=6),
             "1.0 = real speed, 0.25 for first runs"),
            ("Pattern", ttk.Entry(tab, textvariable=self.pattern, width=20), "e.g. D-DU-UDU, overrides the song"),
            ("BPM", ttk.Entry(tab, textvariable=self.bpm, width=8), "for a pattern or no song (blank = 90)"),
            ("Strokes", ttk.Combobox(tab, textvariable=self.strokes, values=STROKES, width=10), "D, DU, DDU... or auto"),
        )
        for r, (name, widget, hint) in enumerate(rows):
            ttk.Label(tab, text=name).grid(row=r, column=0, sticky="w", pady=2)
            self.add(widget).grid(row=r, column=1, sticky="w", padx=8)
            ttk.Label(tab, text=hint, foreground="gray").grid(row=r, column=2, sticky="w")
        flags = ttk.Frame(tab)
        flags.grid(row=len(rows), column=0, columnspan=3, sticky="w", pady=(6, 0))
        self.add(ttk.Checkbutton(flags, text="Wrist only", variable=self.wrist_only)).pack(side="left")
        self.add(ttk.Checkbutton(flags, text="Save a run log (Testing/results)",
                                 variable=self.run_log)).pack(side="left", padx=16)
        ttk.Label(tab, text="These apply the next time you press Start.", foreground="gray").grid(
            row=len(rows) + 1, column=0, columnspan=3, sticky="w", pady=(6, 0))
        return tab

    def mimic_tab(self):
        tab = ttk.Frame(self.tabs, padding=10)
        self.left = self.var("left", False)
        self.right = self.var("right", True)
        self.together = self.var("together", False)
        self.wrist = self.var("wrist", False)
        self.preview = self.var("preview", True)
        self.swap = self.var("swap", False)
        self.camera = self.var("camera", "0")

        def box(parent, text, variable):
            return self.add(ttk.Checkbutton(parent, text=text, variable=variable, command=self.changed), live=True)

        hands = ttk.Frame(tab)
        hands.grid(row=0, column=0, sticky="w")
        ttk.Label(hands, text="Hands").pack(side="left", padx=(0, 10))
        box(hands, "Left", self.left).pack(side="left")
        box(hands, "Right", self.right).pack(side="left", padx=10)
        self.together_box = box(tab, "Both hands at the same time (otherwise it follows whichever hand the camera sees)",
                                self.together)
        self.together_box.grid(row=1, column=0, sticky="w", pady=2)
        self.wrist_box = box(tab, "Wrist motion (follows the right hand's tilt)", self.wrist)
        self.wrist_box.grid(row=2, column=0, sticky="w", pady=2)
        box(tab, "Camera preview window", self.preview).grid(row=3, column=0, sticky="w", pady=2)
        box(tab, "Swap left/right (if your right hand shows up as left)", self.swap).grid(
            row=4, column=0, sticky="w", pady=2)
        cam = ttk.Frame(tab)
        cam.grid(row=5, column=0, sticky="w", pady=2)
        ttk.Label(cam, text="Camera").pack(side="left")
        self.add(ttk.Spinbox(cam, textvariable=self.camera, from_=0, to=9, width=4)).pack(side="left", padx=8)
        self.add(ttk.Button(tab, text="Calibrate tracking (nothing moves)",
                            command=lambda: self.start(calibrate=True))).grid(row=6, column=0, sticky="w", pady=(8, 0))
        ttk.Label(tab, text="The boxes above change the mimic live while it runs. Untick both hands to pause it.",
                  foreground="gray").grid(row=7, column=0, sticky="w", pady=(6, 0))
        return tab

    def restore(self):
        saved = load_settings()
        for name, value in saved.items():
            v = self.vars.get(name)
            if v is None or type(value) is not (bool if isinstance(v, tk.BooleanVar) else str):
                continue
            if name == "song" and value not in (NO_SONG, *self.songs):
                continue
            v.set(value)
        if saved.get("tab") in (0, 1):
            self.tabs.select(saved["tab"])

    def save(self):
        settings = {name: v.get() for name, v in self.vars.items()}
        settings["tab"] = self.tabs.index("current")
        try:
            os.makedirs(os.path.dirname(SETTINGS), exist_ok=True)
            with open(SETTINGS, "w") as f:
                json.dump(settings, f, indent=1)
        except OSError:
            pass

    def text_size(self):
        scale = LARGE if self.large.get() else 1.0
        for name, size in self.base_sizes.items():
            tkfont.nametofont(name).configure(size=round(size * scale))

    # Grey out what doesn't apply. While the mimic runs, only its live boxes stay on.
    def refresh(self):
        busy = self.mode is not None
        live = self.mode == "mimic"
        for w in self.inputs:
            w.configure(state="normal" if not busy or (live and w in self.live_inputs) else "disabled")
        if not busy:
            self.song_box.configure(state="readonly")
            self.delay_entry.configure(state="normal" if self.sim.get() else "disabled")
        if not busy or live:
            both = self.left.get() and self.right.get()
            self.together_box.configure(state="normal" if both else "disabled")
            self.wrist_box.configure(state="normal" if self.right.get() else "disabled")
        self.start_button.configure(state="disabled" if busy else "normal")
        self.stop_button.configure(state="normal" if busy else "disabled")
        self.arm_button.configure(state="normal" if busy and self.waiting else "disabled",
                                  text="Next" if self.mode == "calibrate" else "Arm")

    def changed(self):
        self.refresh()
        if self.mode == "mimic":
            self.send_live()

    def send_live(self):
        msg = live_message(self.left.get(), self.right.get(), self.together.get(), self.wrist.get(),
                           self.preview.get(), self.swap.get())
        try:
            self.control.sendto(json.dumps(msg).encode(), ("127.0.0.1", CONTROL_PORT))
        except OSError:
            pass
        self.next_send = time.monotonic() + RESEND_S

    def start(self, calibrate=False):
        if self.mode is not None:
            return
        music = not calibrate and self.tabs.index("current") == 0
        if music:
            opts, problem = music_options(self.song.get(), self.track.get(), self.speed.get(), self.pattern.get(),
                                          self.bpm.get(), self.strokes.get(), self.wrist_only.get(), self.run_log.get())
        else:
            opts, problem = mimic_options(self.left.get(), self.right.get(), self.together.get(), self.wrist.get(),
                                          self.preview.get(), self.swap.get(), self.camera.get(), calibrate)
        if not problem and not calibrate:
            extra, problem = shared_options(self.dry_run.get(), self.sim.get(), self.sim_delay.get())
            opts = (opts or []) + (extra or [])
        if problem:
            messagebox.showwarning("Can't start", problem, parent=self.root)
            return
        if "--dry-run" not in opts and os.name == "posix" and not self.sudo_ok():
            return

        self.save()
        self.mode = "calibrate" if calibrate else ("music" if music else "mimic")
        self.recent, self.waiting = "", False
        self.log(f"\n$ python3 run.py {shlex.join(opts)}\n")
        self.status.configure(text="Starting...")
        self.runner.start(opts)
        self.next_send = time.monotonic() + RESEND_S
        self.refresh()

    def sudo_ok(self):
        if sudo_cached():
            return True
        password = simpledialog.askstring("sudo", "Password for the servos (sudo):", show="*", parent=self.root)
        if password is None:
            return False
        if sudo_login(password):
            return True
        messagebox.showerror("sudo", "That password didn't work.", parent=self.root)
        return False

    def arm(self):
        if not self.waiting:
            return
        self.runner.enter()
        self.waiting = False
        self.recent += "\n"
        self.log("\n")
        self.refresh()

    def stop(self):
        if self.mode is None:
            return
        self.runner.stop()
        self.waiting = False
        self.status.configure(text="Stopping... (press Stop again to force it)")
        self.refresh()

    def poll(self):
        text = ""
        while True:
            try:
                text += self.runner.output.get_nowait()
            except queue.Empty:
                break
        if text:
            self.log(text)
            self.recent = (self.recent + text)[-400:]
            # a prompt is the last thing printed before the script waits for Enter
            self.waiting = "Press Enter" in self.recent.rsplit("\n", 1)[-1]
            found = max(STATUS, key=lambda s: self.recent.rfind(s[0]))
            if self.recent.rfind(found[0]) >= 0:
                self.status.configure(text=found[1])
            self.refresh()

        if self.mode == "mimic" and self.runner.running() and time.monotonic() >= self.next_send:
            self.send_live()
        if self.mode is not None and self.runner.finished():
            code = self.runner.proc.returncode
            self.log(f"[gui] finished (exit code {code})\n")
            self.status.configure(text="Idle." if code == 0 else f"Ended with exit code {code}, see the log.")
            self.mode, self.waiting = None, False
            self.refresh()
        if self.closing and self.mode is None:
            self.root.destroy()
            return
        self.root.after(50, self.poll)

    def log(self, text):
        self.text.configure(state="normal")
        self.text.insert("end", text)
        extra = int(self.text.index("end-1c").split(".")[0]) - LOG_LINES
        if extra > 0:
            self.text.delete("1.0", f"{extra + 1}.0")
        self.text.see("end")
        self.text.configure(state="disabled")

    def on_close(self):
        self.save()
        if self.mode is None:
            self.root.destroy()
            return
        if messagebox.askyesno("Quit", "A demo is still running. Stop it and quit?", parent=self.root):
            self.closing = True
            self.stop()
            self.root.after(8000, self.runner.stop)


def main():
    if "--shortcut" in sys.argv[1:]:
        make_shortcut()
        return
    if os.name == "posix" and os.geteuid() == 0:
        sys.exit("[gui] run this without sudo; it asks for your password when the servos need it.")
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
