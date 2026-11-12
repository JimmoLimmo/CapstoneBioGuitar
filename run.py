"""One command for the hand and the sim together.

    python3 run.py                                  # console, sim alongside
    python3 run.py song --wrist-only --speed 0.25   # play the song
    python3 run.py pick                             # grip the pick
    python3 run.py test strum --cycles 20           # a CDR test
    python3 run.py mimic --show                     # show the camera a hand; that robot hand follows
    python3 run.py console --dry-run                # no hardware
    python3 run.py song --no-sim                    # hand only
    python3 run.py song --sim-delay 0.08            # hold the sim back to match servo lag

Run it without sudo: the sim needs your desktop, and the hand part asks for
your password itself.
"""
import os
import socket
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join("Sim", "mvmnt_demo.py")
MODES = {
    "console": os.path.join("Hardware", "hand_console.py"),
    "song": os.path.join("Hardware", "servo_output.py"),
    "pick": os.path.join("Hardware", "pick_up.py"),
    "test": os.path.join("Testing", "hand_tests.py"),
    "mimic": os.path.join("Hardware", "hand_mimic.py"),
}
PORT = 5005
SIM_WAIT_S = 20


def port_taken():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("0.0.0.0", PORT))
        return False
    except OSError:
        return True
    finally:
        s.close()


def start_sim(delay):
    """(sim process or None, whether anything is listening for poses)."""
    if port_taken():
        print(f"[run] a sim is already open on UDP {PORT}; using it.")
        return None, True
    sim = subprocess.Popen([sys.executable, SIM, "--follow", "--port", str(PORT), "--delay", delay], cwd=ROOT)
    end = time.monotonic() + SIM_WAIT_S
    while time.monotonic() < end:
        if sim.poll() is not None:
            print("[run] the sim closed on its own (is pybullet installed?); carrying on without it.")
            return None, False
        if port_taken():
            return sim, True
        time.sleep(0.2)
    print("[run] the sim is slow to open; carrying on, it will catch up.")
    return sim, True


def main():
    args = sys.argv[1:]
    if args and args[0] in ("-h", "--help"):
        print(__doc__)
        return
    mode = args.pop(0) if args and args[0] in MODES else "console"

    use_sim = "--no-sim" not in args
    args = [a for a in args if a != "--no-sim"]
    delay = "0"
    if "--sim-delay" in args:
        i = args.index("--sim-delay")
        delay = args[i + 1] if i + 1 < len(args) else "0"
        del args[i:i + 2]
    helping = "-h" in args or "--help" in args
    reporting = mode == "test" and (not args or args[0] == "report")
    if helping or reporting:
        use_sim = False

    real = "--dry-run" not in args and not helping and not reporting
    if os.name == "posix" and os.geteuid() == 0:
        sys.exit("[run] run this without sudo; it asks for your password for the servo part.")
    needs_sudo = real and os.name == "posix"
    if needs_sudo and subprocess.call(["sudo", "-v"]) != 0:   # password first, before the sim's output
        sys.exit("[run] sudo is needed for the servos (GPIO and I2C).")

    sim, listening = start_sim(delay) if use_sim else (None, False)
    cmd = [sys.executable, MODES[mode], *args]
    if listening:
        cmd += ["--sim-ip", "127.0.0.1"]
    if needs_sudo:
        cmd = ["sudo", *cmd]

    hand = subprocess.Popen(cmd, cwd=ROOT)
    code = None
    while code is None:
        try:
            code = hand.wait()
        except KeyboardInterrupt:
            pass   # the hand gets Ctrl-C too and shuts its relays off itself; wait for it

    if sim and sim.poll() is None:
        sim.terminate()
        try:
            sim.wait(5)
        except subprocess.TimeoutExpired:
            sim.kill()
    sys.exit(code)


if __name__ == "__main__":
    main()
