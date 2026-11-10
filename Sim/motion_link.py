"""Live command link between the hand's control scripts and the PyBullet sim.

Each wrist command (plus the finger curls, when in use) is sent over UDP.
mvmnt_demo.py --follow shows those commands, so the screen and the real hand
play from one timeline.

UDP is used on purpose: sending never waits for the sim, and a lost packet is
replaced by the next one 20 ms later. The servos must never depend on the sim.

Commands only: the servos report no position, so the sim shows where the wrist
was told to go, not where it actually is.
"""

import json
import socket
import time
from collections import deque

from finger_motion import SERVO_NAMES

DEFAULT_PORT = 5005


def encode_pose(run, seq, dev, flex, fingers=None):
    msg = {"run": run, "seq": seq, "dev": round(dev, 3), "flex": round(flex, 3)}
    if fingers is not None:
        msg["f"] = [round(float(fingers.get(name, 0.0)), 3) for name in SERVO_NAMES]
    return json.dumps(msg).encode()


def decode_pose(data):
    """A pose dict (fingers = {servo: curl} or None), or None if the packet isn't one of ours."""
    try:
        msg = json.loads(data.decode())
        fingers = None
        if msg.get("f") is not None:
            curls = [max(0.0, min(1.0, float(c))) for c in msg["f"]]
            if len(curls) != len(SERVO_NAMES):
                return None
            fingers = dict(zip(SERVO_NAMES, curls))
        return {"run": int(msg["run"]), "seq": int(msg["seq"]),
                "dev": float(msg["dev"]), "flex": float(msg["flex"]), "fingers": fingers}
    except (ValueError, KeyError, TypeError, AttributeError):
        return None


class PoseSender:
    """Used inside the servo loop, so it never blocks and never raises on a send."""

    def __init__(self, host, port=DEFAULT_PORT):
        self.addr = (socket.gethostbyname(host), port)   # resolve once, before the loop
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.run = int(time.time() * 1000)   # lets the sim notice a restart
        self.seq = 0
        self.warned = False

    def send(self, dev, flex, fingers=None):
        self.seq += 1
        try:
            self.sock.sendto(encode_pose(self.run, self.seq, dev, flex, fingers), self.addr)
        except OSError as e:
            if not self.warned:
                print(f"[sim-link] send failed ({e}); servos keep running, sim may freeze.")
                self.warned = True


class PoseFollower:
    """Orders incoming poses and holds them back by `delay` seconds.

    Late packets are dropped, and poll() always returns the newest pose that is
    due, so the sim jumps forward if it falls behind instead of building a backlog.
    """

    def __init__(self, delay=0.0):
        self.delay = delay
        self.queue = deque()
        self.run = None
        self.seq = 0
        self.last_received = None

    def feed(self, msg, now):
        if msg["run"] != self.run:    # servo_output was restarted
            self.run = msg["run"]
            self.seq = 0
            self.queue.clear()
        if msg["seq"] <= self.seq:
            return
        self.seq = msg["seq"]
        self.last_received = now
        self.queue.append((now, msg))

    def poll(self, now):
        """(dev, flex, fingers) if a new pose is due, otherwise None."""
        due = None
        while self.queue and self.queue[0][0] <= now - self.delay:
            due = self.queue.popleft()[1]
        return (due["dev"], due["flex"], due["fingers"]) if due else None


class PoseReceiver:
    def __init__(self, port=DEFAULT_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)

    def drain(self, follower, now):
        """Move every waiting packet into the follower."""
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except OSError:   # nothing waiting
                return
            msg = decode_pose(data)
            if msg:
                follower.feed(msg, now)
