# Bio-Inspired Guitarist

New team? Start with "Start here: get the code" just below. This README covers what the code does, its architecture, how to run it, and what is still unfinished. Step-by-step bench procedures (calibrating servos, wiring checks, finger tuning) are in section 8.

Last updated: November 2026 (Fall 2026 team).

## Start here: get the code

The code starts from the Fall 2026 repo: **https://github.com/JimmoLimmo/CapstoneBioGuitar**

Don't work in that repo. Make your team's own copy first, then use your copy's link everywhere these docs show `https://github.com/<owner>/<repo>.git`. A free GitHub organization the whole team owns is better than one person's account: the repo stays when that person leaves.

1. **Make your copy.** Open the link above and click **Fork**, picking your team's organization or account. The fork keeps the full history. For a private repo, use GitHub's **Import repository** (on the **+** menu) with the same link instead, since a fork of a public repo can't be made private.
2. **Clone your copy** onto the laptop:
   ```
   git clone https://github.com/<owner>/<repo>.git CapstoneBioGuitar-main
   ```
3. **Add teammates** under Settings, Collaborators (or give them access through the organization).

Then carry on with "On a laptop" in section 7.

### If the link doesn't work

If the Fall 2026 repo has been made private or deleted, the same code, with its git history, is on the Jetson in `~/Fall2026/CapstoneBioGuitar`. The Jetson has no internet, so bring it to the laptop first (this needs the direct Ethernet link from "Setting up a new Jetson" in section 7). On GitHub, make a new repository and leave it empty (no README, `.gitignore` or license) so the first push doesn't clash. Then:

On the Jetson:
```
cd ~/Fall2026/CapstoneBioGuitar
git bundle create ~/fall2026.bundle --all
```
On the laptop:
```
scp guitar@192.168.10.2:~/fall2026.bundle .
git clone fall2026.bundle CapstoneBioGuitar-main
cd CapstoneBioGuitar-main
git remote set-url origin https://github.com/<owner>/<repo>.git
git push -u origin --all
```

### Calibration values

Git doesn't carry `Hardware/calibration.json`, so the measured values only reach your repo through `Hardware/calibration.example.json`. If the Jetson's file is newer than that copy, copy it into `calibration.example.json` and commit it, or the values are lost.

## 1. What the project is

We're building a prosthetic-style robot hand that plays guitar. There are two hands:

- **Right hand (strumming), this semester's focus.** It has a ball-and-socket "marionette" wrist pulled by four MG995 servos, and five fingers with two HPS-0618SG servos each. The fingers hold a pick and the wrist sweeps it across the strings in time with a song.
- **Left hand (fretting), from Spring 2026.** It has five servos, one per finger. Right now it only mimics a hand seen by a camera, in free air. It doesn't fret and doesn't run at the same time as the right hand. The thumb is currently

Our main goal is a right hand that grips a pick and strums down and up strokes on time. Stretch goals are camera mimic and learning a strum from video.

Everything runs on a Jetson Orin Nano inside the electrical box. A PyBullet simulation on a laptop can show the same commands live while the real hand runs.

## 2. Status at a glance

| Area | State |
| --- | --- |
| Strum timing (song or pattern to wrist angles) | Working, tested in dry runs and the sim |
| Wrist kinematics (wrist angles to 4 servo angles) | Working, with geometry measured from the built wrist (2026-11-17) |
| Safety layer (E-stop, OE pin, arming) | Working on the box |
| Wrist calibration | Channels (lv board, ch 10-13) and neutral pulses saved; signs and limits not saved yet |
| Finger calibration | Channels assigned (hv board, ch 0-9); pulses are 1700 / 8100 starting values, not measured yet |
| Pick grip poses | Starting guesses, to be tuned on the real hand |
| Live sim alongside the hand | Working (UDP) |
| Camera mimic (left and right) | Working in dry run; left hand has last year's pulses, right-hand fingers have starting pulses only |
| CDR test runner | Working; dry-run results in `Testing/results/dry_run/` |
| Learning a strum from video | In progress (Caeden) |

"Dry run" means the code runs with no hardware attached. Every pulse is calculated and printed, but nothing is sent.

## 3. Repository map

Run every command from the repo root (`CapstoneBioGuitar-main/`). The scripts use paths like `Sim/MPL/right_hand.xml` that are relative to the root.

### Active code (Fall 2026)

| Path | What it is |
| --- | --- |
| `run.py` | One command to start the hand and the sim together. Start here. |
| `Sim/strum_motion.py` | The strum choreography: reads a song or pattern, builds a schedule of strokes, and gives the wrist angles at any time. No hardware or PyBullet imports, so both the sim and the hand use it. |
| `Sim/finger_motion.py` | Finger poses (`open`, `fist`, `pick_ready`, `pick_grip`...). Shared by the sim and the hand. |
| `Sim/motion_link.py` | Sends commands from the hand scripts to the sim over UDP (port 5005). |
| `Sim/mvmnt_demo.py` | The PyBullet sim, with both hands. Plays a song by itself, or with `--follow` shows the commands the real hands are getting, or with `--mimic` follows hands on the camera (fingers only). |
| `Sim/hand_tracking.py` | Camera tracking for the mimic scripts and the sim: MediaPipe landmarks to finger curls, with smoothing. Holds `LEFT_TRACKING` / `RIGHT_TRACKING`. |
| `Sim/hand.py` | Manual joint sliders for the sim hand. |
| `Hardware/hand_control.py` | The `Hand` class: one object that drives the wrist and fingers safely. Most scripts are built on it. |
| `Hardware/servo_output.py` | Plays a song on the real hand. Also holds the wrist servo table (`WRIST_SERVOS`), board addresses and the PCA9685 driver wrapper (`ServoBus`). |
| `Hardware/wrist_kinematics.py` | Turns wrist angles into the four tendon lengths, then into spool angles. |
| `Hardware/fingers.py` | Finger servo table (`FINGER_SERVOS`). Converts curl (0 to 1) into a pulse. |
| `Hardware/power_box.py` | Safety rules for every script: OE pin, E-stop watcher, arming, shutdown. |
| `Hardware/calibration.py` | Loads and saves bench values in `Hardware/calibration.json`. |
| `Hardware/calibrate_servo.py` | Interactive bench tool for jogging a servo and saving its pulses. |
| `Hardware/hand_console.py` | Type commands to move the hand (`pose pick_grip`, `wrist 25 0`, `strum DU 8 90`...). |
| `Hardware/pick_up.py` | Puts the pick in the hand, either handed over or taken from a holder. |
| `Hardware/hand_mimic.py` | Camera mimic: show it a left or right hand and the matching robot hand follows. |
| `Hardware/left_hand.py` | The left hand's servos. Last year's `Imitation/mimic.py`, moved onto this box; the camera part is in `Sim/hand_tracking.py`. |
| `Hardware/i2c_bus.py`, `Hardware/run_log.py` | Helpers: open I2C bus 7; record each control tick to a CSV. |
| `Hardware/test_one_servo.py`, `test_one_servo_armed.py` | Quick wiring checks for one channel. |
| `Hardware/relay_polarity_test.py` | Only for an alternative relay wiring. Not used with the REV 3 box. |
| `Testing/hand_tests.py` | The CDR test plan: fingertip accuracy, strum repeatability, tempo. |
| `requirements.txt`, `requirements-jetson.txt` | Python packages for the laptop and for the Jetson (section 7). |
| `Hardware/calibration.example.json` | A copy of the measured calibration values, the starting point for a new Jetson (section 7). |
| `docs/` | The full guide (box, Jetson setup, calibration, tests, code reference), the CDR and the SOP. |

### In progress (Caeden, learning from video)

| Path | What it is |
| --- | --- |
| `Learning/train_strum_from_video.py` | Tracks a strumming hand in a video with MediaPipe and saves a wrist trajectory (`.npz`) |
| `Sim/play_learned_strum.py` | Plays that trajectory in the PyBullet scene |
| `Sim/play_learned_strum_hw.py`, `Hardware/hand_link.py` | Bridge for sending a learned trajectory to the real wrist |

### From earlier teams (kept for reference, mostly not used now)

| Path | What it is |
| --- | --- |
| `Parsing/GuitarProReader.py` | Reads Guitar Pro files. **Still used**: `strum_motion.py` takes note times from it. |
| `Parsing/inputs/` | Songs (`.gp3`/`.gp5`). The default song is `Apollo-Live.gp3`. |
| `Parsing/Fingering.py`, `FingeringAnalysis.py` | Fretting-hand fingering solver (2023) |
| `Parsing/TabToFinger.py`, `xml_parser.py`, `XMLdom.py` | MusicXML parsing experiments |
| `MediaPipe/hand_landmarker.task` | The MediaPipe hand model file. **Still used** by every camera script. |
| `MediaPipe/tracking.py` | Records hand landmarks to CSV at times from `Parsing/parsed.csv` |
| `Learning/main.py`, `Learning/weights/` | Spring 2026 neural network (scikit-learn MLPRegressor): predicts fretting-hand landmark positions from (string, fret, finger). Training data is `Learning/train.csv` (inputs `string`, `fret`, `finger`; 18 outputs, x/y/z for 5 fingertips + wrist); train with `python3 main.py` from `Learning/`. |
| `Learning/csvcombine*.py` | CSV merge utilities for training data |
| `Imitation/mimic.py`, `debug.py`, `servozero.py` | Spring 2026 left-hand camera mimic. Replaced by `Hardware/left_hand.py`. |
| `Testing/servotest.py`, `servozero.py` | Spring 2026 servo tests |
| `cadenCV_Fall2024/` | Optical music recognition (sheet music image to MIDI), from Fall 2024 |
| `Sim/MPL/` | Generic robot hand models. We load `right_hand.xml` (the only one with a full thumb and a 3-axis wrist) and `left_hand.xml`, its mirror image. `make_left_hand.py` rebuilds the left one. |
| `Sim/guitar_model/` | Guitar model for the sim |

Other things you'll see: `*.bundle` files carry code to the Jetson (section 7). Git ignores them.

## 4. How it fits together

### Strumming a song, one control tick (50 times a second)

| Step | File | Takes | Gives |
| --- | --- | --- | --- |
| 1. Build the schedule (once, at start) | `Sim/strum_motion.py` `build_schedule()` | Guitar Pro file, or a BPM, or a pattern like `D-DU-UDU` | List of strokes: (start time, D or U, strike length, recovery length) |
| 2. Wrist pose at time t | `strum_motion.wrist_at()` | Schedule and current time | Deviation angle (the strum sweep) and flexion angle (the lift) in degrees |
| 3. Tendon geometry | `Hardware/wrist_kinematics.py` `servo_angles()` | Deviation and flexion | Spool angle for each of the 4 wrist servos |
| 4. Calibration | `Hardware/servo_output.py` `wrist_pulses()` | Spool angles | PCA9685 pulse per servo, clamped to that servo's limits |
| 5. Output | `ServoBus.set()` | Pulses | Written to the PCA9685 boards over I2C |
| 6. Sim (optional) | `motion_link.PoseSender` | The same angles and finger curls | UDP packet to `mvmnt_demo.py --follow` |

The fingers work the same way but simpler: a pose from `finger_motion.py` is a curl of 0 to 1 per servo. `fingers.py` maps each curl linearly between that servo's `open_pulse` and `closed_pulse`.

`Hardware/hand_control.py` wraps all of this in the `Hand` class, with `arm()`, `move()`, `hold()`, `play()`, `park()` and `close()`. Each tick also checks the E-stop. New scripts should use `Hand` instead of talking to the boards directly.

### Why the choreography lives in `Sim/`

`strum_motion.py` and `finger_motion.py` are plain Python with no PyBullet or hardware imports. The sim and the real hand import the same file, so they can't drift apart. **To change how the hand strums, edit `strum_motion.py`, not the sim or the hardware scripts.**

### How a stroke moves

- A down-stroke starts at `WRIST_YAW_UP_DEG` (+25) and sweeps to `WRIST_YAW_DOWN_DEG` (-20) in `STRIKE_DURATION` (0.12 s). An up-stroke is the mirror.
- The strike is timed so the pick meets the strings halfway through (`CONTACT_POINT`), right on the note.
- After a down-stroke the wrist is already in the ready position for an up-stroke. So alternating D U strumming needs no return trip.
- When two strokes go the same way (D D), the wrist has to come back. It lifts (`LIFT_FLEX_DEG`) on the way so the pick doesn't drag back through the strings.
- Rests in the song just add waiting time. The stroke speed stays the same whatever the tempo.

### Why there's a kinematics step

Each wrist axis is two servos pulling strings on opposite sides of a ball joint. Mirroring one servo's pulse onto the other leaves a string slack or over-tight, because the two sides don't change length by the same amount. `wrist_kinematics.py` calculates each string's real length instead. Moving one axis also changes the other pair's string lengths. With the measured geometry, a plain mirror would leave about 0.7 mm of slack at the +25 deg strum extreme, and the wrist would rattle at the end of every stroke. Because the geometry is handled in code, each servo only needs its neutral pulse and its direction. Run `python Hardware/wrist_kinematics.py` for a table.

## 5. Hardware the code expects

### Electrical box (REV 3)

| Item | Detail |
| --- | --- |
| I2C | Bus 7 (Jetson header pins 3 and 5). Check with `sudo i2cdetect -y -r 7`. |
| `hv` board, 0x40 | 8.4 V PCA9685: right-hand finger servos only |
| `lv` board, 0x41 | 5 V PCA9685 (changing to 6.4 V): wrist on ch 10-13, left hand on ch 1-4 |
| Pin 31 | OE on both boards. HIGH means every servo signal is off. A pull-up holds it HIGH at boot or after a crash. |
| Pin 15 | E-stop sense. HIGH means pressed or cable cut. |
| Servo power | Switched in hardware only. RESET turns it on; E-stop, thermal switch, fuse or MAIN turn it off. The Jetson doesn't control the relays. |

Note: the REV 3 wiring sheets show the two board addresses the other way round. The code follows what `i2cdetect` showed on the real box (checked 2026-11-10).

### Wrist servos (MG995, run at 6 V)

| Code name | Side | Board / channel |
| --- | --- | --- |
| `flex_pos` | North (back of hand) | lv 12 |
| `flex_neg` | South (palm) | lv 13 |
| `dev_pos` | Pinky side | lv 10 |
| `dev_neg` | Thumb side | lv 11 |

`calibrate_servo.py --wrist` confirms the directions. If a pair turns out backwards, `python3 Hardware/calibrate_servo.py --swap dev` (or `--swap flex`) swaps the pair's saved entries.

### Finger servos

There are ten names, `<finger>_base` and `<finger>_tip`, for thumb, index, middle, ring and pinky, all on the `hv` board:

| Finger | Base | Tip |
| --- | --- | --- |
| Thumb | hv 0 | hv 1 |
| Index | hv 3 | hv 2 |
| Middle | hv 4 | hv 5 |
| Ring | hv 6 | hv 7 |
| Pinky | hv 9 | hv 8 |

Until they are calibrated, every finger has 1700 / 8100 as its starting pulses (the thumb the other way round).

### Left hand

`left_index` through `left_pinky` are on lv channels 1-4 (the left hand has no thumb). They reuse last year's pulses (1638 open, 7000 closed).

### Pulse units

Pulses are PCA9685 `duty_cycle` counts at 50 Hz. 1638 to 8191 is 0.5 ms to 2.5 ms, and every channel is clamped to that range.

## 6. Key ideas and words

| Term | Meaning |
| --- | --- |
| Deviation / `dev` / `wrist_UDEV` | Side-to-side wrist angle; the strum sweep. + is toward the pinky. |
| Flexion / `flex` / `wrist_FLEX` | Wrist bend; used to lift the pick clear of the strings |
| Curl | Finger servo position from 0 (straight) to 1 (fully curled) |
| Schedule | The list of strokes `build_schedule()` makes from a song or pattern |
| Pattern | One character per eighth note: `D`, `U`, `-` for no stroke, e.g. `D-DU-UDU` |
| `--dry-run` | No hardware. Every script takes it. |
| `--sim-ip` | Send commands to the sim at that address (`127.0.0.1` for the same computer) |
| SAFE / WAIT / ARMED | `power_box.py` states. Scripts start in WAIT with signals off, arm when you press Enter, and drop to SAFE if the E-stop is pressed. |
| `calibration.json` | Bench values saved by `calibrate_servo.py`, `hand_console.py` (poses) and `hand_mimic.py --calibrate` (tracking ranges). Lives in `Hardware/` on the Jetson. Git ignores it. |

## 7. Getting started

### On a laptop (sim and dry runs, no hardware)

1. Clone your team's copy of the repo ("Start here" at the top if you don't have one yet):
   ```
   git clone https://github.com/<owner>/<repo>.git CapstoneBioGuitar-main
   ```
2. PyBullet often has no Windows install for the newest Python, so use Anaconda (or Miniconda). Set up an environment once, from the Anaconda Prompt:
   ```
   conda create -n bioguitar python=3.12
   conda activate bioguitar
   conda install -c conda-forge pybullet
   cd CapstoneBioGuitar-main
   pip install -r requirements.txt
   ```
   `requirements.txt` skips its Linux-only packages on Windows. If pip tries to build PyBullet and asks for Microsoft Visual C++, the environment isn't active: PyBullet should come from conda-forge.
3. Each time, run `conda activate bioguitar`, then from the repo root:

```
python run.py song --dry-run            # sim opens, the song plays on screen
python run.py console --dry-run         # type commands, watch the sim
python Hardware/servo_output.py --dry-run   # print the schedule and servo demand only, no sim
python Sim/mvmnt_demo.py --mimic --show     # both sim hands follow your hands on the webcam (fingers only)
```

`run.py` opens the sim, waits for it, then starts the hand script with `--sim-ip 127.0.0.1`. Add `--no-sim` to skip the sim.

#### Camera and sim together (wrist included)

`--mimic` moves the fingers only; the wrist stays straight. To see everything the real hand would be told to do, wrist included, run the real mimic script in a dry run and send it to the sim. Nothing needs the Jetson or the box.

Record your hand once first (open, then a fist, when asked). On the laptop this writes the laptop's own `Hardware/calibration.json`; the Jetson's copy isn't touched:
```
python Hardware/hand_mimic.py --calibrate --hand both --show
```
Then open two Anaconda Prompts, activate `bioguitar` in both and `cd` to the repo root. Start the sim first:
```
python Sim/mvmnt_demo.py --follow
```
and the camera in the second window:
```
python Hardware/hand_mimic.py --dry-run --show --hand both --wrist --sim-ip 127.0.0.1
```
- `127.0.0.1` means this computer. With the sim on another computer, use that computer's address instead.
- `--dry-run` keeps the script away from the servo boards. It still warns about uncalibrated servos; that's expected on the laptop.
- Leave out `--wrist` to move the fingers only, or use `--hand right` to follow one hand.
- Check directions by the hand's own sides: tilt toward your pinky and the sim hand should tilt toward its pinky; bend your hand back and it should bend toward the back of its hand. If one is reversed, flip `WRIST_DEV_SIGN` or `WRIST_FLEX_SIGN` in `Sim/hand_tracking.py`. The sim and the hand get the same numbers, so if only the real hand is reversed, test the servos on their own before changing anything: `calibrate_servo.py --wrist`, type `25 0`. Toward the pinky means the servos are right; toward the thumb means they're swapped (`calibrate_servo.py --swap dev`). A different camera can read the tilt the other way, so check again after changing cameras.
- If the sim shows nothing moving, it wasn't started first, or a firewall is blocking UDP port 5005.

### Setting up a new Jetson

Skip this if the Jetson already has `~/CapstoneBioGuitar`. The Jetson has no internet, so everything goes over a direct Ethernet cable from the laptop. The Jetson user is `guitar`.

1. **Fixed addresses.** On the Jetson (monitor and keyboard), turn on SSH and give the Ethernet port `192.168.10.2`. The Orin Nano's port is usually `enP8p1s0`; `ip -br link` shows the name.
   ```
   sudo systemctl enable --now ssh
   sudo nmcli con add type ethernet ifname enP8p1s0 con-name direct ipv4.method manual ipv4.addresses 192.168.10.2/24
   sudo nmcli con up direct
   ```
   On the laptop, open Settings, Network & Internet, Ethernet, IP assignment, Edit, and choose Manual. Turn IPv4 on: address `192.168.10.1`, subnet mask `255.255.255.0`, gateway and DNS blank. Then check with `ping 192.168.10.2` and `ssh guitar@192.168.10.2`. Don't use `192.168.55.x`: that's the Jetson's USB link.
2. **First copy of the repo.** The Jetson can't clone from GitHub, so bundle the repo on the laptop and clone from the bundle. On the laptop, in the repo:
   ```
   git bundle create repo.bundle HEAD main
   scp repo.bundle guitar@192.168.10.2:~/
   ```
   On the Jetson:
   ```
   git clone ~/repo.bundle ~/CapstoneBioGuitar
   cd ~/CapstoneBioGuitar
   git config receive.denyCurrentBranch updateInstead
   ```
   Back on the laptop, add the Jetson as a remote so `git push jetson main` works from then on:
   ```
   git remote add jetson ssh://guitar@192.168.10.2/home/guitar/CapstoneBioGuitar
   ```
3. **Python packages.** `requirements-jetson.txt` has the servo libraries, OpenCV and MediaPipe (no PyBullet). Install them for the `guitar` user, not with sudo. After installing, `pip3 list | grep -i opencv` should show only `opencv-contrib-python`. If `opencv-python-headless` is there too, the camera works but `--show` prints "can't open a preview window here": uninstall every OpenCV package (`pip3 uninstall -y opencv-python opencv-python-headless opencv-contrib-python opencv-contrib-python-headless`) and reinstall only the `opencv-contrib-python` wheel with `pip3 install --user --no-deps <wheel>`. If the Jetson can get internet some other way (a phone's USB tethering, a hotspot), run `pip3 install --user -r requirements-jetson.txt`. Otherwise download the wheels on the laptop (match `--python-version` to `python3 --version` on the Jetson; JetPack 6 has 3.10):
   ```
   pip download -r requirements-jetson.txt -d jetson_wheels --platform manylinux2014_aarch64 --python-version 3.10 --only-binary=:all:
   scp -r jetson_wheels guitar@192.168.10.2:~/
   ```
   and install them on the Jetson:
   ```
   pip3 install --user --no-index --find-links ~/jetson_wheels -r ~/CapstoneBioGuitar/requirements-jetson.txt
   ```
   If `pip download` finds no wheel for a package, download that one again without `--platform` and `--only-binary`; pure-Python packages install from the source file.
4. **The `servo` shortcut, pins 15 and 31, and the boards.** Set up the shortcut (below). Run `sudo /opt/nvidia/jetson-io/jetson-io.py` and make sure pins 15 and 31 are plain GPIO, not PWM. `sudo i2cdetect -y -r 7` should show 40, 41 and 70.
5. **Calibration file.** Start from the committed values, then finish calibration (section 8):
   ```
   cp Hardware/calibration.example.json Hardware/calibration.json
   ```

### Jetson desktop on the laptop (remote desktop)

Remote desktop shows the Jetson's whole desktop in a window on the laptop, over the direct cable. Everything still runs on the Jetson: `run.py` opens the camera preview and the sim there, and you see both on the laptop. Unlike SSH window forwarding, the 3-D sim works this way, because it's drawn on the Jetson and only the picture of the screen is sent.

**Once, on the Jetson (monitor and keyboard):**
1. Open Settings, then Sharing. Turn on the switch at the top right of the window.
2. Open Remote Desktop and turn on **Remote Desktop** and **Remote Control**. Leave Enable Legacy VNC Protocol off.
3. Under Authentication, set User Name `guitar` and Password `password`. These are the remote desktop login, separate from the Jetson's own password.
4. Remote desktop shares the desktop that's already logged in, so turn on automatic login (Settings, Users, Unlock, Automatic Login) and stop the screen locking (Settings, Privacy, Screen: Blank Screen Delay Never, Automatic Screen Lock off). Otherwise the connection shows a lock screen or nothing after a reboot. From a terminal instead: put `AutomaticLoginEnable=true` and `AutomaticLogin=guitar` under `[daemon]` in `/etc/gdm3/custom.conf`, and run:
   ```
   gsettings set org.gnome.desktop.session idle-delay 0
   gsettings set org.gnome.desktop.screensaver lock-enabled false
   ```
5. **Make the login keyring's password blank.** Remote desktop keeps its login in the keyring, and after an automatic login the keyring stays locked, so Windows gets "The token supplied to the function is invalid". Passwords and Keys crashes on this Jetson, so start a new keyring instead. In a terminal on the Jetson's desktop (not over SSH):
   ```
   mv ~/.local/share/keyrings/login.keyring ~/.local/share/keyrings/login.keyring.old
   sudo reboot
   ```
   After the reboot, set the login again; when asked for a password for the new keyring, leave it **blank** and accept the warning:
   ```
   grdctl rdp set-credentials guitar password
   ```
6. Reboot once more and check that the login survived: `grdctl status --show-credentials` should still show `guitar` / `password`.
7. Keep SSH on as the way back in when remote desktop isn't working: Remote Login on in Sharing, or `sudo systemctl enable --now ssh`.

**Each time, on the laptop:**
1. Plug in the direct cable and check `ping 192.168.10.2`.
2. Open **Remote Desktop Connection** from the Start menu. Computer: `192.168.10.2`. Click Connect.
3. Windows fills in your own Microsoft account, which the Jetson won't accept. Click **More choices**, then **Use a different account**.
4. User name `guitar`, password `password`. Tick Remember me so next time skips this step. Click OK.
5. If Windows warns that the remote computer's identity can't be verified, click Yes. That's expected on the direct cable.
6. In a terminal on the remote desktop:
   ```
   cd ~/CapstoneBioGuitar
   python3 run.py mimic --dry-run --show --hand both --wrist   # tracking and sim only, nothing moves
   python3 run.py mimic --show --hand both --wrist             # the real hand too
   ```

- The sim needs PyBullet on the Jetson. The current Jetson has it; `requirements-jetson.txt` leaves it out, so a new Jetson won't. Check with `python3 -c "import pybullet"`. If it's missing, `run.py` says so and runs without the sim. There's no ready-made aarch64 wheel, so download the source on the laptop and build it on the Jetson (about 15 to 20 minutes):
  ```
  pip download pybullet --no-binary :all: --no-deps -d pybullet_src
  scp -r pybullet_src guitar@192.168.10.2:~/
  pip3 install --user ~/pybullet_src/pybullet-*.tar.gz      # on the Jetson
  ```
- The sim, the camera tracking and the servo loop all share the Jetson. The sim is its own process and the servos never wait for it, but if the preview's camera fps drops a lot, add `--no-sim`.
- With no monitor on the Jetson, remote desktop may show a black or tiny screen. An HDMI dummy plug fixes it.
- "The token supplied to the function is invalid": remote desktop couldn't read its login from the locked keyring. Check `grdctl status --show-credentials` on the Jetson's desktop and redo step 5 if the password isn't `password`.
- "Credentials did not work": Windows sent your own account, or a wrong one it remembered. Use More choices, or remove `TERMSRV/192.168.10.2` in Windows Credential Manager (Windows Credentials) and connect again.
- Keep someone at the E-stop for powered runs, the same as at the Jetson itself.

### On the Jetson (real hand)

- The Jetson has no internet. Code goes over the direct Ethernet cable (laptop `192.168.10.1`, Jetson `192.168.10.2`): commit on the laptop, then `git push jetson main`. Without the cable, use a git bundle: `git bundle create update.bundle main` on the laptop, then `git pull /path/to/update.bundle main` on the Jetson in `~/CapstoneBioGuitar`.
- Run `python3 run.py ...` **without** sudo. It asks for the password itself when it starts the servo part. The sim needs your desktop session, and sudo's Python can't see packages pip installed for your user (OpenCV, MediaPipe). `run.py` handles that.
- Scripts run on their own (not through `run.py`) need sudo for GPIO and I2C, plus your user's packages, so run them with the `servo` shortcut: `servo Hardware/...`. Plain `sudo python3` gives a missing-module error. Set the shortcut up once with
  `echo "alias servo='sudo DISPLAY=\$DISPLAY XAUTHORITY=\${XAUTHORITY:-\$HOME/.Xauthority} PYTHONPATH=\$(python3 -m site --user-site) python3'" >> ~/.bashrc && source ~/.bashrc`.
  The `DISPLAY` / `XAUTHORITY` part lets `--show` windows open under sudo.
- Packages: `requirements-jetson.txt` (step 3 above). `Jetson.GPIO` comes with JetPack, and `Hardware/i2c_bus.py` also works with Adafruit Blinka in place of `adafruit-extended-bus`.
- `Hardware/calibration.json` only exists on the Jetson, and git ignores it. After calibrating, copy it back over the example and commit it, so the measured values are saved in the repo:
  `scp guitar@192.168.10.2:~/CapstoneBioGuitar/Hardware/calibration.json Hardware/calibration.example.json` (on the laptop, in the repo).

### First run on the box

1. Box powered, E-stop released, press RESET so the SERVO LED is on.
2. Run a script. It starts with signals off and waits for Enter.
3. Press Enter. It writes the home pulses, then turns signals on.
4. Start slow: `python3 run.py song --wrist-only --speed 0.25`, then 0.5, then 1.0.

## 8. Bench procedures

Run these on the Jetson with the box powered. Each servo script starts with signals off and waits for Enter. Values you save go to `Hardware/calibration.json`.

### Wiring check (no calibration needed)

```
sudo i2cdetect -y -r 7                            # should show 40 and 41
servo Hardware/test_one_servo.py 0         # wiggle channel 0 on the 8.4 V board (thumb_base)
servo Hardware/test_one_servo.py 3 lv      # channel 3 on the 5 V board
python3 Hardware/calibrate_servo.py --show        # what's calibrated and what's missing
```

### Wrist

1. Calibrate each wrist servo by name: `dev_pos`, `dev_neg`, `flex_pos`, `flex_neg`.
   ```
   servo Hardware/calibrate_servo.py dev_pos
   ```
   Type a pulse (`4900`) or a nudge (`+50`, `-50`) to move it, then save:
   - `neutral`: wrist held straight, string just snug.
   - `sign +` or `sign -`: nudge the pulse up. If the string winds in, it's `sign +`.
   - `min` / `max`: jog both ways until the wrist reaches its stop, then back off a little.
2. Check the pulse scale: neutral +/- 1638 should turn the horn about 45 deg. If it doesn't, change `PULSE_PER_SERVO_DEG` in `servo_output.py`.
3. Move all four together with `servo Hardware/calibrate_servo.py --wrist` and try `25 0`, `-20 0`, `0 20`, `25 20`.
   - `25 0` should be the up-strum ready side. If it goes the other way, run `python3 Hardware/calibrate_servo.py --swap dev` (no hardware needed). Do the same for flex with `0 20` and `--swap flex`. This is also where the sign of `LIFT_FLEX_DEG` gets checked.
   - At every pose all four strings should be snug: none slack, none straining. If they aren't, the geometry constants in `wrist_kinematics.py` don't match the real part.
4. `python3 Hardware/servo_output.py --dry-run` prints the schedule, the pulses and how fast each servo has to turn.
5. First real strum: `python3 run.py song --wrist-only --speed 0.25`, then 0.5, then 1.0.

### Fingers

HPS-0618SG servos, two per finger, all on the 8.4 V board (`hv`, 0x40): `<finger>_base` is the bottom segment and `<finger>_tip` is the upper two segments together. A finger pose is ten curl numbers, 0 = straight, 1 = fully curled.

To tie the strings, `servo Hardware/calibrate_servo.py --open` holds every finger servo at its open pulse.

1. Calibrate each servo. Its channel is already saved; a number after the name changes it (`index_tip 3`):
   ```
   servo Hardware/calibrate_servo.py index_tip
   ```
   Save `open` (finger straight, tendon just snug) and `closed` (curled to where it first touches its stop). Don't go past the stop: a stalled HPS-0618SG pulls up to 2.5 A and gets hot fast.
2. Try one finger before the rest are wired: `servo Hardware/hand_console.py --no-wrist --fingers index`, then `finger index 0.3`, `servo index_tip 0.6`, `open`.
3. Tune the pick grip in the console: `pose pick_ready`, adjust with `finger` / `servo`, then `save pick_ready`. Same for `pick_grip`.
4. `python3 run.py pick` runs the hand-off grip and holds it so you can tug the pick. `--holder` takes it from the holder; tune `HOLDER_WRIST` / `LIFT_WRIST` in `pick_up.py` for that.

### Left hand

`left_hand.py` is last year's `Imitation/mimic.py` moved onto this box. MediaPipe tracks a hand on the camera and each left-hand finger servo follows the bend of that finger. The servos are on the 5 V board (`lv`, 0x41):

| Servo | Channel | Pulses |
| --- | --- | --- |
| `left_index` | 1 | 1638 open, 7000 closed (last year's values) |
| `left_middle` | 2 | same |
| `left_ring` | 3 | same |
| `left_pinky` | 4 | same |

Channels 10-13 on this board are the wrist; 0, 5-9, 14 and 15 are spare.

```
python3 Hardware/left_hand.py --dry-run --show            # check the tracking, nothing moves
servo Hardware/calibrate_servo.py left_index       # jog, then: open / closed
servo Hardware/calibrate_servo.py left_index 12    # move a servo to another channel
python3 run.py mimic --hand left                          # the real thing
```

- Needs `opencv-contrib-python` and `mediapipe` on the Jetson.
- `--dry-run --show` prints each finger's bend (its three joints added up). If a finger doesn't reach fully open or closed, measure your hand with `python3 Hardware/hand_mimic.py --calibrate --hand left --show`, or change its `straight_deg` / `curled_deg` in `LEFT_TRACKING` in `Sim/hand_tracking.py`.
- Power: the five left-hand servos stall at about 2 A each, so together they can use the whole 10 A 5 V supply. That's fine for the mimic in free air on its own, but not for fretting under load or running alongside the wrist.

### Camera mimic, both hands

`hand_mimic.py` reads whether the hand in view is a left or a right and drives the matching robot hand; the other one eases open. The sim shows both hands' commands. The wrist is driven only with `--wrist` (from the tilt of your right hand).

- **Wrist goes the wrong way?** Check the servos first: `servo Hardware/calibrate_servo.py --wrist`, then `25 0` should go toward the pinky. If it does, the servos are right and the camera reads the tilt the other way: flip `WRIST_DEV_SIGN` in `Sim/hand_tracking.py`. If it goes toward the thumb, the servos are swapped: `calibrate_servo.py --swap dev`. Changing both at once undoes the fix.
- **`[camera] can't open camera 1`:** a USB webcam shows up as `/dev/video0` and `/dev/video1`, and only `video0` is the picture. Leave `--camera` off.

```
python3 Hardware/hand_mimic.py --dry-run --show   # check the tracking, nothing moves
python3 run.py mimic --dry-run --show             # the same with the sim alongside
python3 run.py mimic --show                       # the real thing
python3 run.py mimic --hand right                 # always the right hand
```

- First run: hold up your right hand and read the top line of the preview. If it says `left`, add `--swap-hands`.
- The hands swap only after the camera has seen the other hand for `SWITCH_HOLD_S` (0.5 s), so a one-frame mislabel does nothing.
- Right hand: each finger's `base` servo follows the knuckle and its `tip` servo follows the two outer joints added together (`RIGHT_TRACKING` in `Sim/hand_tracking.py`). To split them differently, change `_split()`. The `straight_deg` / `curled_deg` values in the code are guesses. Measure your own once with `python3 Hardware/hand_mimic.py --calibrate --hand both --show`: hold your hand open, then in a fist (nothing moves). It saves the ranges and your wrist's straight pose (needed for `--wrist`) to `calibration.json`.
- Servos with no calibration stay still. The right-hand fingers have starting pulses, so they move, but over an unmeasured range until they are calibrated.
- Tracking uses MediaPipe's 3-D landmarks in video mode, so a finger reads the same however the hand is turned, and `CurlFilter` takes out the jitter. If the hand lags real moves, raise `BETA`; if it still shakes when you hold still, lower `MIN_CUTOFF_HZ` (both in `Sim/hand_tracking.py`).
- Sim only, no Jetson: `python Sim/mvmnt_demo.py --mimic --show`. It tracks both hands at once (your left drives the left sim hand, your right the right one), fingers only. A hand out of view for a second goes back to rest. For the wrist too, see "Camera and sim together" in section 7.

### Live sim alongside the hand

`run.py` does this for you when it runs on the Jetson's own desktop, on a monitor or on the laptop through remote desktop (section 7). To run the sim on the laptop itself instead, start it there first:

```
python Sim/mvmnt_demo.py --follow
```

then run the hand on the Jetson with the laptop's address, e.g. `--sim-ip 192.168.10.1` on the direct cable. `hand_console.py`, `pick_up.py`, `hand_mimic.py` and `Testing/hand_tests.py` take the same flag.

- If nothing moves, allow UDP port 5005 through the laptop's firewall.
- If the sim runs ahead of the hand, measure the gap on video and add it with `--delay`, e.g. `--delay 0.08`.
- The sim shows commands, not measured motion. If the sim stops, the servos keep running.

### CDR tests

`Testing/hand_tests.py` runs the three CDR tests and writes a CSV per run to `Testing/results/` (dry runs go to `Testing/results/dry_run/`):

```
servo Testing/hand_tests.py finger index --segment tip
servo Testing/hand_tests.py strum --cycles 20 --bpm 60
servo Testing/hand_tests.py tempo --bpm 60        # then 90, then 120
python3 Testing/hand_tests.py report Testing/results/<file>.csv --fps 120
```

Rehearse any of them with `--dry-run` (add `--auto` to have the prompts answered with made-up values, marked SIMULATED). `--no-fingers` runs the strum and tempo tests with the pick clamped or taped instead of gripped.

For the tempo test, film at 120 fps, type the frame number of each first string contact into the CSV's `contact_frame` column, then run `report`. One frame at 120 fps is 8.3 ms, which is 3.3% of the 0.25 s interval at 120 BPM, so a reading one frame off already uses a third of the +/-10% allowance.

### Strike speed

The CDR's 0.12 s strike needs about 563 deg/s at the wrist. With the measured geometry each servo turns about 2.68 deg per wrist degree, so the servos need about 1500 deg/s. The MG995 at 6 V is rated about 375 deg/s (0.16 s/60 deg, no load), so the wrist will lag the strike by about 4 times; `--speed 0.25` brings it down to about the rating. The dry run prints these numbers. To close the gap:

- a larger spool radius, or tie points closer to the ball (more speed, less torque)
- a longer `STRIKE_DURATION` or a narrower sweep in `Sim/strum_motion.py`

## 9. Common tasks

| I want to... | Do this |
| --- | --- |
| Play the song | `python3 run.py song` (add `--wrist-only` if the fingers aren't ready, `--speed 0.25` to slow down) |
| Play another song | `--song "name.gp5" --track 0`, or change `SONG_FILE` in `Sim/strum_motion.py`. Songs go in `Parsing/inputs/`. |
| Strum a pattern | `python3 run.py song --pattern D-DU-UDU --bpm 90 --repeats 4` |
| Change strum speed, angles or lift | Edit the constants under `# motion` in `Sim/strum_motion.py` |
| Move the hand by hand | `python3 run.py console`, then type `help` |
| Calibrate a servo | `servo Hardware/calibrate_servo.py <servo name>` (see section 8) |
| See what's calibrated | `python3 Hardware/calibrate_servo.py --show` |
| Tune and save a finger pose | In the console: `pose pick_ready`, adjust with `finger` / `servo`, then `save pick_ready` |
| Grip the pick | `python3 run.py pick` |
| Run a CDR test | `python3 run.py test strum --cycles 20`; results go to `Testing/results/` |
| Camera mimic | `python3 run.py mimic --show` (try `--dry-run --show` first) |
| Camera mimic in the sim only | `python Sim/mvmnt_demo.py --mimic --show` (fingers only) |
| Camera mimic in the sim, wrist too | `python Sim/mvmnt_demo.py --follow`, then in a second window `python Hardware/hand_mimic.py --dry-run --show --hand both --wrist --sim-ip 127.0.0.1` |
| Re-place the hands or guitar in the sim | Set `CALIBRATE = True` in `Sim/mvmnt_demo.py`, drag the sliders (`left_*` move the left hand), then paste the printed values |

## 10. Safety rules built into the code

Every script that moves a servo goes through `power_box.py` and follows the same rules:

- Signals start OFF and stay off until someone presses Enter.
- A script refuses to start if the E-stop is pressed, if a servo it needs has no calibration, or if two servos share a channel.
- The hand eases into its first pose instead of jumping.
- An E-stop press drops to SAFE immediately, and the script never re-arms by itself.
- Every exit (finish, Ctrl-C, crash, SIGTERM) turns signals off and stops the PWM.
- Only run one servo script at a time. Both boards share the OE pin, so the first script to exit turns off signals for the other.

The hardware E-stop is the real safety. The software is designed so it never works against it.

## 11. Conventions for changes

- Put shared motion logic in `Sim/strum_motion.py` or `Sim/finger_motion.py`, with no PyBullet or hardware imports.
- New hardware scripts should use `Hand` from `hand_control.py` so they get the safety rules automatically.
- Every script should support `--dry-run` and `--sim-ip`.
- Measured values go in `calibration.json` through the tools, not as edits to the tables in the code. The tables in the code are defaults.
- Keep comments short. Use plain `#` comments above functions.
- Check with a dry run before running on the box.

## 12. Known issues and placeholders

Placeholders that still need real values:

- Wrist `reel_in_sign` and pulse limits (the neutral pulses are saved).
- Finger open and closed pulses (the channels are saved; the pulses are 1700 / 8100 starting values).
- `PICK_READY` / `PICK_GRIP` poses, `HOLDER_WRIST` / `LIFT_WRIST` in `pick_up.py`, and the camera-mimic angle ranges (measure them with `hand_mimic.py --calibrate`).
- The sign of `LIFT_FLEX_DEG` (whether it lifts away from the guitar or into it) has to be checked on the real wrist.

Things to know:

- **Strike speed.** A 0.12 s strike needs a peak of about 1500 deg/s at the servo with the measured geometry. The MG995 is rated around 375 deg/s, so expect the wrist to lag the strike. The dry run prints these numbers. Options are under "Strike speed" in section 8.
- **`CALIBRATE = True` is set in `Sim/mvmnt_demo.py`.** Running the sim by itself opens the placement sliders instead of playing the song. Set it to `False` to see the strum on its own. `--follow` mode (what `run.py` uses) isn't affected.
- **The sim shows commands, not measured motion.** The servos don't report their position. The sim's hand is the generic MPL model, not our design, and its wrist stops around -15 deg when -20 is commanded.
- **Old scripts write straight to 0x40** with no safety layer: `Imitation/mimic.py`, `Imitation/servozero.py`, `Testing/servotest.py`, `Testing/servozero.py`. On the current box 0x40 is the right-hand finger board, so don't run them there. `Testing/servotest.py` also imports `ExtendedI2C` from the wrong package and probably won't run as written.
- **`test_one_servo_armed.py`'s top comment labels the boards the old way round.** Use the table in section 5.
- **`requirements.txt` lists `guitarpro`,** but the strum code's own error message tells you to `pip install PyGuitarPro`. If the song doesn't load, the code falls back to constant-tempo strumming and prints why.
- **`Sim/play_learned_strum*.py` copy the sim placement with the hand anchor 2 cm higher** (z = 0.557 vs 0.537 in `mvmnt_demo.py`).

## 13. More information

- `docs/Code_and_Operations_Guide.pdf`: the full guide: the electrical box, Jetson setup, calibration, running the hand, the CDR tests and a reference for every module. Its LaTeX source is next to it.
- `docs/CDR.pdf` and `docs/Fall26_SOP.pdf`: design review and safety procedure.
- `credits.md`: outside projects and models we used.
