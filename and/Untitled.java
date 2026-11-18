# Bio-Inspired Guitarist

This README covers what the code does, its architecture, how to run it, and what is still unfinished. Step-by-step bench procedures (calibrating servos, wiring checks, finger tuning) are in section 8.

Last updated: November 2026 (Fall 2026 team).

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
| Wrist kinematics (wrist angles to 4 servo angles) | Working, but the geometry numbers are placeholders |
| Safety layer (E-stop, OE pin, arming) | Working on the box |
| Wrist calibration | Channels assigned (lv board, ch 10-13); neutral pulses not measured yet |
| Finger calibration | No channels or pulses filled in yet |
| Pick grip poses | Starting guesses, to be tuned on the real hand |
| Live sim alongside the hand | Working (UDP) |
| Camera mimic (left and right) | Working in dry run; left hand has last year's pulses, right hand is uncalibrated |
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
| `Sim/mvmnt_demo.py` | The PyBullet sim, with both hands. Plays a song by itself, or with `--follow` shows the commands the real hands are getting, or with `--mimic` follows hands on the camera. |
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

Each wrist axis is two servos pulling strings on opposite sides of a ball joint. Mirroring one servo's pulse onto the other leaves a string slack or over-tight, because the two sides don't change length by the same amount. `wrist_kinematics.py` calculates each string's real length instead. Moving one axis also changes the other pair's string lengths. With the placeholder geometry, a plain mirror would leave about 2.3 mm of slack at the +25 deg strum extreme, and the wrist would rattle at the end of every stroke. Because the geometry is handled in code, each servo only needs its neutral pulse and its direction. Run `python Hardware/wrist_kinematics.py` for a table.

## 5. Hardware the code expects

### Electrical box (REV 3)

| Item | Detail |
| --- | --- |
| I2C | Bus 7 (Jetson header pins 3 and 5). Check with `sudo i2cdetect -y -r 7`. |
| `hv` board, 0x40 | 8.4 V PCA9685: right-hand finger servos only |
| `lv` board, 0x41 | 5 V PCA9685 (changing to 6.4 V): wrist on ch 10-13, left hand on ch 0-4 |
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

`calibrate_servo.py --wrist` confirms the directions. If a pair turns out backwards, swap its two channel numbers.

### Finger servos

There are ten names, `<finger>_base` and `<finger>_tip`, for thumb, index, middle, ring and pinky, all on the `hv` board. Channels aren't assigned yet.

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
| `calibration.json` | Bench values saved by `calibrate_servo.py` and `hand_console.py`. Lives in `Hardware/` on the Jetson. Git ignores it. |

## 7. Getting started

### On a laptop (sim and dry runs, no hardware)

1. Use Python 3.10, 3.11 or 3.12. PyBullet often has no Windows install for the newest Python.
2. Install what the sim needs: `pip install pybullet numpy PyGuitarPro`. The full `requirements.txt` also lists the servo libraries, which often fail to install on Windows. You don't need them for dry runs.
3. From the repo root:

```
python run.py song --dry-run            # sim opens, the song plays on screen
python run.py console --dry-run         # type commands, watch the sim
python Hardware/servo_output.py --dry-run   # print the schedule and servo demand only, no sim
python Sim/mvmnt_demo.py --mimic --show     # both sim hands follow your hands on the webcam (needs opencv-python, mediapipe)
```

`run.py` opens the sim, waits for it, then starts the hand script with `--sim-ip 127.0.0.1`. Add `--no-sim` to skip the sim.

`SIMULATION_SETUP.md` in the Capstone folder has more install troubleshooting.

### On the Jetson (real hand)

- The Jetson has no internet. Code goes over the direct Ethernet cable (laptop `192.168.10.1`, Jetson `192.168.10.2`): commit on the laptop, then `git push jetson main`. Without the cable, use a git bundle: `git bundle create update.bundle main` on the laptop, then `git pull /path/to/update.bundle main` on the Jetson in `~/CapstoneBioGuitar`.
- Run `python3 run.py ...` **without** sudo. It asks for the password itself when it starts the servo part. The sim needs your desktop session, and sudo's Python can't see packages pip installed for your user (OpenCV, MediaPipe). `run.py` handles that.
- Scripts run on their own (not through `run.py`) need sudo for GPIO and I2C, plus your user's packages, so run them with the `servo` shortcut: `servo Hardware/...`. Plain `sudo python3` gives a missing-module error. Set the shortcut up once with
  `echo "alias servo='sudo DISPLAY=\$DISPLAY XAUTHORITY=\${XAUTHORITY:-\$HOME/.Xauthority} PYTHONPATH=\$(python3 -m site --user-site) python3'" >> ~/.bashrc && source ~/.bashrc`.
  The `DISPLAY` / `XAUTHORITY` part lets `--show` windows open under sudo.
- Servo packages: `Jetson.GPIO`, `adafruit-circuitpython-pca9685`, and either `adafruit-extended-bus` or Adafruit Blinka. Camera scripts also need `opencv-python` and `mediapipe`.
- `Hardware/calibration.json` only exists on the Jetson. Back it up; it's the only copy of the measured values.

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
servo Hardware/test_one_servo.py 13        # wiggle channel 13 on the 8.4 V board
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
   - `25 0` should be the up-strum ready side. If it goes the other way, swap the `dev_pos` / `dev_neg` channel numbers in `WRIST_SERVOS`. Do the same for flex with `0 20`. This is also where the sign of `LIFT_FLEX_DEG` gets checked.
   - At every pose all four strings should be snug: none slack, none straining. If they aren't, the geometry constants in `wrist_kinematics.py` don't match the real part.
4. `python3 Hardware/servo_output.py --dry-run` prints the schedule, the pulses and how fast each servo has to turn.
5. First real strum: `python3 run.py song --wrist-only --speed 0.25`, then 0.5, then 1.0.

### Fingers

HPS-0618SG servos, two per finger, all on the 8.4 V board (`hv`, 0x40): `<finger>_base` is the bottom segment and `<finger>_tip` is the upper two segments together. A finger pose is ten curl numbers, 0 = straight, 1 = fully curled.

1. Calibrate each servo. The number after the name sets its channel:
   ```
   servo Hardware/calibrate_servo.py index_tip 3
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

Channels 10-13 on this board are the wrist; 5-9, 14 and 15 are spare.

```
python3 Hardware/left_hand.py --dry-run --show            # check the tracking, nothing moves
servo Hardware/calibrate_servo.py left_index       # jog, then: open / closed
servo Hardware/calibrate_servo.py left_index 12    # move a servo to another channel
python3 run.py mimic --hand left                          # the real thing
```

- Needs `opencv-python` and `mediapipe` on the Jetson.
- `--dry-run --show` prints each finger's bend (its three joints added up). If a finger doesn't reach fully open or closed, change its `straight_deg` / `curled_deg` in `LEFT_TRACKING` in `Sim/hand_tracking.py`.
- Power: the five left-hand servos stall at about 2 A each, so together they can use the whole 10 A 5 V supply. That's fine for the mimic in free air on its own, but not for fretting under load or running alongside the wrist.

### Camera mimic, both hands

`hand_mimic.py` reads whether the hand in view is a left or a right and drives the matching robot hand; the other one eases open. The sim shows both hands' commands. The wrist is not driven.

```
python3 Hardware/hand_mimic.py --dry-run --show   # check the tracking, nothing moves
python3 run.py mimic --dry-run --show             # the same with the sim alongside
python3 run.py mimic --show                       # the real thing
python3 run.py mimic --hand right                 # always the right hand
```

- First run: hold up your right hand and read the top line of the preview. If it says `left`, add `--swap-hands`.
- The hands swap only after the camera has seen the other hand for `SWITCH_HOLD_S` (0.5 s), so a one-frame mislabel does nothing.
- Right hand: each finger's `base` servo follows the knuckle and its `tip` servo follows the two outer joints added together (`RIGHT_TRACKING` in `Sim/hand_tracking.py`). To split them differently, change `_split()`. The `straight_deg` / `curled_deg` values are guesses: read the real angles off `--dry-run --show` with the finger straight and curled.
- Servos with no calibration stay still, so until the right fingers are calibrated the right hand moves only in the sim.
- Tracking uses MediaPipe's 3-D landmarks in video mode, so a finger reads the same however the hand is turned, and `CurlFilter` takes out the jitter. If the hand lags real moves, raise `BETA`; if it still shakes when you hold still, lower `MIN_CUTOFF_HZ` (both in `Sim/hand_tracking.py`).
- Sim only, no Jetson: `python Sim/mvmnt_demo.py --mimic --show`. It tracks both hands at once (your left drives the left sim hand, your right the right one). A hand out of view for a second goes back to rest.

### Live sim alongside the hand

`run.py` does this for you when it runs on the Jetson's own desktop. To show the sim on the laptop instead, start it there first:

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

The CDR's 0.12 s strike needs about 563 deg/s at the wrist. With the placeholder geometry each servo turns about 1.5 deg per wrist degree, so the payout side peaks around 900 deg/s. The MG995 at 6 V is rated about 375 deg/s (0.16 s/60 deg, no load), so the wrist will lag the strike by about 2.4 times. The dry run prints the real numbers once the geometry is filled in. To close the gap:

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
| Camera mimic in the sim only | `python Sim/mvmnt_demo.py --mimic --show` |
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

- Wrist geometry constants at the top of `Hardware/wrist_kinematics.py` (take them from the 301/302/303 CAD).
- Wrist `neutral_pulse` values, and confirmation of `reel_in_sign` and the pulse limits.
- All finger channels and pulses in `fingers.py`.
- `PICK_READY` / `PICK_GRIP` poses, `HOLDER_WRIST` / `LIFT_WRIST` in `pick_up.py`, and the camera-mimic angle ranges.
- The sign of `LIFT_FLEX_DEG` (whether it lifts away from the guitar or into it) has to be checked on the real wrist.

Things to know:

- **Strike speed.** A 0.12 s strike needs a peak of about 900 deg/s at the servo with the current geometry. The MG995 is rated around 375 deg/s, so expect the wrist to lag the strike. The dry run prints these numbers. Options are under "Strike speed" in section 8.
- **`CALIBRATE = True` is set in `Sim/mvmnt_demo.py`.** Running the sim by itself opens the placement sliders instead of playing the song. Set it to `False` to see the strum on its own. `--follow` mode (what `run.py` uses) isn't affected.
- **The sim shows commands, not measured motion.** The servos don't report their position. The sim's hand is the generic MPL model, not our design, and its wrist stops around -15 deg when -20 is commanded.
- **Old scripts write straight to 0x40** with no safety layer: `Imitation/mimic.py`, `Imitation/servozero.py`, `Testing/servotest.py`, `Testing/servozero.py`. On the current box 0x40 is the right-hand finger board, so don't run them there. `Testing/servotest.py` also imports `ExtendedI2C` from the wrong package and probably won't run as written.
- **`test_one_servo_armed.py`'s top comment labels the boards the old way round.** Use the table in section 5.
- **`requirements.txt` lists `guitarpro`,** but the strum code's own error message tells you to `pip install PyGuitarPro`. If the song doesn't load, the code falls back to constant-tempo strumming and prints why.
- **`Sim/play_learned_strum*.py` copy the sim placement with the hand anchor 2 cm higher** (z = 0.557 vs 0.537 in `mvmnt_demo.py`).

## 13. More information

- `SIMULATION_SETUP.md` (Capstone folder): installing PyBullet and the separate MANO/Ursina sim.
- `CDR.pdf` and `Fall 26 SOP.pdf` (Capstone folder): design review and safety procedure.
- The Capstone project's `right-hand-electrical-box.md`: box layout and wiring notes.
- `credits.md`: outside projects and models we used.
