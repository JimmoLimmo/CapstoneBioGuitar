# Hardware output (right hand)

Quick map of the scripts:

| Script | What it's for |
| --- | --- |
| `hand_console.py` | Type commands to move fingers and wrist. Use this to program the finger poses. |
| `servo_output.py` | Play the song (wrist strum; fingers grip the pick unless `--wrist-only`). |
| `pick_up.py` | Get the pick into the hand (hand-off, or from a holder). |
| `Testing/hand_tests.py` | The CDR test plan: fingertip accuracy, strum repeatability, tempo. |
| `calibrate_servo.py`, `test_one_servo.py` | Bench tools for one servo / the wrist. |
| `hand_control.py`, `fingers.py`, `power_box.py`, `wrist_kinematics.py` | Shared code the scripts above use. |

From the repo root, `python3 run.py [console|song|pick|test] ...` opens the sim
and runs the hand together (run it without sudo; it asks for the password).

Every script takes `--dry-run` (no hardware needed) and `--sim-ip` (show the
commands live in the sim). The wrist section below was written first; the
finger sections are near the end.

Turns the strum choreography into real PCA9685 servo commands for the
marionette wrist (ball-and-socket joint, four Hiwonder HPS-0618SG servos in two opposing pairs, on the 8.4 V board). 
Tested only against a fake PCA9685 so far; **no real servo has run this yet.**

## The chain

```
Sim/strum_motion.py      wrist_at(t)             -> deviation deg, flexion deg
Hardware/wrist_kinematics.py  servo_angles()     -> 4 spool angles (dev_pos, dev_neg, flex_pos, flex_neg)
Hardware/servo_output.py      per-servo calibration -> 4 PCA9685 pulses, 50 Hz
```

`strum_motion.py` is shared with the PyBullet sim, so timing changes made
there show up in both.

## Live sim alongside the hand

The sim can show the same commands the servos receive: the wrist angles, and
the finger curls when the fingers are in use. Start the sim first, on the
computer with the screen:

    python Sim/mvmnt_demo.py --follow

Then start the hand on the Jetson, using the sim computer's IP address:

    python Hardware/servo_output.py --wrist-only --sim-ip <sim computer IP>

To try it with no hardware, run both on one computer:

    python Hardware/servo_output.py --dry-run --sim-ip 127.0.0.1
    python Hardware/hand_console.py --dry-run --sim-ip 127.0.0.1

`hand_console.py`, `pick_up.py` and `Testing/hand_tests.py` take the same
`--sim-ip`. The Jetson and the sim computer have to be on one network for this
(a direct Ethernet cable between them works; the Jetson doesn't need internet).

If nothing moves, allow UDP port 5005 through the sim computer's firewall.
If the sim runs ahead of the hand, measure the gap on video and add it with
`--delay`, for example `--delay 0.08`.

The sim shows commands, not measured motion, because the servos report no
position. It also uses the generic MPL hand, so it stops at that model's wrist
limit (about -15 deg) when the command goes to -20 deg. If the sim stops, the
servos keep running.

## Power box (REV 3 wiring sheets 0-5)

Servo power is switched in hardware only: RESET latches the relay coils
(D1 seal-in from K1 NO), and the E-stop (NC1), TH1, F3 or MAIN drop them.
IN1/IN2 are tied to GND; there are no Jetson wires to the relay board.
`power_box.py` handles the Jetson side for every script:

| Jetson pin | Use |
| --- | --- |
| 31 (out) | OE of both PCA9685. HIGH = all servo signals off (R5 10k pull-up holds it HIGH at boot / crash). |
| 15 (in) | E-stop NC2: LOW = released, HIGH = pressed or cable cut (R4 10k pull-up to pin 17). |
| 1 / 3 / 5 / 9 | I2C bus 7 to both PCA9685 (VCC, SDA, SCL, GND). |

Every script that moves a servo:
1. starts with OE HIGH (signals off);
2. refuses to start if the E-stop is pressed;
3. waits for you to press Enter, writes the home pulses, then sets OE LOW;
4. servo power itself comes from the box: press RESET if the SERVO LED is off;
5. drops to SAFE (OE HIGH, PWM off) the instant the E-stop is pressed, and
   never re-arms by itself; start the script again;
6. sets OE HIGH and turns the PWM off on every exit: end of run, Ctrl-C,
   SIGTERM or a crash.

Run them with `sudo` on the Jetson (GPIO and I2C access). Needs
`Jetson.GPIO`, `adafruit-circuitpython-pca9685` and `adafruit-extended-bus`.

Quick wiring check, no calibration needed:

    sudo python3 Hardware/test_one_servo.py 13        # channel 13, 8.4 V board
    sudo python3 Hardware/test_one_servo.py 3 lv      # channel 3, 5 V board

Boot / shutdown: R5 holds OE HIGH whenever the Jetson isn't driving pin 31,
so servo signals stay off at boot, after a script exits and with the Jetson off.

## Why there's a kinematics step

Each wrist axis is two servos pulling strings on opposite sides of the ball.
The simple plan was "the pulling servo gets pulse P, the opposite one gets the
mirror of P." On a ball joint the two sides don't change length by the same
amount, and moving one axis also changes the other pair's string lengths.
With the placeholder geometry, a mirror would leave about 2.3 mm of slack at
the +25 deg strum extreme (about 13 deg of spool rotation). The wrist would
rattle at the end of every stroke.

`wrist_kinematics.py` works out each of the four string lengths from the
wrist pose, then turns the change into spool rotation. Run it on its own for
a table:

    python Hardware/wrist_kinematics.py

Because the geometry is handled in code, calibration gets simpler: each servo
only needs its neutral pulse and its direction. You no longer measure
joint-angle/pulse pairs through the linkage.

## Still placeholders

1. **Geometry**: the constants at the top of `wrist_kinematics.py` (arm
   radius/drop, anchor radius/drop, spool radius). Take them from the 301/302/303
   CAD. These set how far each servo turns per degree of wrist movement.
2. **Wrist calibration**: `WRIST_SERVOS` in `servo_output.py` (board, channel,
   neutral_pulse, reel_in_sign, min/max_pulse for each of the 4 servos).
3. **Finger calibration**: `FINGER_SERVOS` in `fingers.py` (channel, open_pulse,
   closed_pulse for each of the 10 servos). See "Fingers" below.
4. **Finger poses**: `PICK_READY` / `PICK_GRIP` in `Sim/finger_motion.py` are
   starting guesses. Tune them with `hand_console.py`.
5. **Pick holder**: `HOLDER_WRIST` / `LIFT_WRIST` in `pick_up.py`, only needed
   for `pick_up.py --holder`.

## Bench procedure (wrist)

1. Wire the four wrist servos and write down which channel goes to which side.
2. For each servo: `sudo python3 Hardware/calibrate_servo.py <channel>` (add `lv` for the 5 V board)
   - With the wrist held straight, find the pulse where its string is just snug.
     That's `neutral_pulse`.
   - Nudge the pulse up. If the string winds in, `reel_in_sign = +1`, else `-1`.
   - Jog both ways until the wrist reaches its stop, then back off a little.
     Those are `min_pulse` / `max_pulse`.
3. Check `PULSE_PER_SERVO_DEG`: neutral +/- 1638 should turn the horn about 45 deg.
4. `sudo python3 Hardware/calibrate_servo.py --wrist`, then try `25 0`, `-20 0`, `0 20`, `25 20`.
   - `25 0` should be the up-strum ready side. If it goes the other way, swap
     the `dev_pos`/`dev_neg` channel numbers. Do the same for flex with `0 20`.
     This is also where `LIFT_FLEX_DEG`'s sign finally gets checked for real.
   - At every pose all four strings should be snug: none slack, none straining.
     If they aren't, the geometry constants don't match the real part.
5. `python Hardware/servo_output.py --dry-run` prints the schedule, pulses, and
   how fast each servo has to turn.
6. `sudo python3 Hardware/servo_output.py --wrist-only --speed 0.25` is the first
   real strum at quarter speed. Then 0.5, then 1.0.

## Fingers

Ten MG90S servos, two per finger: `<finger>_base` (bottom segment) and
`<finger>_tip` (upper two segments together). A finger pose is ten "curl"
numbers, 0 = straight, 1 = fully curled. Poses live in `Sim/finger_motion.py`,
so the sim and the real hand use the same ones.

1. Calibrate each servo: `sudo python3 Hardware/calibrate_servo.py <channel> lv`
   - `open_pulse`: finger straight, tendon just snug.
   - `closed_pulse`: segment curled to where it first touches its stop. Don't go
     past it; a stalled MG90S gets hot fast.
   Put the channel and both pulses in `FINGER_SERVOS` in `fingers.py`.
2. Try one finger before the rest are wired:
   `sudo python3 Hardware/hand_console.py --no-wrist --fingers index`
   then `finger index 0.3`, `servo index_tip 0.6`, `open`.
3. With all ten done, tune the pick grip in the console: `pose pick_ready`,
   adjust with `finger` / `servo`, then `save pick_ready` prints the line to
   paste into `finger_motion.py`. Same for `pick_grip`.
4. `sudo python3 Hardware/pick_up.py` runs the hand-off grip and holds it so
   you can tug the pick.

Addresses (as wired on the box): the 8.4 V wrist board ("hv") is 0x40, the
5 V finger board ("lv") is 0x41. The REV 3 sheets show them the other way round. Check with `sudo i2cdetect -y -r 7`.

## Tests

`Testing/hand_tests.py` runs the three CDR tests and writes a CSV per run to
`Testing/results/` (dry runs go to `Testing/results/dry_run/`):

    sudo python3 Testing/hand_tests.py finger index --segment tip
    sudo python3 Testing/hand_tests.py strum --cycles 20 --bpm 60
    sudo python3 Testing/hand_tests.py tempo --bpm 60        # then 90, then 120
    python Testing/hand_tests.py report Testing/results/<file>.csv --fps 120

Rehearse any of them with `--dry-run` (add `--auto` to have the prompts
answered with made-up values, marked SIMULATED). `--no-fingers` runs the strum
and tempo tests with the pick clamped or taped instead of gripped.

For the tempo test, film at 120 fps, then type the frame number of each first
string contact into the CSV's `contact_frame` column and run `report`. One
frame at 120 fps is 8.3 ms, which is 3.3% of the 0.25 s interval at 120 BPM, so
a reading that is one frame off already uses a third of the +/-10% allowance.

## Safety behaviour

- Refuses to move anything if any wrist servo is missing its channel or neutral
  pulse, or if two servos share a channel.
- Eases from straight into the first ready pose, so there's no jump at start-up.
- At the end of a run it eases back to neutral, then turns the servo signals
  off (OE HIGH) and releases the PWM outputs. On Ctrl-C or an E-stop it turns
  the signals off straight away. `--hold` was removed: signals must go off on
  every exit.
- The hardware E-stop is still the real safety. None of the above replaces it.

## Strike speed

The CDR's 0.12 s strike needs about 563 deg/s at the wrist. With the
placeholder geometry each servo turns about 1.5 deg per wrist degree, and the
payout side peaks around 900 deg/s. The HPS-0618SG is rated at about
1000 deg/s (0.06 s/60 deg at 8.4 V, no load), so that fits with little margin;
under load it may still lag a little. The dry run prints the real numbers once
the geometry is filled in. If it lags:
- a larger spool radius, or tie points closer to the ball (more speed, less torque)
- a longer `STRIKE_DURATION` or a narrower sweep in `strum_motion.py`
