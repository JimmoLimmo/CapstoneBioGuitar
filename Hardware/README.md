# Hardware output (right hand)

Turns the strum choreography into real PCA9685 servo commands for the
marionette wrist in the CDR (ball-and-socket joint, four MG90S servos in two
opposing pairs). Tested only against a fake PCA9685 so far; **no real servo
has run this yet.**

## The chain

```
Sim/strum_motion.py      wrist_at(t)             -> deviation deg, flexion deg
Hardware/wrist_kinematics.py  servo_angles()     -> 4 spool angles (dev_pos, dev_neg, flex_pos, flex_neg)
Hardware/servo_output.py      per-servo calibration -> 4 PCA9685 pulses, 50 Hz
```

`strum_motion.py` is shared with the PyBullet sim, so timing changes made
there show up in both. (Note: `Sim/mvmnt_demo.py` currently carries its own
inline copy of this logic, with BPM 132 instead of 90. The motion math is
identical, but the two copies can drift apart.)

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
2. **Wrist calibration**: `WRIST_SERVOS` in `servo_output.py` (channel,
   neutral_pulse, reel_in_sign, min/max_pulse for each of the 4 servos).
3. **Grip**: `OTHER_CHANNELS["grip"]` is still one channel. It becomes the 10
   finger servos once those are wired.

## Bench procedure (wrist)

1. Wire the four wrist servos and write down which channel goes to which side.
2. For each servo: `python Hardware/calibrate_servo.py <channel>`
   - With the wrist held straight, find the pulse where its string is just snug.
     That's `neutral_pulse`.
   - Nudge the pulse up. If the string winds in, `reel_in_sign = +1`, else `-1`.
   - Jog both ways until the wrist reaches its stop, then back off a little.
     Those are `min_pulse` / `max_pulse`.
3. Check `PULSE_PER_SERVO_DEG`: neutral +/- 1638 should turn the horn about 45 deg.
4. `python Hardware/calibrate_servo.py --wrist`, then try `25 0`, `-20 0`, `0 20`, `25 20`.
   - `25 0` should be the up-strum ready side. If it goes the other way, swap
     the `dev_pos`/`dev_neg` channel numbers. Do the same for flex with `0 20`.
     This is also where `LIFT_FLEX_DEG`'s sign finally gets checked for real.
   - At every pose all four strings should be snug: none slack, none straining.
     If they aren't, the geometry constants don't match the real part.
5. `python Hardware/servo_output.py --dry-run` prints the schedule, pulses, and
   how fast each servo has to turn.
6. `python Hardware/servo_output.py --wrist-only --speed 0.25` is the first
   real strum at quarter speed. Then 0.5, then 1.0.

## Safety behaviour

- Refuses to move anything if any wrist servo is missing its channel or neutral
  pulse, or if two servos share a channel.
- Eases from straight into the first ready pose, so there's no jump at start-up.
- On any stop (end of song, Ctrl-C, exception), it eases back to neutral and
  releases the PWM outputs, so the servos go limp. Use `--hold` to keep holding
  instead.
- The hardware e-stop is still the real safety. None of the above replaces it.

## Open issue: strike speed

The CDR's 0.12 s strike needs about 563 deg/s at the wrist. With the
placeholder geometry each servo turns about 1.5 deg per wrist degree, and the
payout side peaks around 900 deg/s. The MG90S is rated at about 600 deg/s with
no load. The dry run prints the real numbers once the geometry is filled in.
Ways out, roughly in order:
- a larger spool radius, or tie points closer to the ball (more speed, less torque)
- a longer `STRIKE_DURATION` or a narrower sweep in `strum_motion.py`
- running the servos at 6 V (about 750 deg/s rated)
