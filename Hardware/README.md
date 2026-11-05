# Hardware output (right hand)

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

The sim can show the same wrist commands the servos receive. Start the sim
first, on the computer with the screen:

    python Sim/mvmnt_demo.py --follow

Then start the hand on the Jetson, using the sim computer's IP address:

    python Hardware/servo_output.py --wrist-only --sim-ip <sim computer IP>

To try it with no hardware, run both on one computer:

    python Hardware/servo_output.py --dry-run --sim-ip 127.0.0.1

If nothing moves, allow UDP port 5005 through the sim computer's firewall.
If the sim runs ahead of the hand, measure the gap on video and add it with
`--delay`, for example `--delay 0.08`.

The sim shows commands, not measured motion, because the servos report no
position. It also uses the generic MPL hand, so it stops at that model's wrist
limit (about -15 deg) when the command goes to -20 deg. If the sim stops, the
servos keep running.

## Power box (relays and E-stop)

Servo power comes from the electrical box, not straight from the supplies.
`power_box.py` handles it for every script:

| Jetson pin | Use |
| --- | --- |
| 7 (out) | relay IN1, K1 = 5 V servo power and master hold. Active LOW. |
| 11 (out) | relay IN2, K2 = 8.4 V servo power. Active LOW. |
| 15 (in) | E-stop: LOW = released, HIGH = pressed or cable cut |
| 3 / 5 | I2C bus 7: PCA9685 `lv` at 0x41 (5 V group), `hv` at 0x42 (8.4 V group, the wrist) |

Every script that moves a servo:
1. starts with pins 7 and 11 HIGH (relays off);
2. refuses to start if the E-stop is pressed;
3. waits for you to press Enter, writes the home pulses, then sets 7 and 11 LOW;
4. asks you to press the box's RESET button (servo power only comes on then);
5. drops to SAFE (relays off, PWM off) the instant the E-stop is pressed, and
   never re-arms by itself; start the script again;
6. turns the relays off on every exit: end of run, Ctrl-C, SIGTERM or a crash.

Run them with `sudo` on the Jetson (GPIO and I2C access). Needs
`Jetson.GPIO`, `adafruit-circuitpython-pca9685` and `adafruit-extended-bus`.

Quick wiring check, no calibration needed:

    sudo python3 Hardware/test_one_servo.py 13        # channel 13, 8.4 V board
    sudo python3 Hardware/test_one_servo.py 3 lv      # channel 3, 5 V board

Not done yet: the box's rules also ask for a boot-time service that holds
pins 7 and 11 HIGH before any script runs, and for the relays to be set off
before the Jetson shuts down. A service that holds the pins would block the
scripts from using them, so this needs either pull-up resistors on IN1/IN2
or one long-running process that owns the pins.

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
3. **Grip**: `OTHER_CHANNELS["grip"]` is still one channel. It becomes the 10
   finger servos once those are wired.

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

## Safety behaviour

- Refuses to move anything if any wrist servo is missing its channel or neutral
  pulse, or if two servos share a channel.
- Eases from straight into the first ready pose, so there's no jump at start-up.
- At the end of a run it eases back to neutral, then turns the relays off and
  releases the PWM outputs. On Ctrl-C or an E-stop it turns the relays off
  straight away. `--hold` was removed: the relays must go off on every exit.
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
