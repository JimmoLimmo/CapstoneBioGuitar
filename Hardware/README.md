# Hardware output (right hand)

Turns the sim's strum choreography into real PCA9685 servo commands. This
is new as of 2026-09-17 and untested against real hardware -- treat it as
scaffolding, not a finished driver.

## How it connects to the sim

`Sim/mvmnt_demo.py` used to define the whole strum (song schedule, timing,
wrist angles) inline. That logic is now factored into `Sim/strum_motion.py`,
which has **no pybullet import** -- it's pure math plus the Guitar Pro
file reading. Both consumers import it and call the same
`wrist_at(schedule, now)` every tick:

- `Sim/mvmnt_demo.py` feeds `(yaw_deg, flex_deg)` into PyBullet's
  `setJointMotorControlArray`.
- `Hardware/servo_output.py` feeds the same two numbers into a PCA9685 over
  I2C -- the same driver/bus/address last year's left hand used
  (`Imitation/mimic.py`, `Imitation/servozero.py`).

If you change strum timing, edit `strum_motion.py` once and both the sim and
the real hand pick it up.

## What's still a placeholder

`servo_output.py`'s `CHANNEL_MAP` has `channel: None` and made-up
`deg`/`pulse` bounds for every joint. None of it is real yet because:

1. We don't know the final channel assignment (which PCA9685 channel drives
   wrist yaw, which drives wrist flex, which drives the grip actuator) --
   that's an electrical/fab call, not a software one.
2. Even once wired, the `deg -> pulse` numbers have to be measured on the
   actual servo + linkage geometry, not assumed from a datasheet -- last
   year's own two scripts (`mimic.py` vs `servozero.py`) used different pulse
   ranges for the same kind of servo, which is exactly the kind of thing that
   goes wrong if you copy someone else's calibration constants.

## Filling it in

1. Wire up one servo (start with wrist yaw / `wrist_UDEV`) to a known PCA9685
   channel and confirm the I2C bus number (`I2C_BUS = 7` carries over from
   last year -- reconfirm once the second Jetson's wiring is settled).
2. Run `python Hardware/calibrate_servo.py <channel>` and jog it by hand to
   the two positions that matter (e.g. the down-strum limit and the up-strum
   limit). Note the pulse value at each.
3. Put the channel number and those two `(deg, pulse)` pairs into
   `CHANNEL_MAP` in `servo_output.py`.
4. Repeat for `wrist_FLEX` and `grip` (and `wrist_PRO` later, if/when it's
   actuated -- for now it's just held at `WRIST_PRO_BASE_DEG`).
5. Test with `python Hardware/servo_output.py --dry-run` first (prints the
   commanded angles on a schedule, touches no hardware, needs no adafruit
   libs) to sanity-check timing, then drop `--dry-run` once the channel map
   is real.

## Known open questions carried over from the sim work

- **`LIFT_FLEX_DEG`'s sign** is still an untested guess in the sim (see
  `project_bioguitar_history` notes) -- confirm the direction on real
  hardware before trusting the flex calibration bounds above.
- **Control rate**: `CONTROL_HZ = 50` in `servo_output.py` is a starting
  guess, not tied to the sim's 240 Hz physics step (which doesn't apply
  here -- there's no physics to keep in sync with, just a servo that holds
  position between updates). Raise or lower it if the real strum looks
  stepped or if I2C traffic becomes a bottleneck.
- **`STRIKE_DURATION = 0.12s`**: whether a real servo + linkage can actually
  complete that sweep in 120ms hasn't been bench-tested. If the physical
  strike lags behind the commanded angle, that's a motion-profile problem
  (raise `STRIKE_DURATION` in `strum_motion.py`) or a torque/gearing problem,
  not a bug in this script.
