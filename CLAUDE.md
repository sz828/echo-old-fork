# Override_V1

Competition code for a VEX V5RC **Override** (2026-2027) robot. PROS kernel,
LemLib 0.5.6 for odometry and motion, plus a hand-written Monte Carlo
Localization filter that corrects odometry drift against the field.

## Build

Standard PROS project, target `v5`.

```
pros build          # compile
pros upload         # or: pros mu   (build + upload)
pros terminal       # view Logger output
```

The ARM toolchain is **not** available in cloud sessions, so changes cannot be
compiled there. Verify logic with a host-side harness (`g++`) where the code is
pure maths, and leave the real build to the user's machine.

On the user's machine the toolchain *is* installed, by the PROS VS Code
extension, and `make` works. One catch: it defaults to `C:\WINDOWS\` for
temporary files and dies with "Cannot create temporary file". Point `TMP` at a
writable directory first.

## Layout

| Path | Purpose |
| --- | --- |
| `include/robot.hpp` | **The one config file.** Ports, measurements, field map. |
| `src/robot.cpp` | Builds the chassis, odometry and MCL globals. |
| `include/auton.hpp` | Motion wrappers that apply the motion-chaining switch. |
| `include/selector.hpp`, `src/selector.cpp` | Auton selector and the routine table. |
| `include/mcl.hpp`, `src/mcl.cpp` | The particle filter. |
| `include/macros.hpp`, `src/macros.cpp` | The deposit macro's state machine. |
| `include/health.hpp`, `src/health.cpp` | Port and battery check behind the screen warning. |
| `include/tuning.hpp`, `src/tuning.cpp` | PID auto-tuner, run from the selector. |
| `src/main.cpp` | PROS entry points and the brain-screen readout. |
| `DRIVER_MACROS.md` | Further proposed macros. Design only; only deposit exists. |

`include/lemlib`, `include/pros`, `include/liblvgl`, `include/fmt` and
`firmware/` are vendored. Do not edit them.

## Conventions

The first four are the usual cause of a filter that will not converge. The
last two are about ownership: who sets a value, and who draws to the screen.

- **Field frame.** Centre is `(0, 0)`, walls at `±72` inches.
- **Heading.** Degrees, `0` points along `+Y`, increasing **clockwise**. This is
  LemLib's convention and MCL matches it exactly.
- **Sensor mounting** (`mcl::SensorConfig`): `offsetX` is inches right of the
  tracking centre, `offsetY` is inches forward, `angle` is degrees clockwise
  from robot forward, so `0` forward, `90` right, `180` back, `270` left.
- **Ports.** A negative motor port means that motor is reversed. `0` means not
  installed.
- **Starting pose belongs to the selector table**, not the routine. A routine
  must never call `chassis.setPose()`: `selector::runSelected()` has already
  applied the pose from the table and re-seeded MCL by the time it runs. A
  routine that sets its own pose makes the table a lie.
- **Only `screenTask` may touch `pros::lcd`.** Not "one owner at a time" --
  one task, full stop. LLEMU sits on LVGL, which is not thread-safe, and that
  covers `read_buttons()` as well as the drawing. Two tasks inside it corrupt
  its heap and the brain data-aborts in `lv_malloc_core`, which is a crash in
  whichever task was unlucky rather than at the call that caused it. The
  selector draws by being ticked from `screenTask`; anything else that wants
  the screen does the same. `initialize()` is the one exception, and only
  because it runs before the task exists.

## The robot

Fast, light cycler. Not a pushing robot.

**Drivetrain.** Four 11 W motors, two per side, 3.25" omnis all round, no
traction wheels. 600 rpm cartridges through 36T to 60T, so 360 rpm at the
wheel, about 5.1 ft/s. Track width 10.43 in (26.5 cm).

**Manipulators.**

| Subsystem | Motors | Notes |
| --- | --- | --- |
| Intake | 1 × 11 W | Floor pickup. Coast when idle. |
| Lift | 2 × 11 W | **DR4B.** Keeps the carriage level, so objects stay upright. Brake hold, it backdrives under load. |
| Grabber | 1 × 5.5 W | Flex-wheel roller, not a claw. Grips by compliance. |
| Grabber pivot | 1 × 5.5 W | Wrist. Rotates the object off onto a Goal. |

Power budget: `4×11 + 11 + 2×11 + 2×5.5 = 88 W`, exactly at the legal limit.
**There are no watts spare.** Any new mechanism must be passive or pneumatic.

**Toggles are handled by a passive mechanism**, deliberately, for that reason.

**Sensing.** IMU plus four distance sensors. No tracking wheels, so lateral and
turn distance come from motor encoders, which slip on an all-omni drive. That
slip is precisely what MCL exists to correct.

## Game facts that drive the code

Field is 12 ft square, so the `±72` inch walls in `mcl.hpp` are correct.

- Nine Goals inside the perimeter: four neutral Short Goals (one per quadrant),
  one neutral Tall Goal at the midfield centre, and four Alliance Goals (one per
  quadrant, two red and two blue).
- Four Toggles, one at the centre of each field wall. Set when fully seated
  against its mounts with no robot touching it.
- Scoring: an Alliance-colour Pin in a Goal is 5 points. A yellow Pin is 10
  points to whichever Alliance owns that quadrant's Toggle.
- Endgame: 8 points per robot inside the Midfield boundary, plus yellow points
  on the centre Goal.
- **Possession is capped at one Pin and one Cup at a time.** This is why there
  are two independent single-object manipulators and no magazine or indexer.
  Do not add one.
- Robot fits 18" × 18" × 18" at the start, may expand to a 24" × 18" footprint
  in one declared direction, and is limited to 88 W of motors.

## Driver control

**Tank.** Left stick Y drives the left side, right stick Y the right, through
`chassis.tank()` in `opcontrol()`. Both stick Y axes are therefore spoken for,
so everything else binds to a button.

| Button | Subsystem |
| --- | --- |
| `L1` / `L2` | Intake **and** grabber together, in / out |
| `R1` / `R2` | Lift up / down |
| `X` / `B` | Angler (the grabber pivot) up / down |
| `A` | Run the deposit macro, or abort one that is running |

Everything but `A` is **hold-to-run**, in `handleManipulators()` in `main.cpp`.
All four of those bindings also cancel a running deposit macro, because the
macro owns the lift, grabber and angler while it runs.
A toggle that is out of sync with what the driver believes is on is worse than
a button that has to be held, and none of these mechanisms has a sensor to
resync from. Releasing a button calls `brake()`, which applies the brake mode
set in `initRobot()`: hold everywhere except the intake, which coasts.

It must be `brake()` and never `move(0)`. `move()` is voltage control, and zero
volts freewheels no matter what the brake mode says -- the mode is only
consulted on a zero *velocity* command, which is what `brake()` sends. The two
look identical on the bench and differ exactly where it matters: the wrist
drifts back down and the DR4B sinks under its own weight. The same applies to
every stage stop in `macros.cpp`.

The mode itself is set once, and a smart motor that loses its cable comes back
with the VEXos default, which is coast. For the angler that means the wrist
quietly dropping whatever it was holding; for the DR4B, which backdrives, it
means the lift sinking under its own weight as soon as the driver lets go. So
`enforceHoldBrakes()` in `robot.cpp` re-checks both and rewrites whichever is
wrong, the lift motor by motor. `screenTask` calls it on the same 250 ms tick
as `health::poll()`, because that task is the only one running in every
competition state. The port signs in `ports::LIFT_MOTORS` need no such
treatment: PROS keeps reversal in the `MotorGroup` rather than on the brain.

The intake and grabber share `L1`/`L2` and run in the same direction, because
both pull the object the same way through the robot. This is a driver
preference, not a constraint -- possession is capped at one Pin *and* one Cup,
so if loading them independently turns out to matter, split the bindings.

Motor speeds live in `speeds::` in `robot.hpp`. The grabber and angler are 5.5 W
motors run short of full voltage on purpose: they stall far sooner than an 11 W
motor and there is no watt budget to replace one.

### The deposit macro

`A` runs it. Nine stages: raise until the grabber's distance sensor loses the
face of the stack, keep rising for `CLEAR_DWELL_MS`, drive forward
`APPROACH_IN`, lower until the lift stalls against the stack, eject, lift off
what was just placed by `LIFT_OFF_RISE`, settle back down `LIFT_OFF_SETTLE`,
reverse `BACKUP_IN`, drop the lift to the height it started from.

The settle keeps the back-off from being taken at full lift height. It is the
one stage that gives up quietly: a settle that hangs hands on to the back-off
anyway rather than failing, because by then the object is placed, released and
already clear, and nothing is gained by staying parked over the stack.

Touchdown is detected by the lift going still, not by a fixed drop and not by a
current spike. A DR4B backdrives, so gravity does most of the work on the way
down and the current stays low even once the object is supported; what changes
is that the lift stops moving. That makes the stage self-adjusting to any stack
height. Descending `LOWER_MAX_TRAVEL` without ever stalling fails the macro
rather than ejecting anyway -- if nothing stopped the lift, nothing is under the
object, and dropping it from height is how a stack gets knocked over.

`macros::update()` is called once per `opcontrol()` loop and each stage is one
step of a state machine, so **the sequence never blocks the control loop**. It
does take the drivetrain, but only for the approach and the back-off, which is
what `macros::ownsDrive()` reports. `opcontrol()` must not call `chassis.tank()`
while that is true: the drive legs are asynchronous LemLib motions in their own
task, and a `tank()` every 20 ms overwrites them.

Touching `L1`/`L2`/`R1`/`R2`, pressing `A` again, or pushing either drive stick
past `deposit::STICK_DEADBAND` cancels it and hands the lift, grabber and
drivetrain straight back.

The angler is taken too, and `X`/`B` cancel along with the rest. Pressing `A`
swings the wrist to `deposit::ANGLER_TARGET` (-180 degrees) and holds it there
for the whole sequence, then leaves it there -- the driver brings it back with
`X`/`B`. It is driven on voltage and capped at `speeds::ANGLER` rather than
handed to `move_absolute()`, because the angler is a 5.5 W motor and the
motor's own position PID would drive it at full voltage into the end of travel.

`ANGLER_TARGET` is measured from wherever the wrist sat **at boot**, since that
is what PROS calls zero. Booting the robot with the wrist already swung out
therefore makes the target wrong by however far out it was.

The two drive legs are relative nudges, aimed at a field point computed from
the pose at the moment the leg starts. A correction mid-leg would move the
field under that target, so **MCL is paused across the legs** via
`localization.pause()` and resumed the moment the back-off finishes. They do
lean on the lateral PID, which is still on LemLib defaults.

`mcl::MCL::pause()` is not `stop()`. `stop()` blocks for two update periods
waiting for the filter task to leave its loop, which would stall the control
loop that called it, and `start()` re-seeds the cloud from scratch. `pause()`
suppresses only the write to the chassis pose; particles keep tracking
odometry, so nothing has to re-converge afterwards. `releaseDrive()` in
`macros.cpp` calls `resume()` on every path out of the macro, so an abort
partway through a leg cannot leave the filter paused.

`macros::runDeposit()` is the blocking form, for auton routines.

The macro refuses to start if `ports::DISTANCE_GRABBER` is not reporting a
distance sensor -- two short rumbles -- because a blind lift driving up into a
pile is how mechanisms get bent. Every number it uses is in `deposit::` in
`robot.hpp`, and all of them are placeholders.

`DRIVER_MACROS.md` holds the rest of the macro proposals. None of those exist.

## Pre-match check

`health::poll()` compares every port in `robot.hpp` against what the brain
actually reports on it, and reads the battery. A missing port, a swapped cable
(the port reports the wrong device type) or a battery below
`checks::MIN_BATTERY_PERCENT` puts a warning on **line 6 of the brain screen and
line 2 of the controller**, and rumbles once per new problem.

It is drawn in both screen views, including over the selector, because the
minutes before a match are exactly when it wants to be seen. `screenTask` owns
that line; `selector::draw()` deliberately stops at line 5.

There is no motor diagnostics screen. This replaced it.

## Autonomous selector

`selector::runSelected()` is the whole of `autonomous()`. Routines live in the
`ROUTINES` table in `src/selector.cpp`. Adding one is two steps: write the
function, add a row. Nothing else is registered anywhere.

```cpp
{"Red left", Alliance::RED, {-48, -60, 0}, redLeft},
//  name      alliance      start pose     function
```

- **Selection** is the brain screen left/right buttons, mirrored to the
  controller D-pad. Buttons are polled with edge detection rather than
  registered as callbacks, so a press cannot land underneath a redraw.
- **Ticked by `screenTask`**, not by the competition hooks, which are empty.
  That task runs in every competition state, so selection is live from boot
  until `release()` takes the screen back, with or without a field controller
  attached. Driving it from `competition_initialize()` and `disabled()` instead
  is what caused the LVGL crash above: those are separate tasks.
- **Persistence.** The index is written to `/usd/auton.txt` and survives a
  reboot mid-event. Every SD failure is non-fatal: no card means index 0, which
  is "Do nothing".
- **Alliance** is a field on each routine, not a separate switch. Override is
  not colour-symmetric -- Alliance Goals belong to a colour and a yellow Pin
  scores to whoever owns that quadrant's Toggle -- so a red routine run on blue
  scores for the opposition. Read it back with `selector::alliance()`.

## MCL notes

Each 50 ms cycle runs predict, sense, estimate, correct, resample. It corrects
**position only**. Heading stays with the IMU, which beats anything four beams
can infer, so leave `correctHeading` false.

The sensor model knows about the nine Goals via `dimensions::fieldObstacles()`.
Without them every beam landing on a Goal would be scored as though it had
found a distant wall, dragging the pose outward. Beams that hit a Goal are
weighted with the wider `obstacleSigma`, because near a curved edge a small pose
error swings the predicted range a long way.

`beamFloor` is the uniform term of the beam model. It stops one blocked beam,
usually another robot, from annihilating a particle the other three agree with.

Safety rails, all deliberate: readings gated on confidence and a 2 to 55 inch
range, no correction until the cloud converges to within 6 inches, only 25% of
the disagreement applied per cycle, and a hard 4 inch cap per cycle.

## Outstanding measurements

These are placeholders in the code. The filter should not be trusted until they
are real numbers.

- [ ] **Distance sensor offsets** in `dimensions::distanceSensors()`. All four
      are a placeholder 6.5 in.
- [ ] **Goal coordinates** in `dimensions::fieldObstacles()`. Currently guessed
      on the 24 in tile grid. A Goal in the wrong place is worse than one left
      out, so `return {}` until they are measured.
- [ ] **Lateral and angular PID** in `pidGains::` in `robot.hpp`. Still LemLib
      starting values, and the angular loop in particular wants a look since the
      track width correction narrowed the robot by roughly 9%. The "Tune: turn"
      and "Tune: drive" routines will find these for you; the numbers still have
      to be pasted back by hand.
- [x] **Manipulator gearsets** in `gearsets::` in `robot.hpp`. All four
      confirmed green. This matters to the deposit macro beyond speed and
      torque: `lift.get_position()` reports degrees scaled by the configured
      cartridge, so every lift distance in `deposit::` is only meaningful while
      this stays green.
- [ ] **Routine starting poses** in the `ROUTINES` table in `selector.cpp`. All
      placeholders. MCL is seeded from them, so a wrong pose here puts the
      particle cloud in the wrong half of the field before the robot has moved.
- [x] **Grabber distance sensor port**, `ports::DISTANCE_GRABBER`. Port 11,
      mounted on the grabber looking forward at the stack.
- [ ] **Deposit macro constants** in `deposit::` in `robot.hpp`. Every one is a
      guess. `CLEAR_DISTANCE` matters most: it has to sit above whatever the
      sensor reads at loading height, or the macro thinks it has already topped
      the stack and deposits on the spot. After that come `STALL_VELOCITY` and
      `LOWER_MIN_TRAVEL`, which decide whether the lower stage sees a touchdown
      at all: too high a stall velocity and it stops in mid-air, too low and it
      keeps pushing after the object has landed.
