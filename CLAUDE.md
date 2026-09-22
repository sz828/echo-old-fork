# Echo

Competition code for a VEX V5RC **Override** (2026-2027) robot.

This is a fork of **2654E Echo**, whose code was written for High Stakes
(2024-2025), being ported onto a new robot. That history is visible everywhere in
the names: `goalClamp`, `WALL_STAKE_PRIME_HEIGHT`, `getRing()`, the top intake's
ring-colour ejection, the ladder-hang sequences. **Those names do not map onto
Override scoring elements.** When you read a symbol, assume it means what it
meant in High Stakes until you have checked otherwise.

What has actually been ported so far is the **hardware layer**: ports, the
subsystem list, and the driver bindings for the lift, intake/grabber and grabber
pivot. Everything above that — the autonomous routines, the paths, the top
intake's colour sorting, the hang — is still last season's robot playing last
season's game. See [Ported, and not](#ported-and-not).

The framework is WPILib-style commands (`include/command/`, vendored from
`libcommand`), plus Eigen, a units library, and a Monte Carlo particle filter for
localization.

`main` is the pristine upstream clone. All of our work is on `test`.

## Build

Standard PROS project, target `v5`.

```
pros build          # compile
pros upload         # or: pros mu   (build + upload)
pros terminal       # view output
```

The ARM toolchain is **not** available in cloud sessions, so changes cannot be
compiled there. Verify logic with a host-side harness (`g++`) where the code is
pure maths, and leave the real build to the user's machine.

On the user's machine the toolchain *is* installed, by the PROS VS Code
extension, and `make` works. One catch: it defaults to `C:\WINDOWS\` for
temporary files and dies with "Cannot create temporary file". Point `TMP` at a
writable directory first.

## Layout

Almost everything is header-only; `src/` holds seven small translation units, and
`src/main.cpp` is the only one worth reading.

| Path | Purpose |
| --- | --- |
| `include/config.h` | **The numbers file.** `CONFIG::` — geometry, gear ratios, lift heights, PID gains, drivetrain feedforward, distance sensor offsets. |
| `include/subsystems/subsystems.h` | **The wiring file.** Ports, subsystem construction, every controller binding, every named command. Read this first. |
| `include/subsystems/lift.h` | `LiftSubsystem` — DR4B, PID on position, stall-based zeroing. |
| `include/subsystems/bottomIntake.h` | `MotorSubsystem` — a generic percent-voltage motor group. Used for the intake/grabber group *and* the grabber pivot, despite the filename. |
| `include/subsystems/topIntake.h` | `TopIntakeSubsystem` — last season's hooked chain intake, with an optical sensor for ring colour. Still built; see below. |
| `include/subsystems/drivetrain.h` | `DrivetrainSubsystem` — tank drive, IMU, odometry, feedforward, sysid recording, PTO and winch for the hang, and it owns the particle filter. The biggest file in the repo. |
| `include/subsystems/solenoidSubsystem.h` | `SolenoidSubsystem` — one or more ADI digital outs driven together. |
| `include/command/` | The command framework. Vendored; do not edit. |
| `include/commands/` | Robot-specific commands: `rotate.h`, `driveMove.h`, `ramsete.h`, `driveToGoal.h`, `ltvUnicycleController.h`, and the per-mechanism ones under `lift/` and `intake/`. |
| `include/autonomous/` | The routine table (`autonCommands.h`) and the routines themselves. |
| `include/localization/` | The particle filter, and the sensor models that feed it. |
| `include/motionProfiling/` | Bezier paths and the profiler that turns them into velocity setpoints. |
| `include/velocityProfile/` | Trapezoidal profiles. |
| `include/sysid/` | Least-squares system identification, used by `characterizeLinear`/`characterizeAngular`. |
| `include/telemetry/` | `TELEMETRY`, a serial writer on port 0 at 921600 baud. |
| `include/auton.h`, `include/autonomous/autons.h` | The `Auton` and `Alliance` enums, the `AUTON` define, the `ALLIANCE` global. |
| `src/main.cpp` | PROS entry points and the two background tasks. |
| `static/` | Path JSONs, compiled into the binary as PROS assets. |

`include/Eigen`, `include/pros`, `include/json`, `include/units`,
`include/unsupported`, `include/command`, `firmware/` and `libraries/` are
vendored. Do not edit them.

`include/motionProfiling/path.h` is a zero-byte file that several headers
include. That is upstream's doing, not a deletion.

## How the framework works

If you have used WPILib this is familiar; if not, read this before touching
`subsystems.h`.

- A **`Subsystem`** owns hardware and has a `periodic()` that runs every frame.
- A **`Command`** declares which subsystems it requires. The scheduler guarantees
  one command per subsystem at a time; scheduling a new command cancels whatever
  held the requirement.
- Each subsystem is registered with a **default command** that runs whenever
  nothing else requires it: `CommandScheduler::registerSubsystem(sub, default)`
  at the bottom of `subsystemInit()`. That is where "what happens when the driver
  lets go" is defined — not in an `else` branch somewhere.
- `CommandScheduler::run()` is called from a **10 ms task** started in
  `initialize()`. Nothing in a command may block; a command that needs to wait
  returns `false` from `isFinished()` instead.
- **Triggers are polled, not callbacks.** `initializeController()` builds them
  with `->andOther()`, `->negate()`, `->whileTrue()`, `->onTrue()`.
- Commands compose: `->withTimeout()`, `->until()`, `->race()`, `->with()`,
  `->andThen()`, `->repeatedly()`, `Sequence`, `ParallelCommandGroup`,
  `ParallelRaceGroup`, `ScheduleCommand`.
- `PathCommands::registerCommand("name", command)` maps a string to a command so
  a path can fire it by name mid-motion. `initializePathCommands()` holds the
  table.

`subsystemInit()` runs `initializeCommands()`, then `initializeController()`,
then `initializePathCommands()`, in that order, because the later two reference
the `Command*` globals the first one fills in.

### Composed triggers are polled during autonomous

There are two event loops. `CommandScheduler::run()` polls `eventLoop`
unconditionally and `teleopEventLoop` only when the robot is neither autonomous
nor disabled.

`CommandController::getTrigger()` puts a button on the **teleop** loop. But
`andOther()`, `orOther()` and `negate()` each return `new Trigger(condition)`
using the single-argument constructor, which takes the **general** loop. So:

- `primary.getTrigger(DIGITAL_B)->whileTrue(...)` — teleop only, as you would
  expect.
- `primary.getTrigger(DIGITAL_R1)->andOther(...)->whileTrue(...)` — polled in
  autonomous too.

Every shoulder binding and the `Y` binding are composed, so **holding a shoulder
button during autonomous drives the lift or the intake** and cancels whatever the
routine had scheduled on it. `CommandScheduler::schedule()` does bail out when
`pros::competition::is_disabled()`, so the disabled period is covered; the
autonomous period is not.

## Conventions

The first three are the usual cause of a filter that will not converge. The last
two are about ownership: who sets a value, and who draws to the screen.

- **Units.** Everything public is a unit-typed quantity (`5_in`, `90_deg`,
  `200_ms`) from `include/units`. Internally the filter and every Eigen vector
  are in **metres and radians** — `.getValue()` gives you the SI number,
  `.Convert(degree)` the one you meant. Mixing the two silently is the easiest
  way to break this code.
- **Field frame.** Centre is `(0, 0)`. The walls in `localization/distance.h` are
  at `±1.78308` m, i.e. `±70.2` in, which is the interior wall-to-wall
  measurement of the 12 ft field.
- **Heading.** `0` points along `+X` and increases **counter-clockwise** — that
  is what `Distance::p()` assumes, and the filter is fed `-imu.get_rotation()` to
  get there from the IMU's clockwise-positive degrees. `Eigen::Vector3f` poses
  are `(x, y, theta)` throughout.
- **Sensor mounting.** A distance sensor offset is an `Eigen::Vector3f` of
  `(forward, left, angle)` in the robot frame, built in `CONFIG::` from unit
  literals: `DISTANCE_LEFT_OFFSET(4.2_in, 5.0_in, 90_deg)` is 4.2 in forward of
  centre, 5 in to the left, facing 90 degrees counter-clockwise from forward.
- **Ports.** A negative motor port means that motor is reversed. Ports are
  written inline in `subsystemInit()`; the comment block above it lists them and
  says where they came from.
- **Nothing but `initialize()` may touch `pros::lcd`.** LLEMU sits on LVGL, which
  is not thread-safe, and that covers `read_buttons()` as well as the drawing.
  Two tasks inside it corrupt its heap and the brain data-aborts in
  `lv_malloc_core`, which is a crash in whichever task was unlucky rather than at
  the call that caused it. Today `screen_update_loop` in `main.cpp` is an empty
  loop with its body commented out, so the rule costs nothing; if the screen
  comes back, it gets **one** owner task and everything that wants to draw is
  ticked from it.

It must be `brake()` and never `move(0)` — see `MotorSubsystem::hold()`.
`move_voltage()` is voltage control, and zero volts freewheels no matter what the
brake mode says; the mode is only consulted on a zero *velocity* command, which
is what `brake()` sends. The two look identical on the bench and differ exactly
where it matters: the wrist drifts back down and the DR4B sinks under its own
weight. The lift avoids the problem a third way, by holding a PID target rather
than a voltage — `holdPositionCommand()` latches the angle it was released at.

**Cartridges are not set everywhere.** `DrivetrainSubsystem` calls `set_gearing`
on both sides and `TopIntakeSubsystem` on its motor; the lift, the intake/grabber
group and the grabber pivot never do, so they inherit whatever the brain has
persisted for that port, and a motor that has lost its cable comes back on the
VEXos default.
This matters beyond speed and torque: `lift.get_position()` reports degrees
scaled by the configured cartridge, so every angle in `CONFIG::` is only
meaningful while that stays put.

## The robot

Fast, light cycler. Not a pushing robot.

**Drivetrain.** Four 11 W motors, two per side, blue (600 rpm) cartridges, omnis
all round, no traction wheels.

| Subsystem | Motors | Ports | Notes |
| --- | --- | --- | --- |
| Left drive | 2 × 11 W | `-2, -7` | |
| Right drive | 2 × 11 W | `4, 6` | |
| Lift | 2 × 11 W | `13, -14` | **DR4B.** Keeps the carriage level, so objects stay upright. Brake hold, it backdrives under load. |
| Intake + grabber | 1 × 11 W + 1 × 5.5 W | `3, 20` | One `MotorGroup`, so every intake command drives both. Coast when idle. |
| Grabber pivot | 1 × 5.5 W | `12` | Wrist. Rotates the object off onto a Goal. Brake hold. |
| Top intake | 1 × 11 W | `5` | Last season's. Still constructed. |

Sensors: IMU on `8`; four distance sensors, front `18`, left `10`, right `17`,
back `16`; rotation sensors on `9` (odometry) and `19` (winch, on the PTO);
optical on `21`. ADI: `'a'`/`'d'` hang, `'c'` PTO, `'e'` clamp.

The smart ports come from TheLib, as the comment above `subsystemInit()` records.
TheLib defines no port for the top intake, the odometry rotation sensor or the
winch, so those were put on ports it leaves free.

Power budget: `4×11 + 11 + 2×11 + 2×5.5 = 88 W`, exactly at the legal limit.
**There are no watts spare** — and that budget leaves out the top intake on port
5, which is an eighth 11 W motor and the tenth motor overall. With it the robot
draws 99 W against an 88 W limit, so it is **not currently legal**. Either the
top intake comes off or something else does.

**Toggles are handled by a passive mechanism**, deliberately, for that reason.

The grabber and pivot are 5.5 W motors. They stall far sooner than an 11 W motor
and there is no watt budget to replace one, but the bindings hand them
`pctCommand(±1.0)`, i.e. the full 12 V. Capping them is a one-line change and
probably wants making.

No tracking wheels for lateral or turn distance, so slip on an all-omni drive is
precisely what the particle filter exists to correct.

## Game facts that drive the code

Field is 12 ft square.

- Nine Goals inside the perimeter: four neutral Short Goals (one per quadrant),
  one neutral Tall Goal at the midfield centre, and four Alliance Goals (one per
  quadrant, two red and two blue).
- Four Toggles, one at the centre of each field wall. Set when fully seated
  against its mounts with no robot touching it.
- Scoring: an Alliance-colour Pin in a Goal is 5 points. A yellow Pin is 10
  points to whichever Alliance owns that quadrant's Toggle.
- Endgame: 8 points per robot inside the Midfield boundary, plus yellow points on
  the centre Goal.
- **Possession is capped at one Pin and one Cup at a time.** This is why there are
  two independent single-object manipulators and no magazine or indexer. Do not
  add one.
- Robot fits 18" × 18" × 18" at the start, may expand to a 24" × 18" footprint in
  one declared direction, and is limited to 88 W of motors.

## Driver control

**Tank**, through `DrivetrainSubsystem::tank(primary)`, registered as the
drivetrain's default command: left stick Y drives the left side, right stick Y the
right. Both stick Y axes are therefore spoken for, so everything else binds to a
button. Bindings are built in `initializeController()`.

| Button | Effect |
| --- | --- |
| `L1` / `L2` | Intake **and** grabber together, in / out |
| `R1` / `R2` | Lift up / down, full 12 V either way |
| `B` / `X` | Grabber pivot up / down |
| `Y` | Lift to `DESCORE_HEIGHT`; *or*, once `hangReleased`, run the hang sequence |
| `DOWN` | Repeating corner-clear sequence — **drives the robot** |
| `LEFT` / `UP` | Angular / linear sysid characterization — **drives the robot** through a fixed voltage script |
| `RIGHT` | Clamp solenoid. `whileFalse`: commanded high on release, low while held, so its resting state is high. Which of those is "clamped" depends on how the cylinder is plumbed — check the robot, not the code. |
| `A` | Held for 200 ms, flips `ALLIANCE` to the opposing colour |
| `L1`+`L2`+`R1`+`R2` | Hang release. `onTrue`, and it latches `hangReleased`, so it fires once and stays fired. |

Everything but `A`, `RIGHT` and the hang-release combo is **hold-to-run**. A
toggle that is out of sync with what the driver believes is on is worse than a
button that has to be held, and none of these mechanisms has a sensor to resync
from.

Two of these want a second look:

- **`A` is an alliance toggle on a face button**, and `ALLIANCE` feeds the top
  intake's colour ejection and every `flip` argument in autonomous. It takes a
  200 ms hold, so a tap is harmless, but a lean on the controller between matches
  is not. It belongs on a partner-controller combo.
- **`LEFT` and `UP` run sysid.** They drive the robot at up to full voltage on a
  timed script with no regard for where it is. Fine on blocks, not in a match.

The shoulder bindings each carry `->andOther(...->negate())` for the other three
shoulders, so that the four-shoulder hang-release combo does not also drive the
lift and intake on its way through. `negatedHang` keeps them all off while a hang
command is running.

Releasing a button falls back to the subsystem's default command: the intake
stops (`stopIntake()`, a 0 V command on a coast group), the pivot brakes where it
was left (`holdCommand()`), the lift holds the angle it was released at
(`holdPositionCommand()`), and both solenoids go low.

The intake and grabber share `L1`/`L2` and run in the same direction, because both
pull the object the same way through the robot. They are literally one
`MotorGroup` (`{3, 20}`), so this is not a binding that can be split without
splitting the subsystem first. Possession is capped at one Pin *and* one Cup, so
if loading them independently turns out to matter, that is the change to make.

### The lift

`LiftSubsystem` is the most finished subsystem, and the one to copy from.

- Position comes from `motor.get_position(0) / CONFIG::LIFT_RATIO`, in
  revolutions, converted to an `Angle`. `LIFT_RATIO` is `18/6`.
- `periodic()` runs `pid.update()` on the position **unless** a raw voltage has
  been set; `setVoltage()` latches that voltage, `setTarget()` clears it and
  returns to closed loop. Gains: `PID(1.2, 0.0, 3.0, 0.2, 1.0)`.
- Stall detection is current-based: any frame under 1000 mA resets `lastFree`,
  and `stalled(duration)` reports how long it has been drawing more than that.
- `zero()` uses it — drive down at -20% until stalled for 300 ms (2 s timeout),
  ease off, then `tare_position()`. That is the only way the lift learns where
  zero is, so **every angle in the code is relative to wherever it was tared**,
  which at boot is wherever the lift happened to be sitting. Nothing in the
  driver bindings calls `zero()` any more; it survives only as the `resetLB` path
  command.
- `positionCommand(angle, threshold)` finishes inside `threshold`; a threshold of
  `0.0` means it never finishes, so it holds until something else cancels it. Most
  call sites pass `0.0` deliberately.
- `TrapLiftPosition` in `commands/lift/trapPosition.h` is the profiled
  alternative, feeding the PID a trapezoidal setpoint instead of a step.

## Autonomous

There is **no run-time selector**. Two separate switches:

- `AUTON` is a **compile-time** `#define` in `include/auton.h`, currently
  `N_1_6`. The values are in `autonomous/autons.h`: `N_1_6`, `N_1_6P`, `N_6`,
  `P_4`, `P_1_3`, `SKILLS`, `NONE`. Changing routines means rebuilding and
  re-uploading.
- `ALLIANCE` is a mutable global (`inline auto ALLIANCE = RED`), changed at run
  time by the `A` binding. `OPPONENTS` is a macro derived from it.

`AutonomousCommands::getAuton()` is a `switch` over `AUTON` that returns one
`Command*`, built once in `initialize()`. `autonomous()` schedules it.
`opcontrol()` cancels it, then — if `AUTON == SKILLS` — reschedules it until the
partner controller's `RIGHT` is pressed, which is how skills is driven from the
driver period.

`AUTON` is also read in the `DrivetrainSubsystem` constructor: the IMU is only
calibrated blocking when autonomous, disabled, or skills.

A routine sets its own starting pose, as an `Eigen::Vector3f` in metres and
radians, and seeds the filter with `setNorm()`. It then chains `Ramsete` motions
over compiled-in Bezier paths, `Rotate` turns, and mechanism commands.

Paths are **compiled into the binary**, not read from the SD card:
`BEZIER_MP_ASSET(n_1_6_1_red)` expands to a PROS `ASSET` on
`static/n_1_6_1_red.json` plus a `BezierMotionProfile` built from it at
initialization. Adding a path means dropping the JSON in `static/` and adding the
macro. `BEZIER_MIRRORED_MP_ASSET` builds a red and a blue copy from one file;
most routines instead keep two JSONs and pick with `flip`.

Override is not colour-symmetric — Alliance Goals belong to a colour and a yellow
Pin scores to whoever owns that quadrant's Toggle — so a red routine run on blue
scores for the opposition. The `flip` argument threaded through `Rotate`,
`setNorm` and the path pairs is the mirroring mechanism.

## Localization notes

`DrivetrainSubsystem` owns a `ParticleFilter<CONFIG::NUM_PARTICLES>` (250
particles; the template `static_assert`s at 500) and steps it from `periodic()`,
so every 10 ms. It estimates **position only** — heading is passed in from the
IMU by a function the filter calls, never inferred.

Each step:

1. **Predict.** Odometry change from the rotation sensor on port 9, blurred by a
   uniform `±DRIVE_NOISE` (5%) on distance and `±ANGLE_NOISE` (3 deg) on heading,
   applied to every particle.
2. **Gate.** If the robot has moved less than `maxDistanceSinceUpdate` (2 in)
   *and* less than `maxUpdateInterval` (2 s) has passed, stop here and report the
   particle mean. The sensor model only runs when there is new information to
   justify it.
3. **Weight.** Any particle outside the field is teleported to a uniform random
   point inside it. Then each sensor returns an optional likelihood and they are
   **multiplied** together, so one sensor returning `nullopt` is ignored but one
   returning a near-zero weight kills the particle.
4. **Resample.** Low-variance systematic resampling. The reported pose is the mean
   of the resampled cloud, with the IMU heading attached.

The only sensor model actually wired up is `Distance` (four of them, in
`subsystemInit()`). It knows **the four walls and nothing else** — no Goals, no
other robots. A beam that lands on a Goal is therefore scored as though it had
found a distant wall, which drags the pose outward; that is a real limitation of
the current model, not something the code handles. Each sensor carries a per-unit
tuning scalar (0.979 to 0.987) calibrated against a known distance, and a
standard deviation that widens as the reading's confidence drops. Readings of
9999 mm are discarded.

`gps.h` and `line.h` are other `Sensor` implementations, present but not
constructed.

The cloud is seeded by `initUniform(-70_in, -70_in, 70_in, 70_in, 0_deg, false)`
in `subsystemInit()` — i.e. "somewhere on the field", converging from the distance
sensors. An autonomous routine overrides that with `setNorm()`.

`exponentialPose` in `DrivetrainSubsystem` is a separate, uncorrected pose
integrated with an exponential (arc) model. It is written but never read.

## Ported, and not

What is this robot's, and what is still Echo's. Do not trust the second column
until it has been through the first.

**This robot's:** the ports in `subsystemInit()`, the subsystem list including
`grabberPivotSubsystem`, the lift / intake / pivot driver bindings, the default
commands (hold, brake, coast), and the `MotorSubsystem` group-of-two intake.

**Still last season's**, live in the build and reachable from a controller:

- **The top intake** (`TopIntakeSubsystem`, port 5) — a hooked chain intake with
  optical ring-colour ejection. Built, registered, and driven by `intakeWithEject`,
  `loadLB`, `basicLoadLB` and `cornerClearIntakeSequence`. Its motor is not in the
  88 W budget above.
- **The hang** — PTO, winch rotation sensor, `hangOut`/`hangIn`, `barToBarHang`,
  and the four-shoulder release. Override's endgame is being inside the Midfield
  boundary, not hanging from a ladder.
- **The clamp** (`goalClampSubsystem`, ADI `'e'`) — a mobile-goal clamp. It is
  also what `robotHasGoal()` reports, which selects between the two sets of
  drivetrain feedforward gains.
- **`DIGITAL_DOWN`'s corner-clear sequence** — a ring-corner routine.
- **Every autonomous routine** in `include/autonomous/`, and every path in
  `static/`. They are High Stakes field positions.
- **The lift heights** in `CONFIG::` — `WALL_STAKE_LOAD_HEIGHT`,
  `WALL_STAKE_PRIME_HEIGHT`, `DESCORE_HEIGHT`, `ALLIANCE_STAKE_SCORE_HEIGHT`.
  `DESCORE_HEIGHT` is on `Y` today.

## Numbers this document and the code disagree on

Listed rather than resolved. Each needs a measurement, not a guess.

- **Wheel size.** `CONFIG::DRIVE_RADIUS = 2.75_in / 2.0`. If the drive is on
  3.25" omnis, every distance the drivetrain reports is short by 15%.
- **Track width.** `CONFIG::TRACK_WIDTH = 11_in`; the robot was measured at
  10.43 in (26.5 cm). It feeds both the odometry and the path profiler's
  curvature speed limit.
- **Drive gearing.** `CONFIG::DRIVE_RATIO = 48.0/36.0` — a 36 T motor gear into a
  48 T wheel gear, so 450 rpm at the wheel on blue cartridges. The robot is built
  36 T to 60 T, which is 360 rpm and about 5.1 ft/s. `DRIVE_RATIO` is Echo's
  number.
- **Odometry.** "No tracking wheels", but `getOdomDistance()` reads
  `pros::Rotation(9)` and is what `getDistance()` and the filter use;
  `getLeftDistance()`/`getRightDistance()` compute from motor encoders and are
  unused. `getOdomDistance()` also divides by `DRIVE_RATIO` and scales by
  `DRIVE_RADIUS`, which is only right if that sensor is on the gearbox rather
  than on its own wheel — `ODOM_RADIUS = 2_in/2` exists and is never referenced.

## Outstanding

- [ ] **Drivetrain geometry** — `DRIVE_RADIUS`, `DRIVE_RATIO`, `TRACK_WIDTH`, and
      what the rotation sensor on port 9 is actually attached to. Everything
      downstream of these is wrong by a constant factor until they are measured.
- [ ] **Drivetrain PID and feedforward** in `CONFIG::` — `TURN_PID_GOAL`,
      `TURN_PID_NO_GOAL`, `DISTANCE_PID`, `K_s`, `DRIVETRAIN_TUNING_SCALAR`, and
      the four `DRIVETRAIN_*_VELOCITY_FF_*` rows. All fitted to Echo's chassis,
      which was a different robot with six drive motors.
      `characterizeLinear()`/`characterizeAngular()` on `UP` and `LEFT` regenerate
      the feedforward rows and print them over serial; the numbers still have to
      be pasted back by hand. `robotHasGoal()` picking between the goal and
      no-goal sets is also a High Stakes idea.
- [ ] **Cartridges.** Set them explicitly on the lift, intake/grabber and pivot,
      the way the drivetrain does. Until then every lift angle depends on brain
      state.
- [ ] **Cap the 5.5 W motors.** The grabber and pivot get full 12 V from
      `pctCommand(±1.0)`.
- [ ] **Lift angles.** The four heights in `CONFIG::` are High Stakes. Re-measure
      against this robot's DR4B, and note they are only meaningful relative to
      where `zero()` tares.
- [ ] **Decide what comes off.** The top intake, hang, PTO and clamp are all
      still built. The power budget says at least one of them cannot stay.
- [ ] **Autonomous.** Every routine and every path is last season's field. Nothing
      here is runnable on an Override field.
- [ ] **A run-time selector.** `AUTON` is a `#define`, so choosing a routine at an
      event means a rebuild.
- [ ] **Obstacles in the sensor model.** `Distance` knows only the four walls. The
      nine Goals are large and in known places; until they are modelled, beams
      that hit one bias the pose outward.
- [ ] **Distance sensor offsets** in `CONFIG::DISTANCE_*_OFFSET`. Real numbers,
      but measured on Echo. Re-measure on this robot, along with the per-sensor
      tuning scalars.
- [x] **Manipulator ports and bindings** — lift `13, -14`, intake/grabber `3, 20`,
      pivot `12`, bound as in the table above.
