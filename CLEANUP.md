# Codebase audit: what was removed

The fork still carried 2654E's High Stakes robot: a top intake with optical ring-colour sorting, a mobile-goal clamp, a PTO winch hang, pneumatics, and every autonomous routine and path. None of that exists on this robot. This audit deletes it, comments out the few pieces that are likely to come back, and leaves the drivetrain, localization and path-following code untouched.

**Recovering anything below.** `ead3900` ("controlled descent fully done") is the last commit with everything intact.

```
git show ead3900:include/subsystems/topIntake.h            # view a deleted file
git checkout ead3900 -- include/subsystems/topIntake.h     # restore it
```

## Deleted files

| File | What it was | Why it went |
| --- | --- | --- |
| `include/subsystems/topIntake.h`, `src/subsystems/topIntake.cpp` | `TopIntakeSubsystem`: hooked chain intake on port 5, optical sensor on 21 reading ring colour | No top intake on this robot. It was also a tenth motor, putting the robot at 99 W against the 88 W limit. |
| `include/commands/intake/positionCommand.h` | `TopIntakePositionCommand`: PID on chain-hook position | Top intake only |
| `include/commands/intake/trapTopPosition.h` | `TrapTopPosition`: profiled version of the above | Top intake only |
| `include/subsystems/solenoidSubsystem.h` | `SolenoidSubsystem`: ADI digital outs | No pneumatics |
| `include/commands/driveToGoal.h` | Drive toward a mobile goal using the AI vision sensor | High Stakes goal, no AI vision sensor. It also didn't compile: it used a `Drivetrain` type that doesn't exist and was never included. |
| `include/commands/ltvUnicycleController.h` | LTV unicycle path follower, an alternative to Ramsete | Never included and didn't compile (same `Drivetrain` type). Ramsete is the follower in use. |
| `include/utils/linear.h` | `discretizeAB`, `dareSolver` (matrix exponential, DARE) | Only used by the LTV controller |
| `include/autonomous/negative.h` | `N_1_6`, `N_1_6P`, `N_6` routines | High Stakes field positions |
| `include/autonomous/positive.h` | `P_4`, `P_1_3` routines | High Stakes field positions |
| `include/autonomous/skills.h` | High Stakes skills routine, ending in a ladder hang | High Stakes field positions |
| `include/autonomous/sharedCommands.h` | Alliance stake, wall stake and ring-corner helpers | High Stakes scoring |
| `static/*.json` (18 files) | `n_1_6_{1..4}_{red,blue}`, `p_4_{1..3}_{red,blue}`, `skills_{1..4}` | Bezier paths on the High Stakes field |
| `uploadAllAutons.py` | Rewrote `auton.h` for each High Stakes routine and uploaded it to a slot | Only knew the deleted routines |
| `.fleet/run.json` | JetBrains Fleet run config for `uploadAllAutons.py` | Its script is gone |

## Deleted code inside kept files

**`include/subsystems/subsystems.h`**
- Subsystems: `topIntakeSubsystem`, `goalClampSubsystem` (ADI `'e'`), `hangSubsystem` (ADI `'a'`/`'d'`).
- Named commands: `loadLB`, `basicLoadLB` (both the optical and no-optical variants), `goalClampTrue`, `hang`, `barToBarHang`, `gripBar`, `letOutString`, `intakeNoEject`, `topIntakeWithEject` (the colour-ejection sequence), `intakeWithEject`, `bottomOuttakeWithEject`, `hangRelease`, `hangIdle`, `cornerClearIntakeSequence`.
- Triggers and state: `negatedHang`, `negatedLBLoad`, `liftLow`, `hangReleased`.
- Path commands: `clamp`, `declamp`, `intakeWithEject`, `intakeNoEject`, `basicLoadLB`, `basicLoadLB2Ring`, `loadLB`, `bottomIntakeOffTopOn`, `stopIntake3`, `hangRelease`, `liftZero`, `scoreAllianceStake`, `LBdrop`, `lbTouch`.
- Startup checks: the "optical sensor missing" rumble, and the top intake in the motor-temperature check.
- Port comment: the top intake, winch and ADI entries.

**`include/subsystems/drivetrain.h`**
- PTO (`pros::adi::DigitalOut pto`, ADI `'c'`), winch rotation sensor (port 19), `ptoActive`, `onPtoActivateStringPosition`. The constructor no longer takes `pto` or `winchRotation`.
- `getStringDistance()`, `activatePto()`, `retractPto()`, `hangController()`, `hangOut()`, `hangIn()`, `hangOutNoPto()`, `hangPctCommand()`.
- `updateAllianceColor()`: read a potentiometer on ADI `'b'` to pick the alliance and wrote it to the controller screen. Only the deleted routines called it.
- The unused `GpsSensor *sensor` member and the `localization/gps.h` include.

**`include/config.h`**
- `INTAKE_RATIO`, `TOP_INTAKE_DEFAULT_TOLERANCE`, `TOP_INTAKE_PID` (top intake).
- `STRING_RATIO`, `START_STRING_LENGTH`, `WINCH_RADIUS` (hang winch).
- `AI_VISION_PIXELS_TO_DEGREES` (AI vision).
- `DEFAULT_DT_COST_Q`, `DEFAULT_DT_COST_R` (LTV controller).

**`include/autonomous/autons.h`, `auton.h`, `autonCommands.h`**
- The enum values `N_1_6`, `N_1_6P`, `N_6`, `P_4`, `P_1_3`. The enum keeps `SKILLS` and `NONE`, and `AUTON` is now `NONE`.
- The `switch` cases for the deleted routines.

**`README.md`**
- 2654E's own README (their notebook, videos, contact details). Replaced with a short one that credits the upstream repo.

## Primary controller bindings

**Kept** (the ones added on this fork): `R1`/`R2` lift, `L1`/`L2` intake + grabber, `B`/`X` grabber pivot, and tank drive on the sticks.

**Deleted**:

| Binding | What it did |
| --- | --- |
| `L1`+`L2`+`R1`+`R2` | Hang release, latched `hangReleased` |
| `Y` | Lift to `DESCORE_HEIGHT` racing the top intake; after hang release, the full hang sequence |
| `DOWN` | Repeating ring-corner-clear sequence that drove the robot |
| `RIGHT` | Clamp solenoid |

**Commented out** in `initializeController()`, ready to bring back:

| Binding | What it does |
| --- | --- |
| `LEFT` / `UP` | Angular / linear drivetrain sysid. Needed to re-tune feedforward for this chassis. |
| `A` | Hold 200 ms to flip `ALLIANCE` |

## Also commented out

- **Lift heights** in `CONFIG::`: `LIFT_IDLE_POSITION`, `WALL_STAKE_LOAD_HEIGHT`, `WALL_STAKE_PRIME_HEIGHT`, `DESCORE_HEIGHT`, `ALLIANCE_STAKE_SCORE_HEIGHT`. These are High Stakes angles. The structure is useful; the numbers need re-measuring on this DR4B.
- **An example routine** in `autonCommands.h`, showing `setNorm` → `Ramsete` → `Rotate` with `flip`, as a template for the first Override routine.

## Behaviour that changed

- **`robotHasGoal()` is always `false`.** It used to read the clamp. It is now a constant lambda in `subsystemInit()`, so `Ramsete`, `Rotate` and `TankMotionProfiling` always use the `*_NO_GOAL` feedforward and `TURN_PID_NO_GOAL`. Echo switched to the `*_GOAL` sets whenever the clamp was closed, either by `RIGHT` in teleop or by the `clamp` path command that most of its autonomous paths fired partway through. With no clamp, only the no-goal sets are ever used. The goal/no-goal machinery itself is unchanged, ready to point at a possession sensor.
- **No autonomous runs.** `AUTON` is `NONE`, so `autonomous()` schedules a command that prints "No auton". At boot the robot no longer blocks in `updateAllianceColor()` reading the potentiometer.
- **Port 5 and every ADI port are now unused**, and the robot is at the legal 88 W.
- **The shoulder bindings still negate each other.** Upstream that stopped the four-shoulder hang combo driving the lift and intake. With the hang gone, all it does now is stop the lift and intake running at the same time. It was left as-is; drop the cross-pair negations if the driver wants both at once.
- **Path command names.** The table now registers `intake`, `outtake` (formerly `outtakeBottom`), `stopIntake` (which no longer also parks the top intake) and `zeroLift` (formerly `resetLB`, still the only caller of `LiftSubsystem::zero()`).

## Deliberately untouched

These files are byte-for-byte identical to `ead3900`:

- Localization: `include/localization/` (particle filter, distance, sensor, config, and the unused `gps.h` and `line.h`)
- Path following and motion: `include/commands/ramsete.h`, `rotate.h`, `driveMove.h`, `include/motionProfiling/`, `include/velocityProfile/`
- Supporting code: `include/feedback/`, `include/sysid/`, `include/utils/utils.h`, `include/utils/motor.h`, `src/utils/`
- Mechanisms and entry points: `include/subsystems/lift.h`, `include/subsystems/bottomIntake.h`, `include/commands/lift/trapPosition.h`, `src/main.cpp`

In `drivetrain.h`, only the hang, PTO, GPS and potentiometer pieces were removed. None of these changed: `periodic()` (filter predict/update, exponential pose, sysid recording), `setDriveSpeeds()`, `getOdomDistance()`, the IMU heading lambda, `initNorm`/`initUniform`/`setNorm`/`getPose`.

## Caveats

- **`gps.h` and `line.h` are not compiled.** Nothing includes them now. They were never constructed, but the build no longer type-checks them either. `line.h`'s field lines are High Stakes tape.
- **The hot binary is much smaller** (text 475 KB → 39 KB). Nothing calls `Ramsete`, the Bezier profiler or the path assets until a routine exists, so the linker drops them. They are non-template classes, so the compiler still fully type-checks them on every build.
