#pragma once

#include "bottomIntake.h"
#include "command/commandController.h"
#include "command/commandScheduler.h"
#include "command/conditionalCommand.h"
#include "command/parallelCommandGroup.h"
#include "command/parallelRaceGroup.h"
#include "command/proxyCommand.h"
#include "command/repeatCommand.h"
#include "command/scheduleCommand.h"
#include "command/sequence.h"
#include "command/waitCommand.h"
#include "command/waitUntilCommand.h"
#include "commands/driveMove.h"
#include "commands/lift/trapPosition.h"
#include "commands/ramsete.h"
#include "commands/rotate.h"
#include "drivetrain.h"
#include "lift.h"
#include "localization/distance.h"
#include "motionProfiling/pathCommands.h"

#include <algorithm>
#include <queue>

inline DrivetrainSubsystem *drivetrainSubsystem;
inline MotorSubsystem *bottomIntakeSubsystem;
inline MotorSubsystem *grabberPivotSubsystem;
inline LiftSubsystem *liftSubsystem;

inline CommandController primary(pros::controller_id_e_t::E_CONTROLLER_MASTER);
inline CommandController partner(pros::controller_id_e_t::E_CONTROLLER_PARTNER);

inline void initializeController() {
    // Lift: R1 raises, R2 lowers, full 12 V either way. The shoulder negations
    // mean no two shoulders drive anything at once.
    primary.getTrigger(DIGITAL_R1)
        ->andOther(primary.getTrigger(DIGITAL_R2)->negate())
        ->andOther(primary.getTrigger(DIGITAL_L1)->negate())
        ->andOther(primary.getTrigger(DIGITAL_L2)->negate())
        ->whileTrue(liftSubsystem->pctCommand(1.0));

    primary.getTrigger(DIGITAL_R2)
        ->andOther(primary.getTrigger(DIGITAL_R1)->negate())
        ->andOther(primary.getTrigger(DIGITAL_L1)->negate())
        ->andOther(primary.getTrigger(DIGITAL_L2)->negate())
        ->whileTrue(liftSubsystem->pctCommand(-1.0));

    // Intake and grabber are one group: L1 intakes, L2 outtakes.
    primary.getTrigger(DIGITAL_L1)
        ->andOther(primary.getTrigger(DIGITAL_L2)->negate())
        ->andOther(primary.getTrigger(DIGITAL_R1)->negate())
        ->andOther(primary.getTrigger(DIGITAL_R2)->negate())
        ->whileTrue(bottomIntakeSubsystem->pctCommand(1.0));
    primary.getTrigger(DIGITAL_L2)
        ->andOther(primary.getTrigger(DIGITAL_L1)->negate())
        ->andOther(primary.getTrigger(DIGITAL_R1)->negate())
        ->andOther(primary.getTrigger(DIGITAL_R2)->negate())
        ->whileTrue(bottomIntakeSubsystem->pctCommand(-1.0));

    // Grabber pivot: B raises, X lowers, holding position when neither is held.
    primary.getTrigger(DIGITAL_B)->whileTrue(grabberPivotSubsystem->pctCommand(1.0));
    primary.getTrigger(DIGITAL_X)->whileTrue(grabberPivotSubsystem->pctCommand(-1.0));

    // Drivetrain sysid: drives the robot through a fixed voltage script and prints
    // the fitted feedforward rows over serial. Bench use only, so off by default;
    // uncomment to regenerate DRIVETRAIN_*_VELOCITY_FF_* in CONFIG::.
    // primary.getTrigger(DIGITAL_LEFT)->whileTrue(drivetrainSubsystem->characterizeAngular());
    // primary.getTrigger(DIGITAL_UP)->whileTrue(drivetrainSubsystem->characterizeLinear());

    // Alliance toggle: hold A for 200 ms to flip ALLIANCE. Wants a partner
    // controller combo rather than a face button if it comes back.
    // primary.getTrigger(DIGITAL_A)->whileTrue((new WaitCommand(200_ms))
    //                                              ->andThen(new InstantCommand(
    //                                                  []() {
    //                                                      auto newAlliance = OPPONENTS;
    //                                                      ALLIANCE = newAlliance;
    //                                                  },
    //                                                  {})));
}

inline void initializePathCommands() {
    // Names a path JSON's "commands" table can fire mid-motion.
    PathCommands::registerCommand("intake", bottomIntakeSubsystem->pctCommand(1.0));
    PathCommands::registerCommand("outtake", bottomIntakeSubsystem->pctCommand(-1.0));
    PathCommands::registerCommand("stopIntake", bottomIntakeSubsystem->pctCommand(0.0));
    PathCommands::registerCommand(
        "zeroLift", liftSubsystem->positionCommand(10_deg)->withTimeout(1_s)->andThen(liftSubsystem->zero()));
}

// Named Command* globals that the bindings and path commands share are built here.
inline void initializeCommands() {}

inline void subsystemInit() {
    TELEMETRY.setSerial(new pros::Serial(0, 921600));

    // Smart ports below come from TheLib (TheLib-main/src/main.cpp):
    //   left drive   -2, -7      right drive   4, 6
    //   inertial      8
    //   lift         13, -14
    //   intake        3          grabber      20   (linked as one group)
    //   grabber pivot 12
    //   distance: front 18, left 10, right 17, back 16
    // TheLib defines no port for the odometry rotation sensor, so it sits on 9,
    // which TheLib leaves free.

    // Intake and grabber run as one group, so every intake command drives both
    // motors together.
    bottomIntakeSubsystem = new MotorSubsystem({3, 20});
    grabberPivotSubsystem = new MotorSubsystem({12}, pros::MotorBrake::hold);
    liftSubsystem = new LiftSubsystem({-13, 14}, PID(1.2, 0.0, 3.0, 0.2, 1.0));
    // hasGoal is fixed false: nothing on this robot reports possession yet, so the
    // drivetrain always uses the *_NO_GOAL feedforward and turn gains.
    drivetrainSubsystem = new DrivetrainSubsystem({-2, -7}, {4, 6}, pros::Imu(8), []() { return false; },
                                                  pros::Rotation(9)); // 9 odom rotation

    pros::Task([]() {
        if (!drivetrainSubsystem->odomConnected()) {
            primary.rumble("-- --");
            pros::delay(2000);
        }

        if (pros::battery::get_capacity() < 50.0) {
            primary.rumble("..-");
            pros::delay(2000);
        }

        // Check motor temps
        if (std::max({drivetrainSubsystem->getTopMotorTemp(), bottomIntakeSubsystem->getTopMotorTemp(),
                      grabberPivotSubsystem->getTopMotorTemp(), liftSubsystem->getTopMotorTemp()}) >= 45.0) {
            primary.rumble(".--");
        }
    });

    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_LEFT_OFFSET, 0.987, pros::Distance(10)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_FRONT_OFFSET, 0.986, pros::Distance(18)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_RIGHT_OFFSET, 0.980, pros::Distance(17)));
    drivetrainSubsystem->addLocalizationSensor(new Distance(CONFIG::DISTANCE_BACK_OFFSET, 0.979, pros::Distance(16)));

    drivetrainSubsystem->initUniform(-70_in, -70_in, 70_in, 70_in, 0_deg, false);

    CommandScheduler::registerSubsystem(drivetrainSubsystem, drivetrainSubsystem->tank(primary));
    CommandScheduler::registerSubsystem(bottomIntakeSubsystem, bottomIntakeSubsystem->stopIntake());
    // Nothing pressed: the pivot brakes where it was left.
    CommandScheduler::registerSubsystem(grabberPivotSubsystem, grabberPivotSubsystem->holdCommand());
    // Nothing pressed: the lift holds the angle it was released at.
    CommandScheduler::registerSubsystem(liftSubsystem, liftSubsystem->holdPositionCommand());

    initializeCommands();
    initializeController();
    initializePathCommands();
}
