#pragma once

#include "autons.h"
#include "auton.h"
#include "command/includes.h"

/**
 * Paths are compiled in: drop the JSON in static/ and declare it here, e.g.
 *
 *   BEZIER_MP_ASSET(example_1_red);
 *   BEZIER_MP_ASSET(example_1_blue);
 *
 * A routine seeds the particle filter with its start pose, then chains Ramsete
 * paths, Rotate turns and mechanism commands. `flip` mirrors it for blue:
 *
 *   static Command *example() {
 *       Eigen::Vector3f startPose{(10.0_in).getValue(), (55.0_in).getValue(), (90_deg).getValue()};
 *       const bool flip = ALLIANCE != RED;
 *
 *       return new Sequence({
 *           drivetrainSubsystem->setNorm(startPose.head<2>(), Eigen::Matrix2f::Identity() * 0.05,
 *                                        startPose.z(), flip),
 *           new Ramsete(drivetrainSubsystem, flip ? &example_1_blue : &example_1_red),
 *           (new Rotate(drivetrainSubsystem, 45_deg, flip))->withTimeout(500_ms),
 *       });
 *   }
 */

/**
 * Allows easy selection of autonomous routines given a AUTON object, called on initialization to build states
 */
class AutonomousCommands {
public:
    static Command *getAuton() {
        switch (AUTON) {
            default:
                return new InstantCommand([]() { std::cout << "No auton" << std::endl; }, {});
        }
    }
};
