#pragma once
#include "config.h"

#include "command/command.h"
#include "command/runCommand.h"

#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

class MotorSubsystem : public Subsystem {
    pros::MotorGroup intakeMotor;

public:
    explicit MotorSubsystem(const std::initializer_list<int8_t> &motors,
                            const pros::MotorBrake brake = pros::MotorBrake::coast) : intakeMotor(motors) {
        intakeMotor.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
        intakeMotor.set_brake_mode_all(brake);
    }

    void periodic() override {
        // No-op
    }

    void setPct(const double pct) {
        this->intakeMotor.move_voltage(pct * 12000.0);
    }

    void hold() {
        this->intakeMotor.move_voltage(0.0);
    }

    void setSpeed(double speed) {
        this->intakeMotor.move_velocity(speed);
    }

    RunCommand *stopIntake() {
        return new RunCommand([this]() { this->setPct(0.0); }, {this});
    }

    RunCommand *pctCommand(double pct) {
        return new RunCommand([this, pct]() { this->setPct(pct); }, {this});
    }

    double getTopMotorTemp() const {
        return this->intakeMotor.get_temperature();
    }

    ~MotorSubsystem() override = default;
};
