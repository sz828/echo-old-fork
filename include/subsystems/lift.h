#pragma once

#include <utility>
#include <algorithm>
#include <cmath>
#include "command/runCommand.h"

#include "command/subsystem.h"
#include "units/units.hpp"
#include "feedback/pid.h"
#include "config.h"

class LiftSubsystem : public Subsystem {
private:
    pros::MotorGroup motor;

    PID pid;

    std::optional<double> voltage;

    Angle lastPosition;
    QAngularVelocity velocity;

    QTime lastFree = 0.0;

    //descent tuning
    double maxDownPct = 0.50;                  // cap on descent effort (0..1)
    QAngularVelocity maxDownSpeed = 180_deg / 1_s;  // descent speed limit
    double kG = 0.0;                           // gravity feedforward, tune after clamps work
    Angle landingzone = 40_deg;                      // start slowing this far above the bottom
    QAngularVelocity landingSpeed = 60_deg / 1_s;    // speed limit at the very bottom
    double downAccel = 900.0;                  // deg/s gained per second when starting a descent
    QAngularVelocity downSpeed = 0.0;          // descent speed being commanded, ramps up to the limit


public:
    explicit LiftSubsystem(const std::initializer_list<int8_t> &motors, const PID &pid) : motor(motors), pid(pid) {
        motor.set_encoder_units_all(pros::MotorEncoderUnits::rotations);
        motor.set_brake_mode_all(pros::MotorBrake::hold);
        motor.tare_position_all();
    }

    void periodic() override {
        auto position = this->getPosition();
        if (!voltage.has_value()) {
            auto command = pid.update(position.Convert(radian));
            command += kG * std::cos(position.Convert(radian));

            if (command < 0.0) {
                command = std::max(command, -maxDownPct);
                if (velocity < -downSpeedLimit(position)) command = 0.0;
            }

            motor.move_voltage(command * 12000.0);
        }

        if (abs(motor.get_current_draw()) < 1000) {
            lastFree = pros::millis() * millisecond;
        }

        velocity = (position - lastPosition)/10_ms;

        lastPosition = position;

    }

    QAngularVelocity downSpeedLimit(Angle position) const {
        // 1 above the landing zone, falls to 0 at the bottom
        float t = std::clamp(position.Convert(degree) / landingzone.Convert(degree), 0.0f, 1.0f);
        return landingSpeed + t * (maxDownSpeed - landingSpeed);
    }

    void setTarget(Angle angle) {
        pid.setTarget(angle.Convert(radian));
        voltage = std::nullopt;
        downSpeed = 0.0;
    }

    void setVoltage(double voltage) {
        this->voltage = voltage;
        if (voltage < 0.0) {
            // Descend on the motor's own velocity loop: it brakes against gravity smoothly,
            // where cutting to 0 V just freewheels and chatters. Speed scales with |voltage|.
            // Ramp up from rest rather than jumping to the limit, so a short drop that starts
            // inside the landing zone doesn't overshoot and hit the bottom hard.
            downSpeed = std::min(downSpeed + downAccel * 10_ms * degree / second / second,
                                 -voltage * downSpeedLimit(getPosition()));
            double rpm = downSpeed.Convert(revolution / minute) * CONFIG::LIFT_RATIO;
            motor.move_velocity(-rpm);
            return;
        }
        downSpeed = 0.0;
        motor.move_voltage(voltage * 12000.0);
    }

    Angle getPosition() const {
        return motor.get_position(0) / CONFIG::LIFT_RATIO * revolution;
    }

    FunctionalCommand *positionCommand(Angle angle, Angle threshold = 8_deg) {
        return new FunctionalCommand(
            [this, angle]() { this->setTarget(angle);
                                     }, [this, angle]() { this->setTarget(angle); }, [](bool _) {
                                     }, [this, threshold, angle]() {
                                         return Qabs(this->getPosition() - angle) < threshold;
                                     }, {this});
    }

    FunctionalCommand *holdPositionCommand() {
        return new FunctionalCommand([this]() {
                                         this->setTarget(this->getPosition());
                                     }, []() {
                                     }, [](bool _) {
                                     }, []() { return false; }, {this});
    }

    RunCommand *pctCommand(double voltage) {
        return new RunCommand([this, voltage]() mutable { this->setVoltage(voltage); }, {this});
    }

    double getTopMotorTemp() const {
        return this->motor.get_temperature();
    }

    bool stalled(QTime duration) const {
        return pros::millis() * 1_ms - lastFree > duration;
    }

    Command *zero() {
        return (new FunctionalCommand([this]() { this->setVoltage(-0.2); }, []() {
                                      }, [this](bool _) {
                                      }, [this]() { return this->stalled(300_ms); }, {this}))->withTimeout(2_s)->andThen(
            this->pctCommand(-0.1)->withTimeout(200_ms))->andThen(this->pctCommand(0.0)->withTimeout(200_ms))->andThen(
            new InstantCommand([this]() { this->motor.tare_position(); }, {this}));
    }

    QAngularVelocity getVelocity() const { return velocity; }

    ~LiftSubsystem() override = default;
};
