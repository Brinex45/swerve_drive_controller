#pragma once

#include <cmath>
#include "swerve_ik.hpp"
#include "differential_mixer.hpp"
#include "azimuth_estimator.hpp"
#include "sim_motor.hpp"

namespace swerve_drive_controller {
    class SimModule {
    public:
        SimModule(double gear_ratio, const ModulePos& position)
            : gear_ratio_(gear_ratio), position_(position) {}
        
        void update(const Twist& twist, double dt){

            ModuleCommand command = computeModuleCommand(twist, position_);

            double m1_pos = motor1_.getPosition();
            double m2_pos = motor2_.getPosition();

            double current_azimuth = estimateAzimuthDerived(m1_pos, m2_pos, gear_ratio_);

            command = optimizeSteerDirection(command, current_azimuth);

            MixerInput mixer_input;

            // TODO: replace with real steer PID at controller layer, this is an idealized instant-convergence placeholder for sim validation
            mixer_input.wheel_speed = command.speed;
            mixer_input.steer_rate = SwerveIK::normalizeAngle(command.angle - current_azimuth) / dt;

            MotorCommand motor_command = mix(mixer_input, gear_ratio_);

            motor1_.setCommandVelocity(motor_command.motor1_velocity);
            motor2_.setCommandVelocity(motor_command.motor2_velocity);

            motor1_.update(dt);
            motor2_.update(dt);
        }

        double getCurrentAngle() const {
            double m1_pos = motor1_.getPosition();
            double m2_pos = motor2_.getPosition();
            return estimateAzimuthDerived(m1_pos, m2_pos, gear_ratio_);
        }
        
        double getCurrentSpeed() const {
            double m1_vel = motor1_.getVelocity();
            double m2_vel = motor2_.getVelocity();
            MixerInput mixer_input = unmix({m1_vel, m2_vel}, gear_ratio_);
            return mixer_input.wheel_speed;
        }
        
        ModulePos getPosition() const {
            return position_;
        }
        
    private:
        SimMotor motor1_, motor2_;
        ModulePos position_;

        double gear_ratio_;
    };
}