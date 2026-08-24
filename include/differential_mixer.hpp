#pragma once

#include <cmath>

namespace swerve_drive_controller
{

    struct MixerInput{
        double wheel_speed;
        double steer_rate;
    };

    struct MotorCommand{
        double motor1_velocity;
        double motor2_velocity;
    };

    inline MotorCommand mix(const MixerInput& input, double gear_ratio)
    {
        MotorCommand cmd;
        cmd.motor1_velocity = (input.wheel_speed + input.steer_rate) / (2.0 * gear_ratio);
        cmd.motor2_velocity = (input.wheel_speed - input.steer_rate) / (2.0 * gear_ratio);
        return cmd;
    }

    inline MixerInput unmix(const MotorCommand& motors, double gear_ratio)
    {
        MixerInput result;
        result.wheel_speed = gear_ratio * (motors.motor1_velocity + motors.motor2_velocity);
        result.steer_rate  = gear_ratio * (motors.motor1_velocity - motors.motor2_velocity);
        return result;
    }

} // namespace swerve_drive_controller