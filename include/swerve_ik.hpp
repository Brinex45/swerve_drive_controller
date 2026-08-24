#pragma once

#include <cmath>

namespace swerve_drive_controller
{

    struct SwerveIK{
        static constexpr double kEpsilon = 1e-6;

        static double normalizeAngle(double angle)
        {
            while (angle > M_PI)
                angle -= 2.0 * M_PI;
            while (angle < -M_PI)
                angle += 2.0 * M_PI;
            return angle;
        }
    };

    struct Twist{
        double vx;
        double vy;
        double omega;
    };

    struct ModuleCommand{
        double speed;
        double angle;
    };

    struct ModulePos{
        double x;
        double y;
    };

    inline ModuleCommand computeModuleCommand(const Twist& twist, const ModulePos& module_pos){
        ModuleCommand command;
        // Implementation for computing module command

        double vx_rot = -twist.omega * module_pos.y;
        double vy_rot = twist.omega * module_pos.x;

        double vx_total = twist.vx + vx_rot;
        double vy_total = twist.vy + vy_rot;

        if (std::abs(vx_total) < SwerveIK::kEpsilon && std::abs(vy_total) < SwerveIK::kEpsilon)
        {
            command.speed = 0.0;
            command.angle = 0.0;
        }
        else
        {
            command.speed = std::sqrt(vx_total * vx_total + vy_total * vy_total);
            command.angle = std::atan2(vy_total, vx_total);
        }

        return command;
    }

    inline ModuleCommand optimizeSteerDirection(const ModuleCommand& target, double current_angle)
    {
        double diff = SwerveIK::normalizeAngle(target.angle - current_angle);
        ModuleCommand optimized;

        if (std::abs(diff) > M_PI / 2.0)
        {
            optimized.speed = -target.speed;
            optimized.angle = SwerveIK::normalizeAngle(target.angle + M_PI);
        }
        else
        {
            optimized.speed = target.speed;
            optimized.angle = target.angle;
        }
        
        return optimized;
    }

} // namespace swerve_drive_controller