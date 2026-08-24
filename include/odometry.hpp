#pragma once

#include <cmath>
#include <vector>
#include "swerve_ik.hpp"

namespace swerve_drive_controller
{
    // one module's reading + its position, paired together for this function
    struct ModuleState
    {
        ModuleCommand command;   // speed, angle (measured, not commanded)
        ModulePos position;
    };

    inline Twist estimateTwist(const std::vector<ModuleState>& module_states)
    {
        if (module_states.empty())
        {
            return Twist{0.0, 0.0, 0.0};
        }

        Twist result{0.0, 0.0, 0.0};

        double vx_sum = 0.0, vy_sum = 0.0, omega_sum = 0.0;
        for (const auto& m : module_states)
        {
            vx_sum += m.command.speed * std::cos(m.command.angle);
            vy_sum += m.command.speed * std::sin(m.command.angle);
        }

        double vx_est = vx_sum / module_states.size();
        double vy_est = vy_sum / module_states.size();

        for (const auto& m: module_states){
            double vx_i = m.command.speed * std::cos(m.command.angle);
            double vy_i = m.command.speed * std::sin(m.command.angle);

            double vx_rot = vx_i - vx_est;
            double vy_rot = vy_i - vy_est;

            double r_sq = m.position.x * m.position.x + m.position.y * m.position.y;
            if (r_sq < SwerveIK::kEpsilon) continue;

            omega_sum += (m.position.x * vy_rot - m.position.y * vx_rot) / r_sq;
        }

        result.omega = omega_sum / module_states.size();
        result.vx = vx_est;
        result.vy = vy_est;
        return result;
    }

    struct Pose
    {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
    };

    inline void integratePose(Pose& pose, const Twist& twist, double dt)
    {
        pose.x += (twist.vx * std::cos(pose.theta) - twist.vy * std::sin(pose.theta)) * dt;
        pose.y += (twist.vx * std::sin(pose.theta) + twist.vy * std::cos(pose.theta)) * dt;
        pose.theta += twist.omega * dt;
    }

} // namespace swerve_drive_controller