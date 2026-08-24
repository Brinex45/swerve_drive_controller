#pragma once

#include <vector>
#include <cmath>
#include "swerve_ik.hpp"
#include "odometry.hpp"
#include "sim_module.hpp"

namespace swerve_drive_controller {

struct ModuleConfig {
    double gear_ratio;
    ModulePos position;
};

class SwerveRobot {
public:
    SwerveRobot(const std::vector<ModuleConfig>& module_configs) {
        for (const auto& config : module_configs) {
            modules_.emplace_back(config.gear_ratio, config.position);
        }
    }

    void update(const Twist& twist, double dt){
        for (auto& module : modules_) {
            module.update(twist, dt);
        }

        std::vector<ModuleState> states;
        for (const auto& module : modules_) {
            states.push_back({
                ModuleCommand{module.getCurrentSpeed(), module.getCurrentAngle()},
                module.getPosition()
            });
        }

        Twist estimated_twist = estimateTwist(states);

        integratePose(pose_, estimated_twist, dt);
    }

    Pose getPose() const {
        return pose_;
    }

    size_t getModuleCount() const {
        return modules_.size();
    }

    ModuleCommand getModuleState(size_t index) const {
        const auto& m = modules_[index];
        return ModuleCommand{m.getCurrentSpeed(), m.getCurrentAngle()};
    }

private:
    std::vector<SimModule> modules_;
    Pose pose_{0.0, 0.0, 0.0};
};

}