#pragma once

#include <cmath>

namespace swerve_drive_controller {
    class SimMotor {
    public:
        void setCommandVelocity(double speed){
            command_velocity_ = speed;
        }

        void update(double dt){
            velocity_ = command_velocity_;
            position_ += velocity_ * dt;
        }
        
        double getPosition() const {
            return position_;
        }
        
        double getVelocity() const {
            return velocity_;
        }

    private:
        double position_ = 0.0;
        double velocity_ = 0.0;
        double command_velocity_ = 0.0;
    };
}