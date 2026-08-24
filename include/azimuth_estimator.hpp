#pragma once

#include <cmath>
#include "swerve_ik.hpp"

namespace swerve_drive_controller
{   
    inline double estimateAzimuthDerived(double motor1_position, double motor2_position, double gear_ratio){
        return (motor1_position - motor2_position) * gear_ratio;
    }

    inline double estimateAzimuthAbsolute(double sensor_reading, double zero_offset){
        return SwerveIK::normalizeAngle(sensor_reading - zero_offset);
    }

} // namespace swerve_drive_controller