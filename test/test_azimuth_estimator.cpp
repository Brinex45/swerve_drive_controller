#include <iostream>
#include <cmath>
#include <cassert>
#include "azimuth_estimator.hpp"
#include "swerve_ik.hpp"
#include "differential_mixer.hpp"

using namespace swerve_drive_controller;

void test_derived_equal_azimuth(){
    double motor1_position = 1.57;
    double motor2_position = 1.57;
    double gear_ratio = 1.0;

    double derived_azimuth = estimateAzimuthDerived(motor1_position, motor2_position, gear_ratio);
    double expected_azimuth = (motor1_position - motor2_position) * gear_ratio;

    assert(std::abs(derived_azimuth - expected_azimuth) < 1e-6);
    std::cout << "test_derived_equal_azimuth passed\n";
}

void test_derived_azimuth_with_offset(){
    double motor1_position = 1.57;
    double motor2_position = 1.35;
    double gear_ratio = 1.0;

    double derived_azimuth = estimateAzimuthDerived(motor1_position, motor2_position, gear_ratio);
    double expected_azimuth = (motor1_position - motor2_position) * gear_ratio;

    assert(std::abs(derived_azimuth - expected_azimuth) < 1e-6);
    std::cout << "test_derived_azimuth_with_offset passed\n";
}

void test_azimuth_derived_matches_mixer_steer_direction (){
    double motor1_vel = 1.0;
    double motor2_vel = 0.5;
    
    double time_stamp = 0.5;

    double gear_ratio = 1.0;

    double motor1_position = (motor1_vel * time_stamp);
    double motor2_position = (motor2_vel * time_stamp);

    double derived_azimuth = estimateAzimuthDerived(motor1_position, motor2_position, gear_ratio);

    MotorCommand motors;
    motors.motor1_velocity = motor1_vel;
    motors.motor2_velocity = motor2_vel;

    MixerInput input = unmix(motors, gear_ratio);
    double mixer_steer_direction = input.steer_rate * time_stamp;

    assert(std::abs(derived_azimuth - mixer_steer_direction) < 1e-6);
    std::cout << "test_azimuth_derived_matches_mixer_steer_direction passed\n";
}

void test_absolute_zero_offset_azimuth(){
    double sensor_reading = 1.57;
    double zero_offset = 0;

    double absolute_azimuth = estimateAzimuthAbsolute(sensor_reading, zero_offset);
    double expected_azimuth = SwerveIK::normalizeAngle(sensor_reading - zero_offset);

    assert(std::abs(absolute_azimuth - expected_azimuth) < 1e-6);
    std::cout << "test_absolute_zero_offset_azimuth passed\n";
}

void  test_absolute_nonzero_offset_azimuth(){
    double sensor_reading = 1.57;
    double zero_offset = 0.5;

    double absolute_azimuth = estimateAzimuthAbsolute(sensor_reading, zero_offset);
    double expected_azimuth = SwerveIK::normalizeAngle(sensor_reading - zero_offset);

    assert(std::abs(absolute_azimuth - expected_azimuth) < 1e-6);
    std::cout << "test_absolute_nonzero_offset_azimuth passed\n";
}

void test_absolute_large_offset_azimuth(){
    double sensor_reading = 1.57;
    double zero_offset = 3.14;

    double absolute_azimuth = estimateAzimuthAbsolute(sensor_reading, zero_offset);
    double expected_azimuth = SwerveIK::normalizeAngle(sensor_reading - zero_offset);

    assert(std::abs(absolute_azimuth - expected_azimuth) < 1e-6);
    std::cout << "test_absolute_large_offset_azimuth passed\n";
}

int main(){

    test_derived_equal_azimuth();
    test_derived_azimuth_with_offset();
    test_azimuth_derived_matches_mixer_steer_direction();
    test_absolute_zero_offset_azimuth();
    test_absolute_nonzero_offset_azimuth();
    test_absolute_large_offset_azimuth();

    return 0;
}