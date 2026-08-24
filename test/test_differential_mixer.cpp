#include <cassert>
#include <cmath>
#include <iostream>
#include "differential_mixer.hpp"

using namespace swerve_drive_controller;
using namespace std;

void test_zero_mixer_input() {
    MixerInput input{0.0, 0.0};        // no movement
    double gear_ratio = 1.0;          // simple gear ratio

    MotorCommand cmd = mix(input, gear_ratio);

    assert(std::abs(cmd.motor1_velocity - 0.0) < 1e-6);
    assert(std::abs(cmd.motor2_velocity - 0.0) < 1e-6);
    cout << "test_zero_mixer_input passed\n";
}

void test_drive_mixer_input() {
    MixerInput input{1.0, 0.0};        // drive forward
    double gear_ratio = 1.0;          // simple gear ratio

    MotorCommand cmd = mix(input, gear_ratio);

    assert(std::abs(cmd.motor1_velocity - 0.5) < 1e-6);
    assert(std::abs(cmd.motor2_velocity - 0.5) < 1e-6);
    cout << "test_drive_mixer_input passed\n";
}

void test_steer_mixer_input() {
    MixerInput input{0.0, 1.0};        // steer in place
    double gear_ratio = 1.0;          // simple gear ratio

    MotorCommand cmd = mix(input, gear_ratio);

    assert(std::abs(cmd.motor1_velocity - 0.5) < 1e-6);
    assert(std::abs(cmd.motor2_velocity + 0.5) < 1e-6);
    cout << "test_steer_mixer_input passed\n";
}

void test_combined_mixer_input() {
    MixerInput input{1.0, 1.0};        // drive forward and steer
    double gear_ratio = 1.0;          // simple gear ratio

    MotorCommand cmd = mix(input, gear_ratio);

    assert(std::abs(cmd.motor1_velocity - 1.0) < 1e-6);
    assert(std::abs(cmd.motor2_velocity - 0.0) < 1e-6);
    cout << "test_combined_mixer_input passed\n";
}

void test_different_gear_ratio_mixer_input(){
    MixerInput input{1.0, 5.0};        // drive forward and steer
    double gear_ratio = 1.2;          // simple gear ratio

    MotorCommand cmd = mix(input, gear_ratio);

    assert(std::abs(cmd.motor1_velocity - 2.5) < 1e-6);
    assert(std::abs(cmd.motor2_velocity + (4.0/2.4)) < 1e-6);
    cout << "test_different_gear_ratio_mixer_input passed\n";
}

void test_unmix_function() {
    MotorCommand motors{1.0, 0.5};     // motor velocities
    double gear_ratio = 2.0;          // simple gear ratio

    MixerInput input = unmix(motors, gear_ratio);

    assert(std::abs(input.wheel_speed - 3.0) < 1e-6);
    assert(std::abs(input.steer_rate - 1.0) < 1e-6);
    cout << "test_unmix_function passed\n";
}

void test_mix_unmix_roundtrip() {
    MixerInput original{1.5, -0.7};
    double gear_ratio = 3.3;

    MotorCommand motors = mix(original, gear_ratio);
    MixerInput recovered = unmix(motors, gear_ratio);

    assert(std::abs(recovered.wheel_speed - original.wheel_speed) < 1e-6);
    assert(std::abs(recovered.steer_rate - original.steer_rate) < 1e-6);
    cout << "test_mix_unmix_roundtrip passed\n";
}

int main() {

    test_zero_mixer_input();
    test_drive_mixer_input();
    test_steer_mixer_input();
    test_combined_mixer_input();
    test_different_gear_ratio_mixer_input();
    test_unmix_function();
    test_mix_unmix_roundtrip();
    
    return 0;
}