#include <cassert>
#include <iostream>
#include <cmath>
#include "sim_motor.hpp"

using namespace swerve_drive_controller;

void test_basic_sim_motor(){
    SimMotor motor;
    motor.setCommandVelocity(10.0);
    motor.update(0.1);

    double expected_position = 1.0; // 10.0 * 0.1
    double expected_velocity = 10.0;

    assert(std::abs(motor.getPosition() - expected_position) < 1e-6);
    assert(std::abs(motor.getVelocity() - expected_velocity) < 1e-6);

    std::cout << "Test passed: Basic SimMotor functionality works as expected." << std::endl;
}

void  test_sim_motor_with_negative_velocity(){
    SimMotor motor;
    motor.setCommandVelocity(-5.0);
    motor.update(0.2);

    double expected_position = -1.0; // -5.0 * 0.2
    double expected_velocity = -5.0;

    assert(std::abs(motor.getPosition() - expected_position) < 1e-6);
    assert(std::abs(motor.getVelocity() - expected_velocity) < 1e-6);

    std::cout << "Test passed: SimMotor handles negative velocity correctly." << std::endl;
}

void test_sim_motor_multiple_ticks(){
    SimMotor motor;
    motor.setCommandVelocity(2.0);

    for (int i = 0; i < 100; ++i) {
        motor.update(0.1);
    }

    double expected_position = 20.0; // 2.0 * 0.1 * 100
    double expected_velocity = 2.0;

    assert(std::abs(motor.getPosition() - expected_position) < 1e-6);
    assert(std::abs(motor.getVelocity() - expected_velocity) < 1e-6);

    std::cout << "Test passed: SimMotor handles multiple update ticks correctly." << std::endl;
}

void test_sim_motor_velocity_change(){
    SimMotor motor;
    motor.setCommandVelocity(3.0);
    motor.update(0.1);

    double expected_position = 0.3; // 3.0 * 0.1
    double expected_velocity = 3.0;

    assert(std::abs(motor.getPosition() - expected_position) < 1e-6);
    assert(std::abs(motor.getVelocity() - expected_velocity) < 1e-6);

    motor.setCommandVelocity(6.0);
    motor.update(0.1);

    expected_position += 0.6; // 6.0 * 0.1
    expected_velocity = 6.0;

    assert(std::abs(motor.getPosition() - expected_position) < 1e-6);
    assert(std::abs(motor.getVelocity() - expected_velocity) < 1e-6);

    std::cout << "Test passed: SimMotor handles velocity changes correctly." << std::endl;
}

void test_sim_motor_with_zero_velocity(){
    SimMotor motor;
    motor.setCommandVelocity(0.0);

    for (int i = 0; i < 100; ++i) {
        motor.update(0.1);
    }

    double expected_position = 0.0; // 0.0 * 0.1
    double expected_velocity = 0.0;

    assert(std::abs(motor.getPosition() - expected_position) < 1e-6);
    assert(std::abs(motor.getVelocity() - expected_velocity) < 1e-6);

    std::cout << "Test passed: SimMotor handles zero velocity correctly." << std::endl;
}

int main (){
    test_basic_sim_motor();
    test_sim_motor_with_negative_velocity();
    test_sim_motor_multiple_ticks();
    test_sim_motor_velocity_change();
    test_sim_motor_with_zero_velocity();

    return 0;
}