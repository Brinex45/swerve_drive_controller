#include<cassert>
#include<cmath>
#include<iostream>
#include"sim_module.hpp"

using namespace swerve_drive_controller;

void test_straight_line_motion() {
    double gear_ratio = 1.0;
    ModulePos position{1.0, 1.0};
    SimModule module(gear_ratio, position);

    Twist twist{1.0, 0.0, 0.0}; // Move forward at 1 m/s
    double dt = 1.0; // 1 second time step

    module.update(twist, dt);

    double speed = module.getCurrentSpeed();
    double angle = module.getCurrentAngle();

    assert(std::abs(speed - 1.0) < 1e-6);
    assert(std::abs(angle) < 1e-6);

    std::cout << "Straight line motion test passed: speed = " << speed << ", angle = " << angle << std::endl;    
}

void test_left_line_motion() {
    double gear_ratio = 1.0;
    ModulePos position{1.0, 1.0};
    SimModule module(gear_ratio, position);

    Twist twist{0.0, 1.0, 0.0}; // Move left at 1 m/s
    double dt = 1.0; // 1 second time step

    module.update(twist, dt);

    double speed = module.getCurrentSpeed();
    double angle = module.getCurrentAngle();

    assert(std::abs(speed - 1.0) < 1e-6);
    assert(std::abs(angle - M_PI/2) < 1e-6);

    std::cout << "Left line motion test passed: speed = " << speed << ", angle = " << angle << std::endl;
}

void test_rotation_motion() {
    double gear_ratio = 1.0;
    ModulePos position{1.0, 1.0};
    SimModule module(gear_ratio, position);

    Twist twist{0.0, 0.0, 1.0}; // Rotate at 1 rad/s
    double dt = 1.0; // 1 second time step

    module.update(twist, dt);

    double speed = module.getCurrentSpeed();
    double angle = module.getCurrentAngle();

    assert(std::abs(speed + sqrt(2.0)) < 1e-6);
    assert(std::abs(angle + M_PI/4) < 1e-6);
    std::cout << "Rotation motion test passed: speed = " << speed << ", angle = " << angle << std::endl;
}

void test_multiple_updates() {
    double gear_ratio = 1.0;
    ModulePos position{1.0, 1.0};
    SimModule module(gear_ratio, position);

    Twist twist{1.0, 1.0, 0.0}; // Move forward at 1 m/s
    double dt = 1.0; // 1 second time step

    for (int i = 0; i < 20; ++i) {
        module.update(twist, dt);
    }

    double speed = module.getCurrentSpeed();
    double angle = module.getCurrentAngle();

    assert(std::abs(speed - sqrt(2.0)) < 1e-6);
    assert(std::abs(angle - M_PI/4) < 1e-6);
    std::cout << "Multiple updates test passed: speed = " << speed << ", angle = " << angle << std::endl;
}

int main(){
    test_straight_line_motion();
    test_left_line_motion();
    test_rotation_motion();
    test_multiple_updates();
    return 0;
}