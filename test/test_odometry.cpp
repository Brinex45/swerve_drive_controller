#include<cassert>
#include<iostream>
#include<cmath>
#include<vector>
#include"odometry.hpp"

using namespace swerve_drive_controller;

void test_odometry_straight_motion(){
    std::vector<ModuleState> module_states = {
        {{1.0, 0.0}, {1.0, 1.0}},
        {{1.0, 0.0}, {-1.0, 1.0}},
        {{1.0, 0.0}, {-1.0, -1.0}},
        {{1.0, 0.0}, {1.0, -1.0}}
    };

    Twist result = estimateTwist(module_states);

    assert(std::abs(result.vx - 1.0) < 1e-4);
    assert(std::abs(result.vy - 0.0) < 1e-4);
    assert(std::abs(result.omega - 0.0) < 1e-4);
    std::cout << "Straight motion test passed!" << std::endl;
}

void test_odometry_left_motion(){
    std::vector<ModuleState> module_states = {
        {{1.0, M_PI/2}, {1.0, 1.0}},
        {{1.0, M_PI/2}, {-1.0, 1.0}},
        {{1.0, M_PI/2}, {-1.0, -1.0}},
        {{1.0, M_PI/2}, {1.0, -1.0}}
    };

    Twist result = estimateTwist(module_states);

    assert(std::abs(result.vx - 0.0) < 1e-4);
    assert(std::abs(result.vy - 1.0) < 1e-4);
    assert(std::abs(result.omega - 0.0) < 1e-4);
    std::cout << "Left motion test passed!" << std::endl;
}

void test_straight_module_flip_motion(){
    std::vector<ModuleState> module_states = {
        {{-1.0, M_PI}, {1.0, 1.0}},
        {{1.0, 0.0}, {-1.0, 1.0}},
        {{-1.0, M_PI}, {-1.0, -1.0}},
        {{1.0, 0.0}, {1.0, -1.0}}
    };

    Twist result = estimateTwist(module_states);

    assert(std::abs(result.vx - 1.0) < 1e-4);
    assert(std::abs(result.vy - 0.0) < 1e-4);
    assert(std::abs(result.omega - 0.0) < 1e-4);
    std::cout << "Straight motion with fliped module test passed!" << std::endl;
}

void test_odometry_recovers_pure_rotation()  {
    std::vector<ModuleState> states = {
        {{-1.41421, -0.785398}, {1.0, 1.0}},
        {{-1.41421, 0.785398}, {-1.0, 1.0}},
        {{1.41421, -0.785398}, {-1.0, -1.0}},
        {{1.41421, 0.785398}, {1.0, -1.0}}
    };

    Twist result = estimateTwist(states);

    assert(std::abs(result.vx - 0.0) < 1e-4);
    assert(std::abs(result.vy - 0.0) < 1e-4);
    assert(std::abs(result.omega - 1.0) < 1e-4);
    std::cout << "test_odometry_recovers_pure_rotation passed: vx=" << result.vx 
               << " vy=" << result.vy << " omega=" << result.omega << std::endl;
}

int main(){
    test_odometry_straight_motion();
    test_odometry_left_motion();
    test_straight_module_flip_motion();
    test_odometry_recovers_pure_rotation();

    return 0;
}