#include<cassert>
#include<iostream>
#include<cmath>
#include<vector>
#include"swerve_robot.hpp"

using namespace swerve_drive_controller;

void test_swerve_robot_forward() {
    std::vector<ModuleConfig> module_configs = {
        {10.0, {1.0, 1.0}},
        {10.0, {-1.0, 1.0}},
        {10.0, {-1.0, -1.0}},
        {10.0, {1.0, -1.0}}
    };

    std::cout << "Testing SwerveRobot Forward Movement" << std::endl;

    SwerveRobot robot(module_configs);
    assert(robot.getModuleCount() == 4);

    Twist twist{1.0, 0.0, 0.0};
    double dt = 0.1;
    robot.update(twist, dt);

    double expected_speed = 1.0; 

    for (size_t i = 0; i < robot.getModuleCount(); ++i) {
        ModuleCommand state = robot.getModuleState(i);
        assert(std::abs(state.speed - expected_speed) < 1e-6);
        assert(std::abs(state.angle) < 1e-6); // Assuming angle is in radians and should be 0
        std::cout << "Module " << i << ": Speed = " << state.speed 
                  << ", Angle = " << state.angle << std::endl;
    }
}

void test_swerve_robot_left(){
    std::vector<ModuleConfig> module_configs = {
        {10.0, {1.0, 1.0}},
        {10.0, {-1.0, 1.0}},
        {10.0, {-1.0, -1.0}},
        {10.0, {1.0, -1.0}}
    };

    std::cout << "Testing SwerveRobot Left Movement" << std::endl;
    SwerveRobot robot(module_configs);
    assert(robot.getModuleCount() == 4);

    Twist twist{0.0, 1.0, 0.0};
    double dt = 0.1;
    robot.update(twist, dt);

    for (size_t i = 0; i < robot.getModuleCount(); ++i) {
        ModuleCommand state = robot.getModuleState(i);
        assert(std::abs(state.speed - 1.0) < 1e-6);
        assert(std::abs(state.angle - M_PI/2) < 1e-6);
        std::cout << "Module " << i << ": Speed = " << state.speed 
                  << ", Angle = " << state.angle << std::endl;
    }
}

void test_swerve_robot_rotate(){
    std::vector<ModuleConfig> module_configs = {
        {10.0, {1.0, 1.0}},
        {10.0, {-1.0, 1.0}},
        {10.0, {-1.0, -1.0}},
        {10.0, {1.0, -1.0}}
    };

    std::cout << "Testing SwerveRobot Rotate Movement" << std::endl;
    SwerveRobot robot(module_configs);
    assert(robot.getModuleCount() == 4);

    Twist twist{0.0, 0.0, 1.0};
    double dt = 0.1;
    robot.update(twist, dt);

    double expected_speed[4] = {-std::sqrt(2.0), -std::sqrt(2.0), std::sqrt(2.0), std::sqrt(2.0)};
    double expected_angle[4] = {-M_PI/4, M_PI/4, -M_PI/4, M_PI/4};

    for (size_t i = 0; i < robot.getModuleCount(); ++i) {
        ModuleCommand state = robot.getModuleState(i);
        assert(std::abs(state.speed - expected_speed[i]) < 1e-6);
        assert(std::abs(state.angle - expected_angle[i]) < 1e-6);
        std::cout << "Module " << i << ": Speed = " << state.speed
                << ", Angle = " << state.angle << std::endl;
    }
}

void test_pose_tracks_straight_motion() {
    std::vector<ModuleConfig> module_configs = {
        {10.0, {1.0, 1.0}}, {10.0, {-1.0, 1.0}},
        {10.0, {-1.0, -1.0}}, {10.0, {1.0, -1.0}}
    };
    SwerveRobot robot(module_configs);

    Twist twist{1.0, 0.0, 0.0};  // forward at 1 m/s
    double dt = 0.1;
    for (int i = 0; i < 10; ++i) {
        robot.update(twist, dt);
    }

    Pose pose = robot.getPose();
    // 10 ticks * 0.1s * 1.0 m/s = 1.0m forward, heading unchanged
    assert(std::abs(pose.x - 1.0) < 1e-3);
    assert(std::abs(pose.y - 0.0) < 1e-3);
    assert(std::abs(pose.theta - 0.0) < 1e-3);
    std::cout << "test_pose_tracks_straight_motion passed: x=" << pose.x 
               << " y=" << pose.y << " theta=" << pose.theta << std::endl;
}

void test_pose_tracks_left_motion() {
    std::vector<ModuleConfig> module_configs = {
        {10.0, {1.0, 1.0}}, {10.0, {-1.0, 1.0}},
        {10.0, {-1.0, -1.0}}, {10.0, {1.0, -1.0}}
    };
    SwerveRobot robot(module_configs);

    Twist twist{0.0, 1.0, 0.0};  // forward at 1 m/s
    double dt = 0.1;
    for (int i = 0; i < 10; ++i) {
        robot.update(twist, dt);
    }

    Pose pose = robot.getPose();
    // 10 ticks * 0.1s * 1.0 m/s = 1.0m forward, heading unchanged
    assert(std::abs(pose.x - 0.0) < 1e-3);
    assert(std::abs(pose.y - 1.0) < 1e-3);
    assert(std::abs(pose.theta - 0.0) < 1e-3);
    std::cout << "test_pose_tracks_straight_motion passed: x=" << pose.x 
               << " y=" << pose.y << " theta=" << pose.theta << std::endl;
}

void test_pose_tracks_rotation_motion() {
    std::vector<ModuleConfig> module_configs = {
        {10.0, {1.0, 1.0}}, {10.0, {-1.0, 1.0}},
        {10.0, {-1.0, -1.0}}, {10.0, {1.0, -1.0}}
    };
    SwerveRobot robot(module_configs);

    Twist twist{0.0, 0.0, 1.0};  // forward at 1 m/s
    double dt = 0.1;
    for (int i = 0; i < 10; ++i) {
        robot.update(twist, dt);
    }

    Pose pose = robot.getPose();
    // 10 ticks * 0.1s * 1.0 m/s = 1.0m forward, heading unchanged
    assert(std::abs(pose.x - 0.0) < 1e-3);
    assert(std::abs(pose.y - 0.0) < 1e-3);
    assert(std::abs(pose.theta - 1.0) < 1e-3);
    std::cout << "test_pose_tracks_straight_motion passed: x=" << pose.x 
               << " y=" << pose.y << " theta=" << pose.theta << std::endl;
}

int main(){
    test_swerve_robot_forward();
    test_swerve_robot_left();
    test_swerve_robot_rotate();

    test_pose_tracks_straight_motion();
    test_pose_tracks_left_motion();
    test_pose_tracks_rotation_motion();

    return 0;
}