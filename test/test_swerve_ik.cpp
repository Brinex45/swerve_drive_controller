#include <cassert>
#include <cmath>
#include <iostream>
#include "swerve_ik.hpp"

using namespace swerve_drive_controller;
using namespace std;

void test_zero_twist() {
    Twist twist{0.0, 0.0, 0.0};       // no movement
    ModulePos pos{1.0, 1.0};          // module at (1, 1)

    ModuleCommand cmd = computeModuleCommand(twist, pos);

    assert(std::abs(cmd.speed - 0.0) < 1e-6);
    assert(std::abs(cmd.angle - 0.0) < 1e-6);
    cout << "test_zero_twist passed\n";
}

void test_pure_translation_at_center() {
    Twist twist{1.0, 0.0, 0.0};       // moving forward, no rotation
    ModulePos pos{0.0, 0.0};          // module AT center

    ModuleCommand cmd = computeModuleCommand(twist, pos);

    assert(std::abs(cmd.speed - 1.0) < 1e-6);
    assert(std::abs(cmd.angle - 0.0) < 1e-6);
    cout << "test_pure_translation_at_center passed\n";
}

void test_pure_translation_at_offset() {
    Twist twist{1.0, 0.0, 0.0};       // moving forward, no rotation
    ModulePos pos{1.0, 1.0};          // module at (1, 1)

    ModuleCommand cmd = computeModuleCommand(twist, pos);

    assert(std::abs(cmd.speed - 1.0) < 1e-6);
    assert(std::abs(cmd.angle - 0.0) < 1e-6);
    cout << "test_pure_translation_at_offset passed\n";
}

void test_pure_rotation() {
    Twist twist{0.0, 0.0, M_PI};       // rotating in place
    ModulePos pos{1.0, 0.0};          // module at (1, 0)

    ModuleCommand cmd = computeModuleCommand(twist, pos);

    assert(std::abs(cmd.speed - M_PI) < 1e-6);
    assert(std::abs(cmd.angle - M_PI/2) < 1e-6); // Should be facing left
    cout << "test_pure_rotation passed\n";
}

void test_translation_and_rotation() {
    Twist twist{1.0, 0.0, M_PI};       // moving forward and rotating
    ModulePos pos{1.0, 1.0};          // module at (1, 1)

    ModuleCommand cmd = computeModuleCommand(twist, pos);

    double expected_vx = 1.0 - M_PI * 1.0; // vx_total = vx + (-omega * y)
    double expected_vy = 0.0 + M_PI * 1.0; // vy_total = vy + (omega * x)
    double expected_speed = std::sqrt(expected_vx * expected_vx + expected_vy * expected_vy);
    double expected_angle = std::atan2(expected_vy, expected_vx);

    assert(std::abs(cmd.speed - expected_speed) < 1e-6);
    assert(std::abs(cmd.angle - expected_angle) < 1e-6);
    cout << "test_translation_and_rotation passed\n";
}

void test_optimize_steer_direction() {
    ModuleCommand target{1.0, M_PI/2}; // target speed and angle
    double current_angle = 0.0;        // current angle

    ModuleCommand optimized = optimizeSteerDirection(target, current_angle);

    // The optimized angle should be the same as the target angle since it's within 90 degrees
    assert(std::abs(optimized.angle - target.angle) < 1e-6);
    assert(std::abs(optimized.speed - target.speed) < 1e-6);
    cout << "test_optimize_steer_direction passed\n";
}

void test_optimize_steer_direction_reverse() {
    ModuleCommand target{1.0, M_PI}; // target speed and angle
    double current_angle = 0.0;      // current angle

    ModuleCommand optimized = optimizeSteerDirection(target, current_angle);

    // The optimized angle should be the target angle + PI and speed should be negated
    assert(std::abs(optimized.angle - SwerveIK::normalizeAngle(target.angle + M_PI)) < 1e-6);
    assert(std::abs(optimized.speed + target.speed) < 1e-6);
    cout << "test_optimize_steer_direction_reverse passed\n";
}

void test_optimize_steer_direction_small_diff_case() {
    ModuleCommand target{1.0, 10 * 3.14/180}; // target speed and angle
    double current_angle = 5 * 3.14/180;    // current angle

    ModuleCommand optimized = optimizeSteerDirection(target, current_angle);

    // The optimized angle should be the target angle + PI and speed should be negated
    assert(std::abs(optimized.angle - target.angle) < 1e-6);
    assert(std::abs(optimized.speed - target.speed) < 1e-6);
    cout << "test_optimize_steer_direction_small_diff_case passed\n";
}

void test_optimize_steer_direction_large_diff_case() {
    ModuleCommand target{1.0, 0 * 3.14/180}; // target speed and angle
    double current_angle = 170 * 3.14/180;    // current angle

    ModuleCommand optimized = optimizeSteerDirection(target, current_angle);

    // The optimized angle should be the target angle + PI and speed should be negated
    assert(std::abs(optimized.angle - SwerveIK::normalizeAngle(target.angle + M_PI)) < 1e-6);
    assert(std::abs(optimized.speed + target.speed) < 1e-6);
    cout << "test_optimize_steer_direction_large_diff_case passed\n";
}

void test_optimize_steer_direction_wraparound_case() {
    ModuleCommand target{1.0, 5 * 3.14/180}; // target speed and angle
    double current_angle = 355 * 3.14/180;    // current angle

    ModuleCommand optimized = optimizeSteerDirection(target, current_angle);

    // The optimized angle should be the target angle + PI and speed should be negated
    assert(std::abs(optimized.angle - target.angle) < 1e-6);
    assert(std::abs(optimized.speed - target.speed) < 1e-6);
    cout << "test_optimize_steer_direction_wraparound_case passed\n";
}

int main() {

    test_zero_twist();
    test_pure_translation_at_center();
    test_pure_translation_at_offset();
    test_pure_rotation();
    test_translation_and_rotation();
    test_optimize_steer_direction();
    test_optimize_steer_direction_reverse();
    test_optimize_steer_direction_small_diff_case();
    test_optimize_steer_direction_large_diff_case();
    test_optimize_steer_direction_wraparound_case();

    return 0;
}