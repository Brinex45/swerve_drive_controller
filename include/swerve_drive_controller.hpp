#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include <controller_interface/controller_interface.hpp>
#include <control_toolbox/pid.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "swerve_drive_controller_parameters.hpp"  // auto-generated from your YAML
#include "swerve_ik.hpp"
#include "differential_mixer.hpp"
#include "azimuth_estimator.hpp"
#include "odometry.hpp"

namespace swerve_drive_controller
{

// Steer PID gains. Currently one shared set (params_.steer_pid) applied to
// every module — see per-Module control_toolbox::Pid below for why each
// module still needs its own runtime instance despite sharing gains.
struct PID
{
  double p = 0.0;
  double i = 0.0;
  double d = 0.0;
  double i_clamp_min = 0.0;
  double i_clamp_max = 0.0;
};

inline control_toolbox::AntiWindupStrategy make_none_antiwindup_strategy()
{
  control_toolbox::AntiWindupStrategy strategy;
  strategy.type = control_toolbox::AntiWindupStrategy::NONE;
  return strategy;
}

// Runtime state for a single swerve module. Populated by copying validated
// values out of params_ during on_configure() — see design note in chat:
// params_ is not used directly in the hot control loop because (a) it can be
// live-updated out from under a running update() call, and (b) our own
// math library (swerve_ik.hpp etc.) is intentionally ROS-agnostic and takes
// plain structs like this one, not generate_parameter_library types.
struct Module
{
  std::string motor1_joint = "";
  std::string motor2_joint = "";

  double module_position_x = 0.0;
  double module_position_y = 0.0;

  double gear_ratio = 0.0;
  double wheel_radius = 0.0;

  // Each module gets its OWN control_toolbox::Pid instance, even though all
  // modules currently share the same gains (pid_gains, copied from the
  // single global params_.steer_pid). This is necessary because Pid objects
  // hold internal integrator state; sharing one instance across modules
  // would let one module's windup bleed into another's.
  PID pid_gains;
  control_toolbox::Pid pid{
    0.0, 0.0, 0.0,
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    make_none_antiwindup_strategy()};

  // Interface handles: raw pointers into controller_manager-owned interface
  // vectors, resolved once in on_activate() and cleared in on_deactivate().
  // Not owned by Module — never freed here.
  hardware_interface::LoanedCommandInterface * motor1_cmd = nullptr;
  hardware_interface::LoanedCommandInterface * motor2_cmd = nullptr;

  // Position state is unconditionally required (see chat: position feedback
  // was removed as a toggle, since both differential azimuth estimation and
  // non-differential direct steer readback need it — there is no supported
  // position-feedback-free mode).
  hardware_interface::LoanedStateInterface * motor1_velocity_state = nullptr;
  hardware_interface::LoanedStateInterface * motor1_position_state = nullptr;
  hardware_interface::LoanedStateInterface * motor2_velocity_state = nullptr;
  hardware_interface::LoanedStateInterface * motor2_position_state = nullptr;
};

class SwerveDriveController : public controller_interface::ControllerInterface
{
public:
  SwerveDriveController() = default;

  // --- lifecycle callbacks ---
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // --- ControllerInterface-specific: no LifecycleNode equivalent ---
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // --- replaces timer callback ---
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // --- generated parameter access ---
  std::shared_ptr<swerve_drive_controller::ParamListener> param_listener_;
  swerve_drive_controller::Params params_;

  // --- our own runtime state, copied from params_ at configure time ---
  std::string module_type_ = "";
  std::vector<Module> modules_;

  bool tf_frame_prefix_enable_ = true;
  std::string tf_frame_prefix_ = "";
  std::string odom_frame_id_ = "odom";
  std::string base_frame_id_ = "base_link";
  double pose_covariance_diagonal_[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double twist_covariance_diagonal_[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Only meaningful for the drive side of a non-differential module — a
  // differential module's drive+steer are coupled through DifferentialMixer,
  // so there is no independent "commanded drive value" to substitute for
  // feedback the way there is for a plain non-differential drive joint.
  // Enforced in update()'s odometry step, not here.
  bool open_loop_odometry_ = false;
  bool enable_odom_tf_ = true;

  double cmd_vel_timeout_ = 0.5;

  // --- odometry: accumulates pose from module feedback each update() ---
  // Twist estimation (estimateTwist) and pose accumulation (integratePose)
  // both live in odometry.hpp, shared with the simulation (SwerveRobot).
  // update() will call estimateTwist() on this cycle's module feedback,
  // then integratePose(pose_, that_twist, period) to advance pose_.
  Pose pose_;

  // --- cmd_vel subscription (realtime-safe handoff) ---
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscriber_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::TwistStamped>>
    received_velocity_msg_ptr_{nullptr};
  rclcpp::Time previous_update_timestamp_{0};
  // Guards against acting on cmd_vel messages received while the controller
  // is not active (e.g. configured but not yet activated, or deactivated).
  // Set true in on_activate(), false in on_deactivate().
  bool subscriber_is_active_ = false;

  // --- odometry publishing ---
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_;
};

}  // namespace swerve_drive_controller