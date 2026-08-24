#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <control_toolbox/pid.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "swerve_drive_controller_parameters.hpp"  // auto-generated from your YAML
#include "swerve_ik.hpp"
#include "differential_mixer.hpp"
#include "azimuth_estimator.hpp"
#include "odometry.hpp"

namespace swerve_drive_controller
{

struct PID
{
  double p = 0.0;
  double i = 0.0;
  double d = 0.0;
  double i_clamp_min = 0.0;
  double i_clamp_max = 0.0;
};

struct Module
{
  std::string motor1_joint = "";
  std::string motor2_joint = "";

  double module_position_x = 0.0;
  double module_position_y = 0.0;

  double gear_ratio = 0.0;
  double wheel_radius = 0.0;

  PID pid_gains;
  control_toolbox::Pid pid;

  hardware_interface::LoanedCommandInterface * motor1_cmd = nullptr;
  hardware_interface::LoanedCommandInterface * motor2_cmd = nullptr;
  hardware_interface::LoanedStateInterface * motor1_state = nullptr;
  hardware_interface::LoanedStateInterface * motor2_state = nullptr;
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

private:
  std::shared_ptr<swerve_drive_controller::ParamListener> param_listener_;
  swerve_drive_controller::Params params_;

  std::string module_type_ = "";
  std::vector<Module> modules_;

  bool tf_frame_prefix_enable_ = true;
  std::string tf_frame_prefix_ = "";
  std::string odom_frame_id_ = "odom";
  std::string base_frame_id_ = "base_link";
  double pose_covariance_diagonal_[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double twist_covariance_diagonal_[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  bool open_loop_ = false;
  bool position_feedback_ = true;
  bool enable_odom_tf_ = true;

  double cmd_vel_timeout_ = 0.5;

  Pose pose_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscriber_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::TwistStamped>>
    received_velocity_msg_ptr_{nullptr};
  rclcpp::Time previous_update_timestamp_{0};

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_;
};

}  // namespace swerve_drive_controller