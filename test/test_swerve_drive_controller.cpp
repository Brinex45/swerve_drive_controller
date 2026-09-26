#include <gmock/gmock.h>
#include <memory>
#include <iostream>
#include <vector>
#include <string>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl/time.h>
#include "swerve_drive_controller.hpp"

// Gives the test access to protected lifecycle methods.
// (protected in the base class -> public here, nothing else changes)
class TestableSwerveDriveController : public swerve_drive_controller::SwerveDriveController
{
public:
  using SwerveDriveController::command_interfaces_;
  using SwerveDriveController::state_interfaces_;
  using SwerveDriveController::on_init;
  using SwerveDriveController::on_configure;
  using SwerveDriveController::on_activate;
  using SwerveDriveController::update;
  using SwerveDriveController::received_velocity_msg_ptr_;  
};

class SwerveDriveControllerTest : public ::testing::Test
{
protected:
  // Runs ONCE for the whole test file, not per-test.
  static void SetUpTestCase()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  // Runs before EACH individual test below.
  void SetUp() override
  {
    controller_ = std::make_unique<TestableSwerveDriveController>();
  }

  void TearDown() override
  {
    if (controller_)
    {
      controller_->release_interfaces();
    }
    controller_.reset();
  }

  std::unique_ptr<TestableSwerveDriveController> controller_;
};

TEST_F(SwerveDriveControllerTest, InitSucceedsWithValidParams)
{
  auto result = controller_->init(
    "test_swerve_drive_controller",   // controller_name
    "",                                // urdf (empty is fine for this test)
    100,                               // cm_update_rate (Hz, arbitrary for a test)
    "",                                // node_namespace
    rclcpp::NodeOptions().parameter_overrides(
      {
        rclcpp::Parameter("module_type", "differential"),
        rclcpp::Parameter("modules", std::vector<std::string>{"test_module"}),
        rclcpp::Parameter("module.test_module.motor1_joint", "motor1_joint"),
        rclcpp::Parameter("module.test_module.motor2_joint", "motor2_joint"),
        rclcpp::Parameter("module.test_module.module_position_x", 0.0),
        rclcpp::Parameter("module.test_module.module_position_y", 0.0),
        rclcpp::Parameter("module.test_module.gear_ratio", 1.0),
        rclcpp::Parameter("module.test_module.wheel_radius", 0.05),
        rclcpp::Parameter("steer_pid.p", 1.0),
        rclcpp::Parameter("steer_pid.i", 0.0),
        rclcpp::Parameter("steer_pid.d", 0.0),
        rclcpp::Parameter("steer_pid.i_clamp_min", 0.0),
        rclcpp::Parameter("steer_pid.i_clamp_max", 1.0),
      }));

  ASSERT_EQ(result, controller_interface::return_type::OK);
}

TEST_F(SwerveDriveControllerTest, ConfigureSucceedsAfterInit)
{
  controller_->init(
    "test_swerve_drive_controller", "", 100, "",
    rclcpp::NodeOptions().parameter_overrides(
      {
        rclcpp::Parameter("module_type", "differential"),
        rclcpp::Parameter("modules", std::vector<std::string>{"test_module"}),
        rclcpp::Parameter("module.test_module.motor1_joint", "motor1_joint"),
        rclcpp::Parameter("module.test_module.motor2_joint", "motor2_joint"),
        rclcpp::Parameter("module.test_module.module_position_x", 0.0),
        rclcpp::Parameter("module.test_module.module_position_y", 0.0),
        rclcpp::Parameter("module.test_module.gear_ratio", 1.0),
        rclcpp::Parameter("module.test_module.wheel_radius", 0.05),
        rclcpp::Parameter("steer_pid.p", 1.0),
        rclcpp::Parameter("steer_pid.i", 0.0),
        rclcpp::Parameter("steer_pid.d", 0.0),
        rclcpp::Parameter("steer_pid.i_clamp_min", 0.0),
        rclcpp::Parameter("steer_pid.i_clamp_max", 1.0),
      }));

  auto result = controller_->on_configure(rclcpp_lifecycle::State());
  ASSERT_EQ(result, controller_interface::CallbackReturn::SUCCESS);
}

TEST_F(SwerveDriveControllerTest, ActivateSucceedsAfterConfigure)
{
  controller_->init(
    "test_swerve_drive_controller", "", 100, "",
    rclcpp::NodeOptions().parameter_overrides(
      {
        rclcpp::Parameter("module_type", "differential"),
        rclcpp::Parameter("modules", std::vector<std::string>{"test_module"}),
        rclcpp::Parameter("module.test_module.motor1_joint", "motor1_joint"),
        rclcpp::Parameter("module.test_module.motor2_joint", "motor2_joint"),
        rclcpp::Parameter("module.test_module.module_position_x", 0.0),
        rclcpp::Parameter("module.test_module.module_position_y", 0.0),
        rclcpp::Parameter("module.test_module.gear_ratio", 1.0),
        rclcpp::Parameter("module.test_module.wheel_radius", 0.05),
        rclcpp::Parameter("steer_pid.p", 1.0),
        rclcpp::Parameter("steer_pid.i", 0.0),
        rclcpp::Parameter("steer_pid.d", 0.0),
        rclcpp::Parameter("steer_pid.i_clamp_min", 0.0),
        rclcpp::Parameter("steer_pid.i_clamp_max", 1.0),
      }));

  auto configure_result = controller_->on_configure(rclcpp_lifecycle::State());
  ASSERT_EQ(configure_result, controller_interface::CallbackReturn::SUCCESS);
  // std::cerr << "CHECKPOINT: on_configure done" << std::endl;

  // motor1: velocity (command) — you already have this one
  hardware_interface::InterfaceInfo motor1_vel_cmd_info;
  motor1_vel_cmd_info.name = hardware_interface::HW_IF_VELOCITY;
  hardware_interface::InterfaceDescription motor1_vel_cmd_desc("motor1_joint", motor1_vel_cmd_info);
  hardware_interface::CommandInterface motor1_vel_cmd(motor1_vel_cmd_desc);

  // motor1: velocity (state)
  hardware_interface::InterfaceInfo motor1_vel_state_info;
  motor1_vel_state_info.name = hardware_interface::HW_IF_VELOCITY;
  hardware_interface::InterfaceDescription motor1_vel_state_desc("motor1_joint", motor1_vel_state_info);
  hardware_interface::StateInterface motor1_vel_state(motor1_vel_state_desc);

  // motor1: position (state)
  hardware_interface::InterfaceInfo motor1_pos_state_info;
  motor1_pos_state_info.name = hardware_interface::HW_IF_POSITION;
  hardware_interface::InterfaceDescription motor1_pos_state_desc("motor1_joint", motor1_pos_state_info);
  hardware_interface::StateInterface motor1_pos_state(motor1_pos_state_desc);

  // motor2: velocity (command) — you already have this one
  hardware_interface::InterfaceInfo motor2_vel_cmd_info;
  motor2_vel_cmd_info.name = hardware_interface::HW_IF_VELOCITY;
  hardware_interface::InterfaceDescription motor2_vel_cmd_desc("motor2_joint", motor2_vel_cmd_info);
  hardware_interface::CommandInterface motor2_vel_cmd(motor2_vel_cmd_desc);

  // motor2: velocity (state)
  hardware_interface::InterfaceInfo motor2_vel_state_info;
  motor2_vel_state_info.name = hardware_interface::HW_IF_VELOCITY;
  hardware_interface::InterfaceDescription motor2_vel_state_desc("motor2_joint", motor2_vel_state_info);
  hardware_interface::StateInterface motor2_vel_state(motor2_vel_state_desc);

  // motor2: position (state)
  hardware_interface::InterfaceInfo motor2_pos_state_info;
  motor2_pos_state_info.name = hardware_interface::HW_IF_POSITION;
  hardware_interface::InterfaceDescription motor2_pos_state_desc("motor2_joint", motor2_pos_state_info);
  hardware_interface::StateInterface motor2_pos_state(motor2_pos_state_desc);

  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  command_interfaces.emplace_back(motor1_vel_cmd);
  command_interfaces.emplace_back(motor2_vel_cmd);

  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  state_interfaces.emplace_back(motor1_vel_state);
  state_interfaces.emplace_back(motor1_pos_state);
  state_interfaces.emplace_back(motor2_vel_state);
  state_interfaces.emplace_back(motor2_pos_state);

  controller_->assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));
  auto activate_result = controller_->on_activate(rclcpp_lifecycle::State());
  // std::cerr << "CHECKPOINT: on_activate returned" << std::endl;

  ASSERT_EQ(activate_result, controller_interface::CallbackReturn::SUCCESS);

  // TearDown();
}

TEST_F(SwerveDriveControllerTest, UpdateSucceedsAfterActivate)
{
  controller_->init(
    "test_swerve_drive_controller", "", 100, "",
    rclcpp::NodeOptions().parameter_overrides(
      {
        rclcpp::Parameter("module_type", "differential"),
        rclcpp::Parameter("modules", std::vector<std::string>{"test_module"}),
        rclcpp::Parameter("module.test_module.motor1_joint", "motor1_joint"),
        rclcpp::Parameter("module.test_module.motor2_joint", "motor2_joint"),
        rclcpp::Parameter("module.test_module.module_position_x", 0.0),
        rclcpp::Parameter("module.test_module.module_position_y", 0.0),
        rclcpp::Parameter("module.test_module.gear_ratio", 1.0),
        rclcpp::Parameter("module.test_module.wheel_radius", 0.05),
        rclcpp::Parameter("steer_pid.p", 1.0),
        rclcpp::Parameter("steer_pid.i", 0.0),
        rclcpp::Parameter("steer_pid.d", 0.0),
        rclcpp::Parameter("steer_pid.i_clamp_min", 0.0),
        rclcpp::Parameter("steer_pid.i_clamp_max", 1.0),
      }));

  auto configure_result = controller_->on_configure(rclcpp_lifecycle::State());
  ASSERT_EQ(configure_result, controller_interface::CallbackReturn::SUCCESS);

  hardware_interface::InterfaceInfo motor1_vel_cmd_info;
  motor1_vel_cmd_info.name = hardware_interface::HW_IF_VELOCITY;
  hardware_interface::InterfaceDescription motor1_vel_cmd_desc("motor1_joint", motor1_vel_cmd_info);
  hardware_interface::CommandInterface motor1_vel_cmd(motor1_vel_cmd_desc);

  // motor1: velocity (state)
  hardware_interface::InterfaceInfo motor1_vel_state_info;
  motor1_vel_state_info.name = hardware_interface::HW_IF_VELOCITY;
  hardware_interface::InterfaceDescription motor1_vel_state_desc("motor1_joint", motor1_vel_state_info);
  hardware_interface::StateInterface motor1_vel_state(motor1_vel_state_desc);
  bool ok1 = motor1_vel_state.set_value(0.0);
  ASSERT_TRUE(ok1);

  // motor1: position (state)
  hardware_interface::InterfaceInfo motor1_pos_state_info;
  motor1_pos_state_info.name = hardware_interface::HW_IF_POSITION;
  hardware_interface::InterfaceDescription motor1_pos_state_desc("motor1_joint", motor1_pos_state_info);
  hardware_interface::StateInterface motor1_pos_state(motor1_pos_state_desc);
  bool ok2 = motor1_pos_state.set_value(0.0);
  ASSERT_TRUE(ok2);

  // motor2: velocity (command) — you already have this one
  hardware_interface::InterfaceInfo motor2_vel_cmd_info;
  motor2_vel_cmd_info.name = hardware_interface::HW_IF_VELOCITY;
  hardware_interface::InterfaceDescription motor2_vel_cmd_desc("motor2_joint", motor2_vel_cmd_info);
  hardware_interface::CommandInterface motor2_vel_cmd(motor2_vel_cmd_desc);

  // motor2: velocity (state)
  hardware_interface::InterfaceInfo motor2_vel_state_info;
  motor2_vel_state_info.name = hardware_interface::HW_IF_VELOCITY;
  hardware_interface::InterfaceDescription motor2_vel_state_desc("motor2_joint", motor2_vel_state_info);
  hardware_interface::StateInterface motor2_vel_state(motor2_vel_state_desc);
  bool ok3 = motor2_vel_state.set_value(0.0);
  ASSERT_TRUE(ok3);

  // motor2: position (state)
  hardware_interface::InterfaceInfo motor2_pos_state_info;
  motor2_pos_state_info.name = hardware_interface::HW_IF_POSITION;
  hardware_interface::InterfaceDescription motor2_pos_state_desc("motor2_joint", motor2_pos_state_info);
  hardware_interface::StateInterface motor2_pos_state(motor2_pos_state_desc);
  bool ok4 = motor2_pos_state.set_value(0.0);
  ASSERT_TRUE(ok4);

  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  command_interfaces.emplace_back(motor1_vel_cmd);
  command_interfaces.emplace_back(motor2_vel_cmd);

  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  state_interfaces.emplace_back(motor1_vel_state);
  state_interfaces.emplace_back(motor1_pos_state);
  state_interfaces.emplace_back(motor2_vel_state);
  state_interfaces.emplace_back(motor2_pos_state);

  controller_->assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));
  auto activate_result = controller_->on_activate(rclcpp_lifecycle::State());

  ASSERT_EQ(activate_result, controller_interface::CallbackReturn::SUCCESS);

  auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_msg->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  twist_msg->twist.linear.x = 0.1;   // pure forward motion, no strafe, no rotation
  twist_msg->twist.linear.y = 0.0;
  twist_msg->twist.angular.z = 0.0;

  controller_->received_velocity_msg_ptr_.writeFromNonRT(twist_msg);

  auto update_result = controller_->update(rclcpp::Clock(RCL_ROS_TIME).now(), rclcpp::Duration(0, 10000000)); // 10ms period
  ASSERT_EQ(update_result, controller_interface::return_type::OK);

  auto motor1_cmd_value = motor1_vel_cmd.get_optional();
  auto motor2_cmd_value = motor2_vel_cmd.get_optional();
  ASSERT_TRUE(motor1_cmd_value.has_value());
  ASSERT_TRUE(motor2_cmd_value.has_value());

  EXPECT_NEAR(motor1_cmd_value.value(), 1.0, 1e-6);
  EXPECT_NEAR(motor2_cmd_value.value(), 1.0, 1e-6);
}