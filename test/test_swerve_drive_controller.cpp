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
  using SwerveDriveController::realtime_odometry_publisher_;             // add this
  using SwerveDriveController::realtime_odometry_transform_publisher_;   // add this
};

class SwerveDriveControllerTest : public ::testing::Test
{
protected:
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

  static hardware_interface::StateInterface make_state_interface(
    const std::string & joint_name, const char * interface_type, double initial_value)
  {
    hardware_interface::InterfaceInfo info;
    info.name = interface_type;
    hardware_interface::InterfaceDescription desc(joint_name, info);
    hardware_interface::StateInterface iface(desc);
    bool ok = iface.set_value(initial_value);
    EXPECT_TRUE(ok);
    return iface;
  }

  static hardware_interface::CommandInterface make_command_interface(
    const std::string & joint_name, const char * interface_type)
  {
    hardware_interface::InterfaceInfo info;
    info.name = interface_type;
    hardware_interface::InterfaceDescription desc(joint_name, info);
    return hardware_interface::CommandInterface(desc);
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

  auto motor1_vel_cmd = make_command_interface("motor1_joint", hardware_interface::HW_IF_VELOCITY);
  auto motor2_vel_cmd = make_command_interface("motor2_joint", hardware_interface::HW_IF_VELOCITY);
  auto motor1_vel_state = make_state_interface("motor1_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto motor1_pos_state = make_state_interface("motor1_joint", hardware_interface::HW_IF_POSITION, 0.0);
  auto motor2_vel_state = make_state_interface("motor2_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto motor2_pos_state = make_state_interface("motor2_joint", hardware_interface::HW_IF_POSITION, 0.0);

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

  auto motor1_vel_cmd = make_command_interface("motor1_joint", hardware_interface::HW_IF_VELOCITY);
  auto motor2_vel_cmd = make_command_interface("motor2_joint", hardware_interface::HW_IF_VELOCITY);
  auto motor1_vel_state = make_state_interface("motor1_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto motor1_pos_state = make_state_interface("motor1_joint", hardware_interface::HW_IF_POSITION, 0.0);
  auto motor2_vel_state = make_state_interface("motor2_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto motor2_pos_state = make_state_interface("motor2_joint", hardware_interface::HW_IF_POSITION, 0.0);

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

TEST_F(SwerveDriveControllerTest, UpdateAppliesSteerCorrectionForDiagonalCommand)
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

  auto motor1_vel_cmd = make_command_interface("motor1_joint", hardware_interface::HW_IF_VELOCITY);
  auto motor2_vel_cmd = make_command_interface("motor2_joint", hardware_interface::HW_IF_VELOCITY);
  auto motor1_vel_state = make_state_interface("motor1_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto motor1_pos_state = make_state_interface("motor1_joint", hardware_interface::HW_IF_POSITION, 0.0);
  auto motor2_vel_state = make_state_interface("motor2_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto motor2_pos_state = make_state_interface("motor2_joint", hardware_interface::HW_IF_POSITION, 0.0);

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
  twist_msg->twist.linear.x = 0.1;
  twist_msg->twist.linear.y = 0.05;   // diagonal — forces a genuine steer angle
  twist_msg->twist.angular.z = 0.0;

  controller_->received_velocity_msg_ptr_.writeFromNonRT(twist_msg);

  auto update_result = controller_->update(rclcpp::Clock(RCL_ROS_TIME).now(), rclcpp::Duration(0, 10000000));
  ASSERT_EQ(update_result, controller_interface::return_type::OK);

  auto motor1_cmd_value = motor1_vel_cmd.get_optional();
  auto motor2_cmd_value = motor2_vel_cmd.get_optional();
  ASSERT_TRUE(motor1_cmd_value.has_value());
  ASSERT_TRUE(motor2_cmd_value.has_value());

  EXPECT_NEAR(motor1_cmd_value.value(), 1.3498578, 1e-4);
  EXPECT_NEAR(motor2_cmd_value.value(), 0.8862102, 1e-4);
}

TEST_F(SwerveDriveControllerTest, UpdateSafeStopsOnStaleCommand)
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
        rclcpp::Parameter("cmd_vel_timeout", 0.5),   // 500ms timeout — the new piece
      }));

  auto configure_result = controller_->on_configure(rclcpp_lifecycle::State());
  ASSERT_EQ(configure_result, controller_interface::CallbackReturn::SUCCESS);

  auto motor1_vel_cmd = make_command_interface("motor1_joint", hardware_interface::HW_IF_VELOCITY);
  auto motor2_vel_cmd = make_command_interface("motor2_joint", hardware_interface::HW_IF_VELOCITY);
  auto motor1_vel_state = make_state_interface("motor1_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto motor1_pos_state = make_state_interface("motor1_joint", hardware_interface::HW_IF_POSITION, 0.0);
  auto motor2_vel_state = make_state_interface("motor2_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto motor2_pos_state = make_state_interface("motor2_joint", hardware_interface::HW_IF_POSITION, 0.0);

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

  // Stamp the message 1 second in the past — older than the 0.5s timeout.
  auto now = rclcpp::Clock(RCL_ROS_TIME).now();
  auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_msg->header.stamp = now - rclcpp::Duration::from_seconds(1.0);
  twist_msg->twist.linear.x = 0.1;   // nonzero on purpose — if the timeout
  twist_msg->twist.linear.y = 0.0;   // check is broken, this would produce
  twist_msg->twist.angular.z = 0.0;  // nonzero motor commands, catching the bug

  controller_->received_velocity_msg_ptr_.writeFromNonRT(twist_msg);

  auto update_result = controller_->update(now, rclcpp::Duration(0, 10000000));
  ASSERT_EQ(update_result, controller_interface::return_type::OK);

  auto motor1_cmd_value = motor1_vel_cmd.get_optional();
  auto motor2_cmd_value = motor2_vel_cmd.get_optional();
  ASSERT_TRUE(motor1_cmd_value.has_value());
  ASSERT_TRUE(motor2_cmd_value.has_value());

  // Stale command -> safe stop -> both motors should be exactly zero.
  EXPECT_NEAR(motor1_cmd_value.value(), 0.0, 1e-6);
  EXPECT_NEAR(motor2_cmd_value.value(), 0.0, 1e-6);
}

TEST_F(SwerveDriveControllerTest, UpdateMultipleModulesCommands)
{
  controller_->init(
    "test_swerve_drive_controller", "", 100, "",
    rclcpp::NodeOptions().parameter_overrides(
      {
        rclcpp::Parameter("module_type", "differential"),
        rclcpp::Parameter("modules", std::vector<std::string>{"module_a", "module_b", "module_c"}),

        rclcpp::Parameter("module.module_a.motor1_joint", "module_a_motor1_joint"),
        rclcpp::Parameter("module.module_a.motor2_joint", "module_a_motor2_joint"),
        rclcpp::Parameter("module.module_a.module_position_x", 0.2),
        rclcpp::Parameter("module.module_a.module_position_y", 0.0),
        rclcpp::Parameter("module.module_a.gear_ratio", 1.0),
        rclcpp::Parameter("module.module_a.wheel_radius", 0.05),

        rclcpp::Parameter("module.module_b.motor1_joint", "module_b_motor1_joint"),
        rclcpp::Parameter("module.module_b.motor2_joint", "module_b_motor2_joint"),
        rclcpp::Parameter("module.module_b.module_position_x", -0.1),
        rclcpp::Parameter("module.module_b.module_position_y", 0.173205),
        rclcpp::Parameter("module.module_b.gear_ratio", 1.0),
        rclcpp::Parameter("module.module_b.wheel_radius", 0.05),

        rclcpp::Parameter("module.module_c.motor1_joint", "module_c_motor1_joint"),
        rclcpp::Parameter("module.module_c.motor2_joint", "module_c_motor2_joint"),
        rclcpp::Parameter("module.module_c.module_position_x", -0.1),
        rclcpp::Parameter("module.module_c.module_position_y", -0.173205),
        rclcpp::Parameter("module.module_c.gear_ratio", 1.0),
        rclcpp::Parameter("module.module_c.wheel_radius", 0.05),

        rclcpp::Parameter("steer_pid.p", 1.0),
        rclcpp::Parameter("steer_pid.i", 0.0),
        rclcpp::Parameter("steer_pid.d", 0.0),
        rclcpp::Parameter("steer_pid.i_clamp_min", 0.0),
        rclcpp::Parameter("steer_pid.i_clamp_max", 1.0),
        rclcpp::Parameter("cmd_vel_timeout", 0.5),
      }));

  auto configure_result = controller_->on_configure(rclcpp_lifecycle::State());
  ASSERT_EQ(configure_result, controller_interface::CallbackReturn::SUCCESS);

  auto module_a_motor1_cmd = make_command_interface("module_a_motor1_joint", hardware_interface::HW_IF_VELOCITY);
  auto module_a_motor2_cmd = make_command_interface("module_a_motor2_joint", hardware_interface::HW_IF_VELOCITY);
  auto module_a_motor1_vel = make_state_interface("module_a_motor1_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto module_a_motor1_pos = make_state_interface("module_a_motor1_joint", hardware_interface::HW_IF_POSITION, 0.0);
  auto module_a_motor2_vel = make_state_interface("module_a_motor2_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto module_a_motor2_pos = make_state_interface("module_a_motor2_joint", hardware_interface::HW_IF_POSITION, 0.0);

  auto module_b_motor1_cmd = make_command_interface("module_b_motor1_joint", hardware_interface::HW_IF_VELOCITY);
  auto module_b_motor2_cmd = make_command_interface("module_b_motor2_joint", hardware_interface::HW_IF_VELOCITY);
  auto module_b_motor1_vel = make_state_interface("module_b_motor1_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto module_b_motor1_pos = make_state_interface("module_b_motor1_joint", hardware_interface::HW_IF_POSITION, 0.0);
  auto module_b_motor2_vel = make_state_interface("module_b_motor2_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto module_b_motor2_pos = make_state_interface("module_b_motor2_joint", hardware_interface::HW_IF_POSITION, 0.0);

  auto module_c_motor1_cmd = make_command_interface("module_c_motor1_joint", hardware_interface::HW_IF_VELOCITY);
  auto module_c_motor2_cmd = make_command_interface("module_c_motor2_joint", hardware_interface::HW_IF_VELOCITY);
  auto module_c_motor1_vel = make_state_interface("module_c_motor1_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto module_c_motor1_pos = make_state_interface("module_c_motor1_joint", hardware_interface::HW_IF_POSITION, 0.0);
  auto module_c_motor2_vel = make_state_interface("module_c_motor2_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto module_c_motor2_pos = make_state_interface("module_c_motor2_joint", hardware_interface::HW_IF_POSITION, 0.0);

  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  command_interfaces.emplace_back(module_a_motor1_cmd);
  command_interfaces.emplace_back(module_a_motor2_cmd);
  command_interfaces.emplace_back(module_b_motor1_cmd);
  command_interfaces.emplace_back(module_b_motor2_cmd);
  command_interfaces.emplace_back(module_c_motor1_cmd);
  command_interfaces.emplace_back(module_c_motor2_cmd);

  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  state_interfaces.emplace_back(module_a_motor1_vel);
  state_interfaces.emplace_back(module_a_motor1_pos);
  state_interfaces.emplace_back(module_a_motor2_vel);
  state_interfaces.emplace_back(module_a_motor2_pos);
  state_interfaces.emplace_back(module_b_motor1_vel);
  state_interfaces.emplace_back(module_b_motor1_pos);
  state_interfaces.emplace_back(module_b_motor2_vel);
  state_interfaces.emplace_back(module_b_motor2_pos);
  state_interfaces.emplace_back(module_c_motor1_vel);
  state_interfaces.emplace_back(module_c_motor1_pos);
  state_interfaces.emplace_back(module_c_motor2_vel);
  state_interfaces.emplace_back(module_c_motor2_pos);

  controller_->assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));
  auto activate_result = controller_->on_activate(rclcpp_lifecycle::State());
  ASSERT_EQ(activate_result, controller_interface::CallbackReturn::SUCCESS);

  auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_msg->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  twist_msg->twist.linear.x = 0.05;
  twist_msg->twist.linear.y = 0.0;
  twist_msg->twist.angular.z = 1.0;

  controller_->received_velocity_msg_ptr_.writeFromNonRT(twist_msg);

  auto update_result = controller_->update(rclcpp::Clock(RCL_ROS_TIME).now(), rclcpp::Duration(0, 10000000));
  ASSERT_EQ(update_result, controller_interface::return_type::OK);

  auto module_a_motor1_value = module_a_motor1_cmd.get_optional();
  auto module_a_motor2_value = module_a_motor2_cmd.get_optional();
  ASSERT_TRUE(module_a_motor1_value.has_value());
  ASSERT_TRUE(module_a_motor2_value.has_value());
  EXPECT_NEAR(module_a_motor1_value.value(), 2.7244617, 1e-4);
  EXPECT_NEAR(module_a_motor2_value.value(), 1.3986440, 1e-4);

  auto module_b_motor1_value = module_b_motor1_cmd.get_optional();
  auto module_b_motor2_value = module_b_motor2_cmd.get_optional();
  ASSERT_TRUE(module_b_motor1_value.has_value());
  ASSERT_TRUE(module_b_motor2_value.has_value());
  EXPECT_NEAR(module_b_motor1_value.value(), -1.2459003, 1e-4);
  EXPECT_NEAR(module_b_motor2_value.value(), -1.9277079, 1e-4);

  auto module_c_motor1_value = module_c_motor1_cmd.get_optional();
  auto module_c_motor2_value = module_c_motor2_cmd.get_optional();
  ASSERT_TRUE(module_c_motor1_value.has_value());
  ASSERT_TRUE(module_c_motor2_value.has_value());
  EXPECT_NEAR(module_c_motor1_value.value(), 2.2352400, 1e-4);
  EXPECT_NEAR(module_c_motor2_value.value(), 2.6564055, 1e-4);
}

TEST_F(SwerveDriveControllerTest, UpdateNonDifferentialModuleCommand)
{
  controller_->init(
    "test_swerve_drive_controller", "", 100, "",
    rclcpp::NodeOptions().parameter_overrides(
      {
        rclcpp::Parameter("module_type", "non_differential"),
        rclcpp::Parameter("modules", std::vector<std::string>{"test_module"}),
        rclcpp::Parameter("module.test_module.motor1_joint", "steer_joint"),
        rclcpp::Parameter("module.test_module.motor2_joint", "drive_joint"),
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

  auto steer_cmd = make_command_interface("steer_joint", hardware_interface::HW_IF_VELOCITY);
  auto drive_cmd = make_command_interface("drive_joint", hardware_interface::HW_IF_VELOCITY);
  auto steer_vel_state = make_state_interface("steer_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto steer_pos_state = make_state_interface("steer_joint", hardware_interface::HW_IF_POSITION, 0.0);
  auto drive_vel_state = make_state_interface("drive_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto drive_pos_state = make_state_interface("drive_joint", hardware_interface::HW_IF_POSITION, 0.0);

  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
  command_interfaces.emplace_back(steer_cmd);
  command_interfaces.emplace_back(drive_cmd);

  std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
  state_interfaces.emplace_back(steer_vel_state);
  state_interfaces.emplace_back(steer_pos_state);
  state_interfaces.emplace_back(drive_vel_state);
  state_interfaces.emplace_back(drive_pos_state);

  controller_->assign_interfaces(std::move(command_interfaces), std::move(state_interfaces));
  auto activate_result = controller_->on_activate(rclcpp_lifecycle::State());
  ASSERT_EQ(activate_result, controller_interface::CallbackReturn::SUCCESS);

  auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_msg->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  twist_msg->twist.linear.x = 0.1;
  twist_msg->twist.linear.y = 0.05;   // diagonal, so steer actually has to move
  twist_msg->twist.angular.z = 0.0;

  controller_->received_velocity_msg_ptr_.writeFromNonRT(twist_msg);

  auto update_result = controller_->update(rclcpp::Clock(RCL_ROS_TIME).now(), rclcpp::Duration(0, 10000000));
  ASSERT_EQ(update_result, controller_interface::return_type::OK);

  auto steer_cmd_value = steer_cmd.get_optional();
  auto drive_cmd_value = drive_cmd.get_optional();
  ASSERT_TRUE(steer_cmd_value.has_value());
  ASSERT_TRUE(drive_cmd_value.has_value());

  // non_differential: motor1 (steer) gets the raw PID output directly —
  // no mixer involved, unlike the differential tests above.
  EXPECT_NEAR(steer_cmd_value.value(), 0.4636476, 1e-4);
  // motor2 (drive) gets the raw wheel_speed directly.
  EXPECT_NEAR(drive_cmd_value.value(), 2.2360680, 1e-4);
}

TEST_F(SwerveDriveControllerTest, UpdatePublishesOdometry)
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

  auto motor1_vel_cmd = make_command_interface("motor1_joint", hardware_interface::HW_IF_VELOCITY);
  auto motor2_vel_cmd = make_command_interface("motor2_joint", hardware_interface::HW_IF_VELOCITY);
  auto motor1_vel_state = make_state_interface("motor1_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto motor1_pos_state = make_state_interface("motor1_joint", hardware_interface::HW_IF_POSITION, 0.0);
  auto motor2_vel_state = make_state_interface("motor2_joint", hardware_interface::HW_IF_VELOCITY, 0.0);
  auto motor2_pos_state = make_state_interface("motor2_joint", hardware_interface::HW_IF_POSITION, 0.0);

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
  twist_msg->twist.linear.x = 0.1;
  twist_msg->twist.linear.y = 0.0;
  twist_msg->twist.angular.z = 0.0;

  controller_->received_velocity_msg_ptr_.writeFromNonRT(twist_msg);

  auto update_time = rclcpp::Clock(RCL_ROS_TIME).now();
  auto update_result = controller_->update(update_time, rclcpp::Duration(0, 10000000));
  ASSERT_EQ(update_result, controller_interface::return_type::OK);

  ASSERT_TRUE(controller_->realtime_odometry_publisher_ != nullptr);
  const auto & odom_msg = controller_->realtime_odometry_publisher_->msg_;

  // Velocity feedback interfaces were never written to (still 0.0), so
  // regardless of the commanded motion, the ESTIMATED twist from feedback
  // should be zero — this proves the publish path runs and reads from
  // feedback, not from the commands we just wrote.
  EXPECT_NEAR(odom_msg.twist.twist.linear.x, 0.0, 1e-6);
  EXPECT_NEAR(odom_msg.twist.twist.linear.y, 0.0, 1e-6);
  EXPECT_NEAR(odom_msg.twist.twist.angular.z, 0.0, 1e-6);
  EXPECT_EQ(odom_msg.header.stamp, update_time);
}

TEST_F(SwerveDriveControllerTest, UpdatePublishesNonZeroOdometryAndTf)
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

  auto motor1_vel_cmd = make_command_interface("motor1_joint", hardware_interface::HW_IF_VELOCITY);
  auto motor2_vel_cmd = make_command_interface("motor2_joint", hardware_interface::HW_IF_VELOCITY);
  // Feedback set to 2.0 rad/s on BOTH motors — via unmix(), equal legs
  // recover a pure wheel_speed (4.0 rad/s) with zero steer_rate, giving a
  // clean, predictable case: speed = 4.0 * wheel_radius = 0.2 m/s, angle = 0.
  auto motor1_vel_state = make_state_interface("motor1_joint", hardware_interface::HW_IF_VELOCITY, 2.0);
  auto motor1_pos_state = make_state_interface("motor1_joint", hardware_interface::HW_IF_POSITION, 0.0);
  auto motor2_vel_state = make_state_interface("motor2_joint", hardware_interface::HW_IF_VELOCITY, 2.0);
  auto motor2_pos_state = make_state_interface("motor2_joint", hardware_interface::HW_IF_POSITION, 0.0);

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

  // cmd_vel content doesn't matter for THIS test — odometry comes from
  // feedback, not from what we command — but update() still needs a valid
  // (non-stale) message to proceed past piece 1.
  auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_msg->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  twist_msg->twist.linear.x = 0.0;
  twist_msg->twist.linear.y = 0.0;
  twist_msg->twist.angular.z = 0.0;
  controller_->received_velocity_msg_ptr_.writeFromNonRT(twist_msg);

  auto update_time = rclcpp::Clock(RCL_ROS_TIME).now();
  auto update_result = controller_->update(update_time, rclcpp::Duration(0, 10000000));  // 0.01s
  ASSERT_EQ(update_result, controller_interface::return_type::OK);

  ASSERT_TRUE(controller_->realtime_odometry_publisher_ != nullptr);
  const auto & odom_msg = controller_->realtime_odometry_publisher_->msg_;

  EXPECT_NEAR(odom_msg.twist.twist.linear.x, 0.2, 1e-6);
  EXPECT_NEAR(odom_msg.twist.twist.linear.y, 0.0, 1e-6);
  EXPECT_NEAR(odom_msg.twist.twist.angular.z, 0.0, 1e-6);

  EXPECT_NEAR(odom_msg.pose.pose.position.x, 0.002, 1e-6);
  EXPECT_NEAR(odom_msg.pose.pose.position.y, 0.0, 1e-6);
  EXPECT_NEAR(odom_msg.pose.pose.orientation.w, 1.0, 1e-6);
  EXPECT_NEAR(odom_msg.pose.pose.orientation.x, 0.0, 1e-6);
  EXPECT_NEAR(odom_msg.pose.pose.orientation.y, 0.0, 1e-6);
  EXPECT_NEAR(odom_msg.pose.pose.orientation.z, 0.0, 1e-6);

  ASSERT_TRUE(controller_->realtime_odometry_transform_publisher_ != nullptr);
  ASSERT_FALSE(controller_->realtime_odometry_transform_publisher_->msg_.transforms.empty());
  const auto & tf_msg = controller_->realtime_odometry_transform_publisher_->msg_.transforms.front();

  EXPECT_NEAR(tf_msg.transform.translation.x, 0.002, 1e-6);
  EXPECT_NEAR(tf_msg.transform.translation.y, 0.0, 1e-6);
  EXPECT_NEAR(tf_msg.transform.rotation.w, 1.0, 1e-6);
}

