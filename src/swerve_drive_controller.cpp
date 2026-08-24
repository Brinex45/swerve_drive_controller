#include "swerve_drive_controller.hpp"

namespace swerve_drive_controller{

    controller_interface::CallbackReturn SwerveDriveController::on_init(
        const controller_interface::HardwareInfo &info)
    {
        


        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration SwerveDriveController::command_interface_configuration()
    {
        


    }

    controller_interface::InterfaceConfiguration SwerveDriveController::state_interface_configuration()
    {
        


    }

    // ── on_configure ─────────────────────────────────────────────────────────
    controller_interface::CallbackReturn SwerveDriveController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        


        return controller_interface::CallbackReturn::SUCCESS;
    }

    // ── on_cleanup ────────────────────────────────────────────────────────────
    controller_interface::CallbackReturn SwerveDriveController::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        


        return controller_interface::CallbackReturn::SUCCESS;
    }

    // ── on_activate ───────────────────────────────────────────────────────────
    controller_interface::CallbackReturn SwerveDriveController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
       


        return controller_interface::CallbackReturn::SUCCESS;
    }

    // ── on_deactivate ─────────────────────────────────────────────────────────
    controller_interface::CallbackReturn SwerveDriveController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        


        return controller_interface::CallbackReturn::SUCCESS;
    }

    // ── read ──────────────────────────────────────────────────────────────────
    controller_interface::return_type SwerveDriveController::read(
        const rclcpp::Time &, const rclcpp::Duration &period)
    {
        


        return controller_interface::return_type::OK;
    }

    // ── write ─────────────────────────────────────────────────────────────────
    controller_interface::return_type SwerveDriveController::write(
        const rclcpp::Time &, const rclcpp::Duration &period)
    {
        


        return controller_interface::return_type::OK;
    }

} // namespace dcmotor

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dcmotor::SwerveDriveController, controller_interface::SystemInterface)