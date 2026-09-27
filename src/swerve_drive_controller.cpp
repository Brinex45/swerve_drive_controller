#include "swerve_drive_controller.hpp"

namespace swerve_drive_controller{

    // ── on_init ───────────────────────────────────────────────────────────────
    // First lifecycle callback called by controller_manager, before any
    // hardware interfaces exist. Its only job: stand up the generated
    // ParamListener and pull the first snapshot of params_. Wrapped in
    // try/catch because generate_parameter_library throws (e.g. on a missing
    // required parameter) rather than returning an error code — an uncaught
    // exception here would crash controller_manager, not just fail to load
    // this one controller, so we must convert it into a clean ERROR return.
    controller_interface::CallbackReturn SwerveDriveController::on_init()
    {
        try
        {
            param_listener_ = std::make_shared<swerve_drive_controller::ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Exception thrown during on_init() while initializing parameters: %s",
                e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration SwerveDriveController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const auto & module_name : params_.modules)
        {
            const auto & module_params = params_.module.modules_map.at(module_name);
            config.names.push_back(module_params.motor1_joint + "/" + hardware_interface::HW_IF_VELOCITY);
            config.names.push_back(module_params.motor2_joint + "/" + hardware_interface::HW_IF_VELOCITY);
        }

        return config;
    }

    controller_interface::InterfaceConfiguration SwerveDriveController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const auto & module_name : params_.modules)
        {
            const auto & module_params = params_.module.modules_map.at(module_name);

            config.names.push_back(module_params.motor1_joint + "/" + hardware_interface::HW_IF_VELOCITY);
            config.names.push_back(module_params.motor2_joint + "/" + hardware_interface::HW_IF_VELOCITY);

            // Position feedback is unconditionally required — both the
            // differential azimuth estimator and non-differential direct
            // steer readback need it. No supported position-feedback-free
            // mode (see chat: position_feedback param removed from YAML).
            config.names.push_back(module_params.motor1_joint + "/" + hardware_interface::HW_IF_POSITION);
            config.names.push_back(module_params.motor2_joint + "/" + hardware_interface::HW_IF_POSITION);
        }

        return config;
    }

    // ── on_configure ─────────────────────────────────────────────────────────
    controller_interface::CallbackReturn SwerveDriveController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        params_ = param_listener_->get_params();

        module_type_ = params_.module_type;

        modules_.clear();
        modules_.reserve(params_.modules.size());

        for (const auto & module_name : params_.modules)
        {
            const auto & module_params = params_.module.modules_map.at(module_name);

            Module module;
            module.motor1_joint = module_params.motor1_joint;
            module.motor2_joint = module_params.motor2_joint;
            module.module_position_x = module_params.module_position_x;
            module.module_position_y = module_params.module_position_y;
            module.gear_ratio = module_params.gear_ratio;
            module.wheel_radius = module_params.wheel_radius;

            module.pid_gains.p = params_.steer_pid.p;
            module.pid_gains.i = params_.steer_pid.i;
            module.pid_gains.d = params_.steer_pid.d;
            module.pid_gains.i_clamp_min = params_.steer_pid.i_clamp_min;
            module.pid_gains.i_clamp_max = params_.steer_pid.i_clamp_max;

            control_toolbox::AntiWindupStrategy antiwindup_strat;
            antiwindup_strat.type = control_toolbox::AntiWindupStrategy::LEGACY;
            antiwindup_strat.i_min = module.pid_gains.i_clamp_min;
            antiwindup_strat.i_max = module.pid_gains.i_clamp_max;

            module.pid = control_toolbox::Pid(
                module.pid_gains.p,
                module.pid_gains.i,
                module.pid_gains.d,
                std::numeric_limits<double>::infinity(),   // u_max — output clamp, unused; matches prior behavior of none
                -std::numeric_limits<double>::infinity(),  // u_min
                antiwindup_strat);

            modules_.push_back(std::move(module));
        }

        received_velocity_msg_ptr_.writeFromNonRT(
            std::make_shared<geometry_msgs::msg::TwistStamped>());

        cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
            "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
            [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) -> void
            {
                if (!subscriber_is_active_)
                {
                    RCLCPP_WARN(
                        get_node()->get_logger(),
                        "Received cmd_vel message but controller is not active — ignoring.");
                    return;
                }
                if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
                {
                    RCLCPP_WARN_ONCE(
                        get_node()->get_logger(),
                        "Received TwistStamped with zero timestamp, setting it to current "
                        "time. This message will only be shown once.");
                    msg->header.stamp = get_node()->get_clock()->now();
                }
                received_velocity_msg_ptr_.writeFromNonRT(msg);
            });

        odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
            "~/odom", rclcpp::SystemDefaultsQoS());
        realtime_odometry_publisher_ =
            std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
                odometry_publisher_);

        realtime_odometry_publisher_->lock();
        realtime_odometry_publisher_->msg_.header.frame_id = odom_frame_id_;
        realtime_odometry_publisher_->msg_.child_frame_id = base_frame_id_;
        realtime_odometry_publisher_->msg_.pose.pose.position.z = 0;

        for (size_t i = 0; i < 6; ++i)
        {
            realtime_odometry_publisher_->msg_.pose.covariance[i * 6 + i] =
                pose_covariance_diagonal_[i];
            realtime_odometry_publisher_->msg_.twist.covariance[i * 6 + i] =
                twist_covariance_diagonal_[i];
        }
        realtime_odometry_publisher_->unlock();

        if (enable_odom_tf_)
        {
            odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
                "/tf", rclcpp::SystemDefaultsQoS());
            realtime_odometry_transform_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
                    odometry_transform_publisher_);

            realtime_odometry_transform_publisher_->lock();
            realtime_odometry_transform_publisher_->msg_.transforms.resize(1);
            auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
            transform.header.frame_id = odom_frame_id_;
            transform.child_frame_id = base_frame_id_;
            realtime_odometry_transform_publisher_->unlock();
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    // ── on_cleanup ────────────────────────────────────────────────────────────
    controller_interface::CallbackReturn SwerveDriveController::on_cleanup(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        modules_.clear();

        cmd_vel_subscriber_.reset();
        odometry_publisher_.reset();
        odometry_transform_publisher_.reset();
        realtime_odometry_publisher_.reset();
        realtime_odometry_transform_publisher_.reset();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    // ── on_activate ───────────────────────────────────────────────────────────
    controller_interface::CallbackReturn SwerveDriveController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        for (auto & module_ : modules_)
        {
            // --- command interfaces (always required) ---
            auto motor1_cmd_it = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&](const auto & iface)
                {
                    return iface.get_prefix_name() == module_.motor1_joint &&
                           iface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                });
            if (motor1_cmd_it == command_interfaces_.end())
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "Unable to find command interface for %s", module_.motor1_joint.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }
            module_.motor1_cmd = &(*motor1_cmd_it);

            auto motor2_cmd_it = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&](const auto & iface)
                {
                    return iface.get_prefix_name() == module_.motor2_joint &&
                           iface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                });
            if (motor2_cmd_it == command_interfaces_.end())
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "Unable to find command interface for %s", module_.motor2_joint.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }
            module_.motor2_cmd = &(*motor2_cmd_it);

            // --- velocity state interfaces (always required) ---
            auto motor1_vel_it = std::find_if(
                state_interfaces_.begin(), state_interfaces_.end(),
                [&](const auto & iface)
                {
                    return iface.get_prefix_name() == module_.motor1_joint &&
                           iface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                });
            if (motor1_vel_it == state_interfaces_.end())
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "Unable to find velocity state interface for %s", module_.motor1_joint.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }
            module_.motor1_velocity_state = &(*motor1_vel_it);

            auto motor2_vel_it = std::find_if(
                state_interfaces_.begin(), state_interfaces_.end(),
                [&](const auto & iface)
                {
                    return iface.get_prefix_name() == module_.motor2_joint &&
                           iface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                });
            if (motor2_vel_it == state_interfaces_.end())
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "Unable to find velocity state interface for %s", module_.motor2_joint.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }
            module_.motor2_velocity_state = &(*motor2_vel_it);

            // --- position state interfaces (unconditionally required) ---
            auto motor1_pos_it = std::find_if(
                state_interfaces_.begin(), state_interfaces_.end(),
                [&](const auto & iface)
                {
                    return iface.get_prefix_name() == module_.motor1_joint &&
                           iface.get_interface_name() == hardware_interface::HW_IF_POSITION;
                });
            if (motor1_pos_it == state_interfaces_.end())
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "Unable to find position state interface for %s", module_.motor1_joint.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }
            module_.motor1_position_state = &(*motor1_pos_it);

            auto motor2_pos_it = std::find_if(
                state_interfaces_.begin(), state_interfaces_.end(),
                [&](const auto & iface)
                {
                    return iface.get_prefix_name() == module_.motor2_joint &&
                           iface.get_interface_name() == hardware_interface::HW_IF_POSITION;
                });
            if (motor2_pos_it == state_interfaces_.end())
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "Unable to find position state interface for %s", module_.motor2_joint.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }
            module_.motor2_position_state = &(*motor2_pos_it);
        }

        subscriber_is_active_ = true;

        return controller_interface::CallbackReturn::SUCCESS;
    }

    // ── on_deactivate ─────────────────────────────────────────────────────────
    controller_interface::CallbackReturn SwerveDriveController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        for (auto & module_ : modules_)
        {
            module_.motor1_cmd = nullptr;
            module_.motor2_cmd = nullptr;

            module_.motor1_velocity_state = nullptr;
            module_.motor2_velocity_state = nullptr;

            module_.motor1_position_state = nullptr;
            module_.motor2_position_state = nullptr;
        }

        subscriber_is_active_ = false;

        return controller_interface::CallbackReturn::SUCCESS;
    }

    // ── update ──────────────────────────────────────────────────────────────────
    controller_interface::return_type SwerveDriveController::update(
        const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // --- piece 1: read cmd_vel, handle timeout ---
        auto twist_msg_ = received_velocity_msg_ptr_.readFromRT();

        if (!twist_msg_ || !(*twist_msg_))
        {
            return controller_interface::return_type::OK;
        }

        const auto age_of_last_command_ = time - (*twist_msg_)->header.stamp;

        Twist target_twist_;
        if (cmd_vel_timeout_ == 0.0 || age_of_last_command_ <= rclcpp::Duration::from_seconds(cmd_vel_timeout_))
        {
            target_twist_.vx = (*twist_msg_)->twist.linear.x;
            target_twist_.vy = (*twist_msg_)->twist.linear.y;
            target_twist_.omega = (*twist_msg_)->twist.angular.z;
        }
        else
        {
            // stale command — target_twist_ stays zero (Twist now has
            // default member initializers), commands a safe stop
        }

        (void)period;

        // --- piece 2 & 3 & 4a: per-module IK, PID + dispatch, feedback readback ---
        std::vector<ModuleState> module_states_;
        module_states_.reserve(modules_.size());

        for (auto & module_ : modules_)
        {
            // get_optional() (not get_value()) is required on Jazzy — state
            // interfaces can be momentarily locked by a concurrent thread,
            // and update() must never block waiting on that lock. An empty
            // optional here means "value not available this cycle" — we
            // skip actuating this module for this cycle rather than act on
            // a stale/wrong angle.
            auto motor1_pos_opt = module_.motor1_position_state->get_optional();
            auto motor2_pos_opt = module_.motor2_position_state->get_optional();

            if (!motor1_pos_opt.has_value() || !motor2_pos_opt.has_value())
            {
                RCLCPP_WARN_THROTTLE(
                    get_node()->get_logger(), *get_node()->get_clock(), 1000,
                    "Position feedback unavailable for module with motor1_joint '%s' this cycle — skipping.",
                    module_.motor1_joint.c_str());
                continue;
            }

            double current_angle = 0.0;
            if (module_type_ == "differential")
            {
                current_angle = estimateAzimuthDerived(
                    motor1_pos_opt.value(), motor2_pos_opt.value(), module_.gear_ratio);
            }
            else  // non_differential — motor1_joint is the steer joint by convention
            {
                current_angle = motor1_pos_opt.value();
            }

            ModulePos module_pos{module_.module_position_x, module_.module_position_y};
            ModuleCommand target = computeModuleCommand(target_twist_, module_pos);
            ModuleCommand optimized = optimizeSteerDirection(target, current_angle);

            // --- piece 3: steer PID, dispatch per module_type, write commands ---
            double angle_error = SwerveIK::normalizeAngle(optimized.angle - current_angle);
            double steer_rate = module_.pid.compute_command(angle_error, period);

            // optimized.speed is linear wheel speed (m/s) from the IK — the
            // motors are velocity interfaces in rad/s, so convert via
            // wheel_radius before mixing/writing.
            double wheel_speed = optimized.speed / module_.wheel_radius;

            bool motor1_ok = false;
            bool motor2_ok = false;

            if (module_type_ == "differential")
            {
                MixerInput mixer_input{wheel_speed, steer_rate};
                MotorCommand motor_cmd = mix(mixer_input, module_.gear_ratio);
                motor1_ok = module_.motor1_cmd->set_value(motor_cmd.motor1_velocity);
                motor2_ok = module_.motor2_cmd->set_value(motor_cmd.motor2_velocity);
            }
            else  // non_differential — motor1 is steer (PID output directly), motor2 is drive
            {
                motor1_ok = module_.motor1_cmd->set_value(steer_rate);
                motor2_ok = module_.motor2_cmd->set_value(wheel_speed);
            }

            if (!motor1_ok || !motor2_ok)
            {
                RCLCPP_WARN_THROTTLE(
                    get_node()->get_logger(), *get_node()->get_clock(), 1000,
                    "Failed to set command interface value for module with motor1_joint '%s' this cycle.",
                    module_.motor1_joint.c_str());
            }

            // --- piece 4a: read velocity feedback for odometry ---
            auto motor1_vel_fb_opt = module_.motor1_velocity_state->get_optional();
            auto motor2_vel_fb_opt = module_.motor2_velocity_state->get_optional();

            if (motor1_vel_fb_opt.has_value() && motor2_vel_fb_opt.has_value())
            {
                double measured_wheel_speed_angular = 0.0;

                if (module_type_ == "differential")
                {
                    // No open-loop substitute exists for a differential
                    // module — drive and steer are coupled through the
                    // mixer, so open_loop_odometry_ has no effect here.
                    MixerInput measured = unmix(
                        MotorCommand{motor1_vel_fb_opt.value(), motor2_vel_fb_opt.value()},
                        module_.gear_ratio);
                    measured_wheel_speed_angular = measured.wheel_speed;
                }
                else  // non_differential
                {
                    // open_loop_odometry_ only substitutes the drive side
                    // here — this cycle's commanded wheel_speed instead of
                    // measured feedback. Steer angle (below) is always
                    // measured, never substituted, regardless of this flag.
                    measured_wheel_speed_angular =
                        open_loop_odometry_ ? wheel_speed : motor2_vel_fb_opt.value();
                }

                ModuleCommand measured_command;
                measured_command.speed = measured_wheel_speed_angular * module_.wheel_radius;
                measured_command.angle = current_angle;  // always feedback-derived

                module_states_.push_back(
                    ModuleState{measured_command, ModulePos{module_.module_position_x, module_.module_position_y}});
            }
        }

        // --- piece 4b: odometry — estimate twist, integrate pose, publish ---
        Twist estimated_twist = estimateTwist(module_states_);
        integratePose(pose_, estimated_twist, period.seconds());

        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, pose_.theta);

        if (realtime_odometry_publisher_ && realtime_odometry_publisher_->trylock())
        {
            realtime_odometry_publisher_->msg_.header.stamp = time;
            realtime_odometry_publisher_->msg_.pose.pose.position.x = pose_.x;
            realtime_odometry_publisher_->msg_.pose.pose.position.y = pose_.y;
            realtime_odometry_publisher_->msg_.pose.pose.orientation = tf2::toMsg(orientation);
            realtime_odometry_publisher_->msg_.twist.twist.linear.x = estimated_twist.vx;
            realtime_odometry_publisher_->msg_.twist.twist.linear.y = estimated_twist.vy;
            realtime_odometry_publisher_->msg_.twist.twist.angular.z = estimated_twist.omega;
            realtime_odometry_publisher_->unlockAndPublish();
        }

        if (enable_odom_tf_ && realtime_odometry_transform_publisher_ &&
            realtime_odometry_transform_publisher_->trylock())
        {
            auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
            transform.header.stamp = time;
            transform.transform.translation.x = pose_.x;
            transform.transform.translation.y = pose_.y;
            transform.transform.rotation = tf2::toMsg(orientation);
            realtime_odometry_transform_publisher_->unlockAndPublish();
        }

        return controller_interface::return_type::OK;
    }


} // namespace swerve_drive_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(swerve_drive_controller::SwerveDriveController, controller_interface::ControllerInterface)