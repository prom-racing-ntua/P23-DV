#include "lifecycle_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"


namespace pid_pp{
    pid_pp::CallbackReturn
        LifecyclePID_PP_Node::on_configure(const rclcpp_lifecycle::State &state)
    {
        model = Model(
            get_parameter("mass").as_double(),
            get_parameter("gravitational_acceleration").as_double(),
            get_parameter("wheelbase").as_double(),
            get_parameter("weight_distribution").as_double(),
            get_parameter("h_cog").as_double(),
            get_parameter("air_density").as_double(),
            get_parameter("Cd*A").as_double(),
            get_parameter("Cl*A").as_double(),
            get_parameter("gear_ratio").as_double(),
            get_parameter("wheel_radius").as_double(),
            get_parameter("efficiency").as_double(),
            get_parameter("Fz_0").as_double(),
            get_parameter("c_tire").as_double(),
            get_parameter("max_positive_torque").as_double(),
            get_parameter("max_negative_torque").as_double(),
            get_parameter("minimum_weight_distribution_rear").as_double());
        pid_controller.init(
            get_parameter("kp").as_int(),
            get_parameter("ki").as_int(),
            get_parameter("kd").as_int(),
            get_parameter("dt").as_double(),
            get_parameter("PID_max_output").as_int(),
            get_parameter("Integral_max_output").as_int());
        pp_controller.init(
            get_parameter("ld_min").as_double(),
            get_parameter("ld_max").as_double(),
            get_parameter("v_min").as_double(),
            get_parameter("v_max").as_double(),
            model.wb,
            get_parameter("emergency_factor").as_double());

        emergency_threshold = get_parameter("emergency_threshold").as_double();
        safety_factor = get_parameter("safety_factor").as_double();
        max_speed = get_parameter("max_speed").as_double();
        safe_speed_to_break = get_parameter("safe_speed_to_break").as_double();
        spline_res_per_meter = get_parameter("spline_resolution_per_meter").as_int();
        laps_to_do = get_parameter("total_laps").as_int();

        is_end = false;

        mutexCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = mutexCallbackGroup;

        sub_waypoints = this->create_subscription<custom_msgs::msg::WaypointsMsg>("waypoints", 10, std::bind(&LifecyclePID_PP_Node::waypoints_callback, this, _1), sub_opt);
        sub_pose = this->create_subscription<custom_msgs::msg::PoseMsg>("pose", 10, std::bind(&LifecyclePID_PP_Node::pose_callback, this, _1), sub_opt);

        pub_actuators = this->create_publisher<custom_msgs::msg::TxControlCommand>("control_commands", 10);
        pub_target = this->create_publisher<custom_msgs::msg::Point2Struct>("pp_target_point", 10);

        RCLCPP_INFO(get_logger(), "Pure Pursuite Node Configured!");

        return pid_pp::CallbackReturn::SUCCESS;
    }

    pid_pp::CallbackReturn
        LifecyclePID_PP_Node::on_activate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Activating Pure Pursuit Node");

        pub_actuators->on_activate();
        pub_target->on_activate();

        RCLCPP_INFO(get_logger(), "Pure Pursuit Node Activated!");

        return pid_pp::CallbackReturn::SUCCESS;
    }

    pid_pp::CallbackReturn
        LifecyclePID_PP_Node::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Deactivating Pure Pursuit Node");

        pub_actuators->on_deactivate();
        pub_target->on_deactivate();

        RCLCPP_INFO(get_logger(), "Pure Pursuit Node deactivated!");

        return pid_pp::CallbackReturn::SUCCESS;
    }

    pid_pp::CallbackReturn
        LifecyclePID_PP_Node::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Cleaning Pure Pursuit Node");

        pub_actuators->on_deactivate();
        pub_target->on_deactivate();

        pub_actuators.reset();
        pub_target.reset();

        /* Edw prepei na kanoyme kai clean up to model */

        RCLCPP_INFO(get_logger(), "Pure Pursuit Node unconfigured!");

        return pid_pp::CallbackReturn::SUCCESS;
    }

    pid_pp::CallbackReturn
        LifecyclePID_PP_Node::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Shutting down Pure Pursuit Node");

        using NodeState = lifecycle_msgs::msg::State;

        uint8_t currentState = state.id();
        
        if (currentState == NodeState::PRIMARY_STATE_UNCONFIGURED) {
            RCLCPP_WARN(get_logger(), "Pure Pursuit Node Shutdown!");
            return pid_pp::CallbackReturn::SUCCESS;
        }

        pub_actuators->on_deactivate();
        pub_target->on_deactivate();

        pub_actuators.reset();
        pub_target.reset();
        /* Edw prepei na kanoyme kai clean up to model */

        RCLCPP_INFO(get_logger(), "Pure Pursuit Node shut down!");

        return pid_pp::CallbackReturn::SUCCESS;
    }

    pid_pp::CallbackReturn
        LifecyclePID_PP_Node::on_error(const rclcpp_lifecycle::State &state)
    {
        return pid_pp::CallbackReturn::SUCCESS;   
    }
}