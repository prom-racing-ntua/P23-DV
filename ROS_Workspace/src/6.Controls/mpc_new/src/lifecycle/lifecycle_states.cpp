#include "l_mpc_new.h"
#include "lifecycle_msgs/msg/state.hpp"

namespace mpc_new
{
    mpc_new::CallbackReturn
    LifecycleHandler::on_configure(const rclcpp_lifecycle::State &state)
    {
        double F_init = get_parameter("F_init").as_double();
        int horizonLength = get_parameter("horizon").as_int();
        double ds = get_parameter("ds").as_double();
        double dt = get_parameter("dt").as_double();
        double vel_max = get_parameter("max_velocity").as_double();
        double maxF = get_parameter("F_max").as_double();
        double minF = get_parameter("F_min").as_double();
        std::string mission = get_parameter("mission").as_string();
        int laps = get_parameter("laps").as_int();
        mpc_solver = new_MpcSolver(F_init, horizonLength, ds, dt, vel_max, maxF, minF, mission, laps);

        //Next build the publishers and the subscibers
        auto sub_opt = rclcpp::SubscriptionOptions();

        //publishers
        dc_publisher = this->create_publisher<custom_msgs::msg::TxControlCommand>("control_commands", 10);
        //subscribers
        pose_subscriber = this->create_subscription<custom_msgs::msg::PoseMsg>("pose", 10, std::bind(&LifecycleHandler::pose_callback, this, _1), sub_opt);
        path_subscriber = this->create_subscription<custom_msgs::msg::WaypointsMsg>("waypoints", 10, std::bind(&LifecycleHandler::waypoints_callback, this, _1), sub_opt);

        spline = nullptr;

        total_laps_cli = this->create_client<custom_msgs::srv::SetTotalLaps>("/p23_status/set_total_laps");

        // Callback Function
        auto response_received_callback = [this](rclcpp::Client<custom_msgs::srv::SetTotalLaps>::SharedFuture future)
        {
            auto result = future.get();
            if (result->success)
                RCLCPP_INFO(get_logger(), "Total mission laps set successfully");
        };

        if (!total_laps_cli->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(get_logger(), "P23 Status service is not available");
        }
        else
        {
            auto request = std::make_shared<custom_msgs::srv::SetTotalLaps::Request>();
            request->total_laps = mpc_solver.total_laps;
            auto future_result = total_laps_cli->async_send_request(request, response_received_callback);
        }

        waypoints_timestamp_log.init("mpc_waypoints");
        RCLCPP_INFO_STREAM(get_logger(),waypoints_timestamp_log.check());
        pose_timestamp_log.init("mpc_pose");
        RCLCPP_INFO_STREAM(get_logger(),pose_timestamp_log.check());
        RCLCPP_INFO_STREAM(get_logger(), "Laps to do: "<< mpc_solver.total_laps);
        RCLCPP_WARN(get_logger(), "\n-- MPC Configured!");
        return mpc_new::CallbackReturn::SUCCESS;
    }

    mpc_new::CallbackReturn
    LifecycleHandler::on_activate(const rclcpp_lifecycle::State &state)
    {
        //mutexCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt = rclcpp::SubscriptionOptions();
        //sub_opt.callback_group = mutexCallbackGroup;

        path_subscriber = this->create_subscription<custom_msgs::msg::WaypointsMsg>("waypoints", 10, std::bind(&LifecycleHandler::waypoints_callback, this, _1), sub_opt);
        pose_subscriber = this->create_subscription<custom_msgs::msg::PoseMsg>("pose", 10, std::bind(&LifecycleHandler::pose_callback, this, _1), sub_opt);

        dc_publisher->on_activate();

        //Initialize on activate my counter
        mpc_solver.global_counter = 0;

        RCLCPP_WARN(get_logger(), "\n-- MPC Activated!");
        return mpc_new::CallbackReturn::SUCCESS;
    }

    mpc_new::CallbackReturn
    LifecycleHandler::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        dc_publisher->on_deactivate();

        RCLCPP_WARN(get_logger(), "\n-- MPC Deactivated!");
        return mpc_new::CallbackReturn::SUCCESS;
    }

    mpc_new::CallbackReturn
    LifecycleHandler::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        dc_publisher->on_deactivate();

        dc_publisher.reset();

        /* Edw prepei na kanoyme kai clean up to model */

        RCLCPP_WARN(get_logger(), "\n-- MPC Un-Configured!");
        return mpc_new::CallbackReturn::SUCCESS;
    }

    mpc_new::CallbackReturn
    LifecycleHandler::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        using NodeState = lifecycle_msgs::msg::State;
        uint8_t currentState = state.id();

        if (currentState == NodeState::PRIMARY_STATE_UNCONFIGURED)
        {
            RCLCPP_INFO(get_logger(), "\n-- MPC Shutdown!");
            return mpc_new::CallbackReturn::SUCCESS;
        }

        dc_publisher->on_deactivate();

        dc_publisher.reset();
        /* Edw prepei na kanoyme kai clean up to model */

        RCLCPP_INFO(get_logger(), "\n-- MPC Shutdown!");
        return mpc_new::CallbackReturn::SUCCESS;
    }

    mpc_new::CallbackReturn
    LifecycleHandler::on_error(const rclcpp_lifecycle::State &state)
    {
        return mpc_new::CallbackReturn::SUCCESS;
    }
}