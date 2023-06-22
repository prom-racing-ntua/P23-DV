#include "lifecycle/lifecycle_velocity_estimation_handler.hpp"
#include "lifecycle_msgs/msg/state.hpp"


namespace ns_vel_est
{
    ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_configure(const rclcpp_lifecycle::State &state)
    {        
        get_parameter("frequency", node_frequency_);

        if (!node_frequency_)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Node frequency not specified: Configuration of Velocity Estimation Failed: " << node_frequency_);
            return ns_vel_est::CallbackReturn::FAILURE;
        }

        estimator_.setDeltaTime(1.0 / node_frequency_);
        estimator_.init();

        vn_200_rotation_matrix_ = getRotationMatrix(get_parameter("vectornav-vn-200.roll").as_double(),
            get_parameter("vectornav-vn-200.pitch").as_double());

        vn_300_rotation_matrix_ = getRotationMatrix(get_parameter("vectornav-vn-300.roll").as_double(),
            get_parameter("vectornav-vn-300.pitch").as_double());

        setSubscribers();

        pub_ = create_publisher<custom_msgs::msg::VelEstimation>("velocity_estimation", 10);
        RCLCPP_WARN(get_logger(), "\n-- Velocity Estimation Configured!");

        return ns_vel_est::CallbackReturn::SUCCESS;
    }

    ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_activate(const rclcpp_lifecycle::State &state) 
    {
        measurement_vector_.setZero();
        updated_sensors_.fill(false);
        pub_->on_activate();
        RCLCPP_WARN(get_logger(), "\n-- Velocity Estimation Activated!");
        return ns_vel_est::CallbackReturn::SUCCESS;
    }

    ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_deactivate(const rclcpp_lifecycle::State &state) 
    {   
        pub_->on_deactivate();
        RCLCPP_WARN(get_logger(), "\n-- Velocity Estimation Deactivated!");
        return ns_vel_est::CallbackReturn::SUCCESS;
    }

    ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_cleanup(const rclcpp_lifecycle::State &state) 
    {
        /* Subscriber cleanup */
        vn_velocity_sub_.reset();
        vn_imu_sub_.reset();
        wheel_encoder_sub_.reset();
        steering_sub_.reset();
        master_sub_.reset();

        /* VelEst class and Publisher cleanup */
        estimator_.reset();
        pub_.reset();
        RCLCPP_WARN(get_logger(), "\n-- Velocity Estimation Un-Configured!");
        return ns_vel_est::CallbackReturn::SUCCESS;
    }

    ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_shutdown(const rclcpp_lifecycle::State &state) 
    {  
        using NodeState = lifecycle_msgs::msg::State;

        uint8_t currentState = state.id();
        
        if (currentState == NodeState::PRIMARY_STATE_UNCONFIGURED) {
            RCLCPP_INFO(get_logger(), "\n-- Velocity Estimation Shutdown!");
            return ns_vel_est::CallbackReturn::SUCCESS;
        }

        /* Subscriber cleanup */
        vn_velocity_sub_.reset();
        vn_imu_sub_.reset();
        wheel_encoder_sub_.reset();
        steering_sub_.reset();
        master_sub_.reset();

        /* VelEst class and Pulibhser cleanup */
        estimator_.reset();
        pub_.reset();
        RCLCPP_INFO(get_logger(), "\n-- Velocity Estimation Shutdown!");
        return ns_vel_est::CallbackReturn::SUCCESS;
    }

    ns_vel_est::CallbackReturn
        LifecycleVelocityEstimationHandler::on_error(const rclcpp_lifecycle::State &state)
    {
        return ns_vel_est::CallbackReturn::SUCCESS;
    }
}