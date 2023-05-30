#include "lifecycle/lifecycle_velocity_estimation.h"
#include "lifecycle/lifecycle_velocity_estimation_handler.hpp"

namespace ns_vel_est
{
    ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_configure(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Configuring Lifecycle Velocity Estimation node");
        
        get_parameter("frequency", node_frequency_);

        if (!node_frequency_)
        {
            RCLCPP_ERROR(get_logger(), "Node frequency not specified: Configuration of Velocity Estimation Failed");
            return ns_vel_est::CallbackReturn::FAILURE;
        }

        estimator_.setDeltaTime(1.0 / node_frequency_);
        estimator_.init();

        vn_200_rotation_matrix_ = getRotationMatrix(get_parameter("vectornav-vn-200.roll").as_double(),
            get_parameter("vectornav-vn-200.pitch").as_double());

        vn_300_rotation_matrix_ = getRotationMatrix(get_parameter("vectornav-vn-300.roll").as_double(),
            get_parameter("vectornav-vn-300.pitch").as_double());

        // Initialize the measurement and update vector with zeros
        measurement_vector_.setZero();
        updated_sensors_.fill(false);
        setSubscribers();

        pub_ = create_publisher<custom_msgs::msg::VelEstimation>("velocity_estimation", 10);
        RCLCPP_INFO(get_logger(), "Lifecycle Velocity Estimation Configured!");

        return ns_vel_est::CallbackReturn::SUCCESS;
    }

    ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_activate(const rclcpp_lifecycle::State &state) 
    {
        RCLCPP_INFO(get_logger(), "Activating Lifecycle Velocity Estimation node");
        pub_->on_activate();
        RCLCPP_INFO(get_logger(), "Lifecycle Velocity Estimation Activated!");
        return ns_vel_est::CallbackReturn::SUCCESS;
    }

    ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_deactivate(const rclcpp_lifecycle::State &state) 
    {   
        RCLCPP_INFO(get_logger(), "Deactivating Lifecycle Velocity Estimation node");
        pub_->on_deactivate();
        RCLCPP_INFO(get_logger(), "Lifecycle Velocity Estimation Deactivated!");
        return ns_vel_est::CallbackReturn::SUCCESS;
    }

    ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_cleanup(const rclcpp_lifecycle::State &state) 
    {
        pub_.reset();
        return ns_vel_est::CallbackReturn::SUCCESS;
    }

    ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_shutdown(const rclcpp_lifecycle::State &state) 
    {  
        return ns_vel_est::CallbackReturn::SUCCESS;
    }

    ns_vel_est::CallbackReturn
        LifecycleVelocityEstimationHandler::on_error(const rclcpp_lifecycle::State &state)
    {
        return ns_vel_est::CallbackReturn::SUCCESS;
    }
}