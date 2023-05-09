#include "lifecycle/lifecycle_velocity_estimation.h"
#include "lifecycle/lifecycle_velocity_estimation_handler.hpp"

namespace ns_vel_est
{
        ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_configure(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Configuring Lifecycle Velocity Estimation node");

        loadParameters();
        cli_ = create_client<custom_msgs::srv::GetFrequencies>("get_frequencies");

        // Initialize the states object and set the time step
        node_frequency_ = getNodeFrequency();
        if (!node_frequency_)
        {
            rclcpp::shutdown();
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

        pub_ = create_publisher<custom_msgs::msg::VelEstimation>("velocity_estimation", 10);
        RCLCPP_INFO(get_logger(), "Lifecycle Velocity Estimation Configured!");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_activate(const rclcpp_lifecycle::State &state) 
    {
        RCLCPP_INFO(get_logger(), "Activating Lifecycle Velocity Estimation node");

        setSubscribers();
        // pub_.on_activate();

        RCLCPP_INFO(get_logger(), "Lifecycle Velocity Estimation Activated!");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_deactivate(const rclcpp_lifecycle::State &state) 
    {   
        RCLCPP_INFO(get_logger(), "Deactivating Lifecycle Velocity Estimation node");
        // pub_.on_deactivate();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        RCLCPP_INFO(get_logger(), "Lifecycle Velocity Estimation Deactivated!");
    }

    ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_cleanup(const rclcpp_lifecycle::State &state) 
    {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    ns_vel_est::CallbackReturn 
        LifecycleVelocityEstimationHandler::on_shutdown(const rclcpp_lifecycle::State &state) 
    {
        return ns_vel_est::CallbackReturn::SUCCESS;
    }

    ns_vel_est::CallbackReturn
        LifecycleVelocityEstimationHandler::on_error(const rclcpp_lifecycle::State &state)
    {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
}