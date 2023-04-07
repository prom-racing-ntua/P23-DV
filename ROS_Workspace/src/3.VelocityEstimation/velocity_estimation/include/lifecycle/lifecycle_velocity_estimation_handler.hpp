#ifndef LIFECYCLE_VELOCITY_ESTIMATION_HANDLER_HPP
#define LIFECYCLE_VELOCITY_ESTIMATION_HANDLER_HPP

#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

// Include custom vectornav messages
#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"
#include "custom_msgs/msg/wheel_speed.hpp"
#include "custom_msgs/msg/steering_angle.hpp"
#include "custom_msgs/msg/node_sync.hpp"
#include "custom_msgs/srv/get_frequencies.hpp"

#include "velocity_estimation.h"

namespace ns_vel_est
{
    // Forward declaration of the VelocityEstimator class so it can be used for the estimator_ member variable
    class VelocityEstimator;

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class LifecycleVelocityEstimationHandler: public rclcpp_lifecycle::LifecycleNode {
    private:
        int node_frequency_;
        unsigned long global_index_;
        VelocityEstimator estimator_;

        // ROS Subscriber members
        rclcpp::Subscription<vectornav_msgs::msg::InsGroup>::SharedPtr vn_velocity_sub_;     // vectornav vn-300
        // rclcpp::Subscription<vectornav_msgs::msg::AttitudeGroup>::SharedPtr vn_attitude_sub_;     // vectornav vn-300
        rclcpp::Subscription<vectornav_msgs::msg::ImuGroup>::SharedPtr vn_imu_sub_;        // vectornav vn-200
        rclcpp::Subscription<custom_msgs::msg::WheelSpeed>::SharedPtr front_wheel_encoder_sub_;
        rclcpp::Subscription<custom_msgs::msg::WheelSpeed>::SharedPtr rear_wheel_encoder_sub_;
        rclcpp::Subscription<custom_msgs::msg::SteeringAngle>::SharedPtr steering_sub_;

        // ROS Subscriber to the master node to sync velocity estimation and perception. The frequency of the node is determined by
        // this subscriber and is set in the master node config.
        rclcpp::Subscription<custom_msgs::msg::NodeSync>::SharedPtr master_sub_;

        // ROS Timer to set the frequency of the algorithm
        // rclcpp::TimerBase::SharedPtr timer_;

        // ROS Publisher to output results to the velocity estimation topic
        rclcpp::Publisher<custom_msgs::msg::VelEstimation>::SharedPtr pub_;

        // ROS Client to get the node frequency from the master node
        rclcpp::Client<custom_msgs::srv::GetFrequencies>::SharedPtr cli_;

        std::array<bool, SensorSize> updated_sensors_;
        ObservationVector measurement_vector_;
        InputVector input_vector_;

        Eigen::Matrix<double, 3, 3> vn_200_rotation_matrix_;
        Eigen::Matrix<double, 3, 3> vn_300_rotation_matrix_;

        Eigen::Matrix<double, 3, 3> getRotationMatrix(double roll, double pitch, double yaw = 0.0);

        void loadParameters();
        void setSubscribers();
        int getNodeFrequency();

        // Callbacks
        void masterCallback(const custom_msgs::msg::NodeSync::SharedPtr msg);
        void velocityCallback(const vectornav_msgs::msg::InsGroup::SharedPtr msg);
        // void attitudeCallback(const vectornav_msgs::msg::AttitudeGroup::SharedPtr msg);
        void imuCallback(const vectornav_msgs::msg::ImuGroup::SharedPtr msg);
        void frontWheelSpeedCallback(const custom_msgs::msg::WheelSpeed::SharedPtr msg);
        void rearWheelSpeedCallback(const custom_msgs::msg::WheelSpeed::SharedPtr msg);
        void steeringCallback(const custom_msgs::msg::SteeringAngle::SharedPtr msg);

    public:
        explicit LifecycleVelocityEstimationHandler();
        //~LifecycleVelocityEstimationHandler();
        void publishResults();

        // Setters

        // Getters
        // Returns the frequency at witch the node executes the EKF algorithm
        int getNodeFrequency() const { return node_frequency_; }
        
    protected:
            ns_vel_est::CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
            ns_vel_est::CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
            ns_vel_est::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
            ns_vel_est::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
            ns_vel_est::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
            ns_vel_est::CallbackReturn on_error(const rclcpp_lifecycle::State & state);
    };
} // namespace ns_vel_est

#endif // VELOCITY_ESTIMATION_HANDLER_H