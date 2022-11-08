#ifndef VELOCITY_ESTIMATION_HANDLER_H
#define VELOCITY_ESTIMATION_HANDLER_H

#include <chrono>
#include <functional>
#include <vector>
#include <rclcpp/rclcpp.hpp>

// Include custom vectornav messages
#include "vectornav_msgs/msg/attitude_group.hpp"
#include "vectornav_msgs/msg/imu_group.hpp"
#include "vectornav_msgs/msg/ins_group.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "velocity_estimation.h"

namespace ns_vel_est
{
// Forward declaration of the VelocityEstimator class so it can be used for the estimator_ member variable
class VelocityEstimator;

class VelocityEstimationHandler : public rclcpp::Node {
private:
    int node_frequency_;
    VelocityEstimator estimator_;

    // ROS Subscriber members
    rclcpp::Subscription<vectornav_msgs::msg::InsGroup>::SharedPtr vn_velocity_sub_;     // vectornav vn-300
    // rclcpp::Subscription<vectornav_msgs::msg::AttitudeGroup>::SharedPtr vn_attitude_sub_;     // vectornav vn-300
    rclcpp::Subscription<vectornav_msgs::msg::ImuGroup>::SharedPtr vn_imu_sub_;        // vectornav vn-200
    // rclcpp::Subscription<>::SharedPtr hall_velocity_sub_;

    // ROS Timer to set the frequency of the algorithm
    rclcpp::TimerBase::SharedPtr timer_;

    // ROS Publisher to output results to the velocity estimation topic
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;

    std::array<bool, ObservationSize> update_vector_;
    ObservationVector measurement_vector_;

    void loadParameters();
    void setSubscribers();

    // Callbacks
    void timerCallback();
    void velocityCallback(const vectornav_msgs::msg::InsGroup::SharedPtr msg);
    // void attitudeCallback(const vectornav_msgs::msg::AttitudeGroup::SharedPtr msg);
    void imuCallback(const vectornav_msgs::msg::ImuGroup::SharedPtr msg);
    // void hallCallback();

public:
    explicit VelocityEstimationHandler();
    //~VelocityEstimationHandler();

    // Setters

    // Getters
    // Returns the frequency at witch the node executes the EKF algorithm
    int getNodeFrequency() const { return node_frequency_; }
    void publishResults();
};
} // namespace ns_vel_est

#endif // VELOCITY_ESTIMATION_HANDLER_H