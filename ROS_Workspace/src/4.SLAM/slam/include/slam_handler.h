#ifndef SLAM_HANDLER_H
#define SLAM_HANDLER_H

#include <string>
#include <limits>
#include <chrono>
#include <functional>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "custom_msgs/msg/vel_estimation.hpp"
#include "custom_msgs/msg/perception2_slam.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/srv/get_frequencies.hpp"

#include "slam.h"


namespace ns_slam
{
class SlamHandler : public rclcpp::Node {
private:
    int node_frequency_;
    unsigned long global_index_;
    GraphSLAM<SlamHandler> slam_object_;

    bool is_mapping_;
    std::ofstream map_log_;

    double perception_range_;
    int optimization_interval_;

    double odometry_weight_;
    double perception_weight_;

    std::string share_dir_;

    bool is_logging_;
    std::ofstream velocity_log_;
    std::ofstream perception_log_;

    // Lap counter variables
    int completed_laps_;
    int cooldown_;
    int cooldown_max_;

    // Last velocity msg received
    custom_msgs::msg::VelEstimation last_vel_msg_;

    // Global lock for SLAM node
    pthread_spinlock_t global_lock_;

    // Callback group for threading
    rclcpp::CallbackGroup::SharedPtr slam_callback_group_;

    // Subscribers to Velocity and Perception topics
    rclcpp::Subscription<custom_msgs::msg::VelEstimation>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<custom_msgs::msg::Perception2Slam>::SharedPtr perception_subscriber_;

    // Slam topics publishers
    rclcpp::Publisher<custom_msgs::msg::LocalMapMsg>::SharedPtr map_publisher_;
    rclcpp::Publisher<custom_msgs::msg::PoseMsg>::SharedPtr pose_publisher_;

    // Optimization timer
    rclcpp::TimerBase::SharedPtr optimization_clock_;
    rclcpp::TimerBase::SharedPtr telemetry_clock_;

    // Velocity Estimation getter in order to set time interval in slam_object_
    rclcpp::Client<custom_msgs::srv::GetFrequencies>::SharedPtr cli_;

    // Load ROS parameters from config files
    void loadParameters();
    int getNodeFrequency();

    void odometryCallback(const custom_msgs::msg::VelEstimation::SharedPtr msg);

    void perceptionCallback(const custom_msgs::msg::Perception2Slam::SharedPtr msg);

    void optimizationCallback();

public:
    // The file paths are passed as arguments to the constructor and can be modified in the main function
    SlamHandler();

    ~SlamHandler();
};
} // namespace ns_slam

#endif // SLAM_HANDLER_H