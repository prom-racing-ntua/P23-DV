#ifndef LIFECYCLE_SLAM_HANDLER_H
#define LIFECYCLE_SLAM_HANDLER_H

#include <string>
#include <limits>
#include <chrono>
#include <cstdio>
#include <functional>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "custom_msgs/msg/vel_estimation.hpp"
#include "custom_msgs/msg/perception2_slam.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/srv/get_frequencies.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "slam.h"


namespace ns_slam
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class LifecycleSlamHandler : public rclcpp_lifecycle::LifecycleNode {
    private:
        unsigned long global_index_;
        GraphSLAM<LifecycleSlamHandler> slam_object_;
        // Global lock for SLAM node
        pthread_spinlock_t global_lock_;

        // Node parameters
        int node_frequency_;
        bool is_mapping_;
        bool is_logging_;

        double perception_range_;
        int optimization_interval_;

        // Not used right now
        double odometry_weight_;
        double perception_weight_;

        // Dynamic accel map variables
        bool map_ready_;
        int accel_cone_count_;
        int num_observations_;
	    std::unordered_map<int, LandmarkInfo> accel_map_;

        // Log file variables
        std::string share_dir_;
        std::ofstream velocity_log_;
        std::ofstream perception_log_;
        std::ofstream map_log_;
        FILE *perception_timestamp_log, *odometry_timestamp_log, *optim_timestamp_log, *run_idx_file;
        double pub_time_1, pub_time_2;

        // Lap counter variables
        int completed_laps_;
        int cooldown_;
        int cooldown_max_;

        // Current perception cone count
        size_t perception_count_;

        // Last velocity msg received
        custom_msgs::msg::VelEstimation last_vel_msg_;

        // Callback group for threading
        rclcpp::CallbackGroup::SharedPtr slam_callback_group_;

        // Subscribers to Velocity and Perception topics
        rclcpp::Subscription<custom_msgs::msg::VelEstimation>::SharedPtr velocity_subscriber_;
        rclcpp::Subscription<custom_msgs::msg::Perception2Slam>::SharedPtr perception_subscriber_;

        // Slam topics publishers
        rclcpp_lifecycle::LifecyclePublisher<custom_msgs::msg::LocalMapMsg>::SharedPtr map_publisher_;
        rclcpp_lifecycle::LifecyclePublisher<custom_msgs::msg::PoseMsg>::SharedPtr pose_publisher_;

        // Optimization timer
        rclcpp::TimerBase::SharedPtr optimization_clock_;
        rclcpp::TimerBase::SharedPtr telemetry_clock_;

        // Load ROS parameters from config files
        void loadParameters();

        void odometryCallback(const custom_msgs::msg::VelEstimation::SharedPtr msg);

        void perceptionCallback(const custom_msgs::msg::Perception2Slam::SharedPtr msg);

        void optimizationCallback();

        void addAccelObservations(const std::vector<PerceptionMeasurement>& observations);

    public:
        LifecycleSlamHandler();
        ~LifecycleSlamHandler();
    protected:
        ns_slam::CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
        ns_slam::CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
        ns_slam::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
        ns_slam::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
        ns_slam::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
        ns_slam::CallbackReturn on_error(const rclcpp_lifecycle::State & state);
    };
} // namespace ns_slam

#endif