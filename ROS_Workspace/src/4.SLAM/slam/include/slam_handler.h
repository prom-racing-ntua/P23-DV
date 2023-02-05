#ifndef SLAM_HANDLER_H
#define SLAM_HANDLER_H

#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <pthread.h>

#include "slam.h"
#include "custom_msgs/msg/perception2_slam.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"

// GTSAM Includes
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

namespace slam_namespace
{
    class GraphSLAM;
    class SLAM_handler : public rclcpp::Node {
    private:
        // SLAM object/class
        GraphSLAM SLAMObject_;

        // Setup Callback group for mutual exclusion
        rclcpp::CallbackGroup::SharedPtr slam_callback_group_;

        // ROS Subscribers
        // Perception Subscription
        rclcpp::Subscription<custom_msgs::msg::Perception2Slam>::SharedPtr perception_subscription_;

        // Velocity Estimation Subscription
        rclcpp::Subscription<custom_msgs::msg::VelEstimation>::SharedPtr velocity_estimation_subscription_;

        // ROS Publisher (Message for SLAM to publish to others)
        // rclcpp::Publisher<custom_msgs::msg::SLAMMessage>::SharedPtr slam_publisher_;

        // Index To Symbol Dictionary
        std::unordered_map<int, gtsam::Symbol> indexToSymbol;
        
        // Index to Pose Dictionary
        std::unordered_map<int, gtsam::Pose2> indexToPose;

        // Node log files
        std::ofstream lapCounterFile;
        std::ofstream poseLogFile;
        std::ofstream mapLogFile;
        std::ofstream velocityLogFile;
        std::ofstream perceptionLogFile;
        std::ofstream timeAnalysisLogFile;

        // Global Index
        int previousGlobalIndex;


        // Global lock for SLAM node
        pthread_spinlock_t globalLock;

        // Current Robot Symbol
        gtsam::Symbol currentRobotSymbol;

        // SLAM Timer
        rclcpp::TimerBase::SharedPtr OptimizationTimer_;
        rclcpp::TimerBase::SharedPtr PublishTimer_;

        // Lap counting material
        int laps_completed, cooldown, cooldown_max;

        // ROS setup Methods
        void setSubscribers();

        // Callback Methods
        void perceptionCallback(const custom_msgs::msg::Perception2Slam::SharedPtr msg);
        void velocityEstimationCallback(const custom_msgs::msg::VelEstimation::SharedPtr msg);

        // Timer Methods
        void optimizationCallback();

        void loadParameters();
        

    public:
        explicit SLAM_handler();
        void publishSLAM();

    }; //slam_namespace
}

#endif
