#include "lifecycle_slam_handler.h"
#include "lifecycle_slam.h"

namespace ns_slam
{
    ns_slam::CallbackReturn
        LifecycleSlamHandler::on_configure(const rclcpp_lifecycle::State &state)
    {   
        get_parameter("velocity_estimation_frequency", node_frequency_);
        // get_parameter("perception_frequency")

        if (!node_frequency_)
        {
            RCLCPP_ERROR(get_logger(), "Could not get Velocity Estimation Frequency: SLAM Configuration failed");
            return ns_slam::CallbackReturn::FAILURE;
        }

        slam_object_.setDeltaTime(1.0 / static_cast<double>(node_frequency_));
        slam_object_.init();

        int init_time{ static_cast<int>(now().seconds()) };
        // Create Log files
        if (is_logging_)
        {
            velocity_log_.open(share_dir_ + "/../../../../testingLogs/velocityLog_" + std::to_string(init_time) + ".txt");
            perception_log_.open(share_dir_ + "/../../../../testingLogs/perceptionLog_" + std::to_string(init_time) + ".txt");
        }

        // If in localization mode load the track map
        if (!is_mapping_)
        {
            std::string track_file{ share_dir_ + get_parameter("track_map").as_string()};
            slam_object_.loadMap(track_file);
        }
        else 
            map_log_.open(share_dir_ + "/../../../../testingLogs/mapLog_" + std::to_string(init_time) + ".txt");

        //Initialize global lock
        if (pthread_spin_init(&global_lock_, PTHREAD_PROCESS_SHARED))
        {
            RCLCPP_ERROR(get_logger(), "Global lock initialization failed. Cannot configure node");
            return ns_slam::CallbackReturn::FAILURE;
        }

        auto sensor_qos{ rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data) };
        slam_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Set ROS objects
        rclcpp::SubscriptionOptions options;
        options.callback_group = slam_callback_group_;

        velocity_subscriber_ = create_subscription<custom_msgs::msg::VelEstimation>(
            "velocity_estimation", 10, std::bind(&LifecycleSlamHandler::odometryCallback, this, std::placeholders::_1), options
        );
        perception_subscriber_ = create_subscription<custom_msgs::msg::Perception2Slam>(
            "perception2slam_topic", 10, std::bind(&LifecycleSlamHandler::perceptionCallback, this, std::placeholders::_1), options
        );

        map_publisher_ = create_publisher<custom_msgs::msg::LocalMapMsg>("local_map", 10);
        pose_publisher_ = create_publisher<custom_msgs::msg::PoseMsg>("pose", 10);

        /* Create the optimization timer and cancel it immediately to stop it from activating */
        optimization_clock_ = create_wall_timer(std::chrono::milliseconds(1000 * optimization_interval_ / node_frequency_),
            std::bind(&LifecycleSlamHandler::optimizationCallback, this), slam_callback_group_);
        optimization_clock_->cancel();

        return ns_slam::CallbackReturn::SUCCESS;
    }

    ns_slam::CallbackReturn
        LifecycleSlamHandler::on_activate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Activating Lifecycle SLAM node");
        /* Activate Publishers and optimization Timer 
            These calls cannot fail so there is nothing to stop us from 
            going to active mode. */

        pose_publisher_->on_activate();
        map_publisher_->on_activate();
        optimization_clock_->cancel();
        optimization_clock_->reset();
        RCLCPP_INFO(get_logger(), "Lifecycle SLAM node Activated!");
        return ns_slam::CallbackReturn::SUCCESS;
    }

    ns_slam::CallbackReturn
        LifecycleSlamHandler::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Deactivating Lifecycle SLAM node");
        /* De-activate Publishers and optimization Timer 
            These calls cannot fail so there is nothing to stop us from 
            going to inactive mode. */
        pose_publisher_->on_deactivate();
        map_publisher_->on_deactivate();
        optimization_clock_->cancel();
        RCLCPP_INFO(get_logger(), "Lifecycle SLAM node deactivated!");
        return ns_slam::CallbackReturn::SUCCESS;
    }

    ns_slam::CallbackReturn
        LifecycleSlamHandler::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        pose_publisher_.reset();
        map_publisher_.reset();
        if (pthread_spin_destroy(&global_lock_))
        {
            RCLCPP_ERROR(get_logger(), "Global lock destruction failed cannot go to unconfigured state");
            return ns_slam::CallbackReturn::FAILURE;
        }
        return ns_slam::CallbackReturn::SUCCESS;
    }

    ns_slam::CallbackReturn
        LifecycleSlamHandler::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Deactivating Lifecycle SLAM node");
        /* De-activate Publishers and optimization Timer 
            These calls cannot fail so there is nothing to stop us from 
            going to inactive mode. */
        pose_publisher_->on_deactivate();
        map_publisher_->on_deactivate();
        optimization_clock_->cancel();

        pose_publisher_.reset();
        map_publisher_.reset();

        return ns_slam::CallbackReturn::SUCCESS;
    }

    ns_slam::CallbackReturn
        LifecycleSlamHandler::on_error(const rclcpp_lifecycle::State &state)
    {
        return ns_slam::CallbackReturn::SUCCESS;
    }
}