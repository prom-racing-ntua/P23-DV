#include "lifecycle_slam_handler.h"


namespace ns_slam
{
    ns_slam::CallbackReturn
        LifecycleSlamHandler::on_configure(const rclcpp_lifecycle::State)
    {   
        node_frequency_ = get_parameter("velocity_estimation_frequency").as_int();

        if (!node_frequency_)
        {
            RCLCPP_ERROR(get_logger(), "Could not get Velocity Estimation Frequency: SLAM Configuration failed");
            return ns_slam::CallbackReturn::FAILURE;
        }

        // Get Parameters
        is_mapping_ = get_parameter("mapping_mode").as_bool();
        perception_range_ = get_parameter("perception_range").as_double();
        optimization_interval_ = get_parameter("optimization_interval").as_int();
        odometry_weight_ = get_parameter("odometry_covariance_weight").as_double();
        perception_weight_ = get_parameter("perception_covariance_weight").as_double();
        cooldown_max_ = get_parameter("lap_counter_cooldown").as_int();
        is_logging_ = get_parameter("logger").as_bool();

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
        map_ready_ = true;
        if (!is_mapping_)
        {
            std::string track_file{ share_dir_ + get_parameter("track_map").as_string()};
            slam_object_.loadMap(track_file);
            // Check if we are using dynamic map creation for acceleration
            if (get_parameter("track_map").as_string() == std::string("/test_tracks/Acceleration.txt"))
            {
                map_ready_ = !get_parameter("dynamic_accel_map").as_bool();
            }
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
            "perception2slam", 10, std::bind(&LifecycleSlamHandler::perceptionCallback, this, std::placeholders::_1), options
        );

        map_publisher_ = create_publisher<custom_msgs::msg::LocalMapMsg>("local_map", 10);
        pose_publisher_ = create_publisher<custom_msgs::msg::PoseMsg>("pose", 10);

        /* Create the optimization timer and cancel it immediately to stop it from activating */
        optimization_clock_ = create_wall_timer(std::chrono::milliseconds(1000 * optimization_interval_ / node_frequency_),
            std::bind(&LifecycleSlamHandler::optimizationCallback, this), slam_callback_group_);
        optimization_clock_->cancel();

        RCLCPP_WARN(get_logger(), "Lifecycle SLAM node Configured!");
        return ns_slam::CallbackReturn::SUCCESS;
    }

    ns_slam::CallbackReturn
        LifecycleSlamHandler::on_activate(const rclcpp_lifecycle::State)
    {
        /* Activate Publishers and optimization Timer 
            These calls cannot fail so there is nothing to stop us from 
            going to active mode. */

        pose_publisher_->on_activate();
        map_publisher_->on_activate();
        optimization_clock_->cancel();
        optimization_clock_->reset();
        RCLCPP_WARN(get_logger(), "Lifecycle SLAM node Activated!");
        return ns_slam::CallbackReturn::SUCCESS;
    }

    ns_slam::CallbackReturn
        LifecycleSlamHandler::on_deactivate(const rclcpp_lifecycle::State)
    {
        /* De-activate Publishers and optimization Timer 
            These calls cannot fail so there is nothing to stop us from 
            going to inactive mode. */
        pose_publisher_->on_deactivate();
        map_publisher_->on_deactivate();
        optimization_clock_->cancel();
        RCLCPP_WARN(get_logger(), "Lifecycle SLAM node Deactivated!");
        return ns_slam::CallbackReturn::SUCCESS;
    }

    ns_slam::CallbackReturn
        LifecycleSlamHandler::on_cleanup(const rclcpp_lifecycle::State)
    {
        completed_laps_ = -1;
        cooldown_ = 0;
        slam_object_.reset();

        pose_publisher_.reset();
        map_publisher_.reset();
        if (pthread_spin_destroy(&global_lock_))
        {
            RCLCPP_ERROR(get_logger(), "Global lock destruction failed cannot go to unconfigured state");
            return ns_slam::CallbackReturn::FAILURE;
        }

        if (is_logging_)
        {
            velocity_log_.close();
            perception_log_.close();
        }

        if (is_mapping_) map_log_.close();

        RCLCPP_WARN(get_logger(), "Lifecycle SLAM node Un-Configured!");
        return ns_slam::CallbackReturn::SUCCESS;
    }

    ns_slam::CallbackReturn
        LifecycleSlamHandler::on_shutdown(const rclcpp_lifecycle::State)
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

        if (is_logging_)
        {
            velocity_log_.close();
            perception_log_.close();
        }

        if (is_mapping_) map_log_.close();

        return ns_slam::CallbackReturn::SUCCESS;
    }

    ns_slam::CallbackReturn
        LifecycleSlamHandler::on_error(const rclcpp_lifecycle::State)
    {
        return ns_slam::CallbackReturn::SUCCESS;
    }
}