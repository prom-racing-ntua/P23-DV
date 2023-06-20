#include <iostream>
#include <stdexcept>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

#include "slam_handler.h"


namespace ns_slam
{
SlamHandler::SlamHandler() : Node("slam_node"), slam_object_(this) {
    loadParameters();
    cli_ = create_client<custom_msgs::srv::GetFrequencies>("get_frequencies");

    completed_laps_ = 0;
    cooldown_ = 0;

    // Initialize slam
    node_frequency_ = getNodeFrequency();
    if (!node_frequency_)
    {
        rclcpp::shutdown();
        return;
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

    else map_log_.open(share_dir_ + "/../../../../testingLogs/mapLog_" + std::to_string(init_time) + ".txt");

    //Initialize global lock
    if (pthread_spin_init(&global_lock_, PTHREAD_PROCESS_SHARED) != 0)
    {
        RCLCPP_ERROR(get_logger(), "Global lock initialization failed: exit program");
        exit(1);
    }

    auto sensor_qos{ rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data) };
    slam_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Set ROS objects
    rclcpp::SubscriptionOptions options;
    options.callback_group = slam_callback_group_;

    velocity_subscriber_ = create_subscription<custom_msgs::msg::VelEstimation>(
        "velocity_estimation", 10, std::bind(&SlamHandler::odometryCallback, this, std::placeholders::_1), options
    );
    perception_subscriber_ = create_subscription<custom_msgs::msg::Perception2Slam>(
        "perception2slam_topic", 10, std::bind(&SlamHandler::perceptionCallback, this, std::placeholders::_1), options
    );

    map_publisher_ = create_publisher<custom_msgs::msg::LocalMapMsg>("local_map", 10);
    pose_publisher_ = create_publisher<custom_msgs::msg::PoseMsg>("pose", 10);

    optimization_clock_ = create_wall_timer(std::chrono::milliseconds(1000 * optimization_interval_ / node_frequency_),
        std::bind(&SlamHandler::optimizationCallback, this), slam_callback_group_);

    RCLCPP_WARN(get_logger(), "Created SlamHandler");
}

SlamHandler::~SlamHandler() {
    if (is_logging_)
    {
        velocity_log_.close();
        perception_log_.close();
    }

    if (is_mapping_) map_log_.close();
}

void SlamHandler::odometryCallback(const custom_msgs::msg::VelEstimation::SharedPtr msg) {
    rclcpp::Time starting_time{ this->now() };

    // Set the structure that will be passed to the slam_object_ member
    OdometryMeasurement odometry{};
    odometry.global_index = static_cast<unsigned long>(msg->global_index);
    odometry.velocity_x = static_cast<double>(msg->velocity_x);
    odometry.velocity_y = static_cast<double>(msg->velocity_y);
    odometry.yaw_rate = static_cast<double>(msg->yaw_rate);
    auto variance_array = static_cast<std::array<double, 9>>(msg->variance_matrix);
    odometry.measurement_noise = Eigen::Map<gtsam::Matrix3>(variance_array.data());

    pthread_spin_lock(&global_lock_);
    bool is_completed_lap{ slam_object_.addOdometryMeasurement(odometry) };
    gtsam::Vector3 current_pose{ slam_object_.getEstimatedCarPose() };
    pthread_spin_unlock(&global_lock_);

    if (is_completed_lap && (cooldown_ == 0))
    {
        completed_laps_++;
        cooldown_ = cooldown_max_;
        RCLCPP_WARN(get_logger(), "Lap Completed!");
    }
    else if (cooldown_ > 0)
    {
        cooldown_--;
    }

    last_vel_msg_ = *msg;

    // Publish pose message
    custom_msgs::msg::PoseMsg pose_msg{};
    pose_msg.position.x = current_pose[0];
    pose_msg.position.y = current_pose[1];
    pose_msg.theta = current_pose[2];
    pose_msg.velocity_state = last_vel_msg_;
    pose_publisher_->publish(pose_msg);

    // Keep odometry log
    if (is_logging_)
    {
        velocity_log_ << odometry.global_index << '\n' << odometry.velocity_x << '\n' << odometry.velocity_y << '\n' << odometry.yaw_rate << '\n';
        for (auto val : variance_array) velocity_log_ << val << ' ';
        velocity_log_ << '\n';
    }
    // Print computation time
    rclcpp::Duration total_time{ this->now() - starting_time };
    // RCLCPP_INFO_STREAM(get_logger(), "\n-- Odometry Callback --\nTime of execution " << total_time.nanoseconds() / 1000000.0 << " ms.");
}

void SlamHandler::perceptionCallback(const custom_msgs::msg::Perception2Slam::SharedPtr msg) {
    rclcpp::Time starting_time{ this->now() };

    auto color{ static_cast<std::vector<int>>(msg->class_list) };
    auto range{ static_cast<std::vector<float>>(msg->range_list) };
    auto theta{ static_cast<std::vector<float>>(msg->theta_list) };

    int observation_size{ static_cast<int>(color.size()) };
    std::vector<PerceptionMeasurement> landmark_list{};

    for (int i{ 0 }; i < observation_size; i++)
    {
        PerceptionMeasurement landmark{};
        gtsam::Matrix2 observation_noise{};
        landmark.range = static_cast<double>(range[i]);

        // Only accept cones that are within the specified range
        if (landmark.range <= 6.0 or (abs(landmark.theta <= 0.7) and landmark.range <= perception_range_))
        {
            landmark.color = static_cast<ConeColor>(color[i]);
            landmark.theta = static_cast<double>(theta[i]);
            // Setting observation noise depending on type of cone
            if (landmark.color == ConeColor::LargeOrange)
            {
                observation_noise << 0.001, 0,
					0, 3 * (0.011*std::pow(landmark.range+1,2) - 0.082*(landmark.range+1) + 0.187);
            }
            else
            {
                observation_noise << 0.0001, 0,
					0, 0.011*std::pow(landmark.range+1,2) - 0.082*(landmark.range+1) + 0.187;
            }
            landmark.observation_noise = observation_noise;
            // RCLCPP_INFO_STREAM(get_logger(), "Adding cone at range " << landmark.range << " m and angle " << landmark.theta << " rad.\n");
            landmark_list.push_back(landmark);
        }
    }

    if (!landmark_list.empty())
    {
        perception_count_ = landmark_list.size();

        pthread_spin_lock(&global_lock_);
        if (is_mapping_) slam_object_.addLandmarkMeasurementSLAM(static_cast<unsigned long>(msg->global_index), landmark_list);
        else slam_object_.addLandmarkMeasurementsLocalization(static_cast<unsigned long>(msg->global_index), landmark_list);
        pthread_spin_unlock(&global_lock_);
    }
    landmark_list.clear();

    // Keep perception log
    if (is_logging_)
    {
        perception_log_ << static_cast<unsigned long>(msg->global_index) << '\n';
        for (auto col : color) perception_log_ << col << ' ';
        perception_log_ << '\n';
        for (auto rng : range) perception_log_ << rng << ' ';
        perception_log_ << '\n';
        for (auto th : theta) perception_log_ << th << ' ';
        perception_log_ << '\n';
    }
    // Print computation time
    rclcpp::Duration total_time{ this->now() - starting_time };
    // RCLCPP_INFO_STREAM(get_logger(), "\n-- Perception Callback --\nTime of execution " << total_time.nanoseconds() / 1000000.0 << " ms.");
}

void SlamHandler::optimizationCallback() {
    rclcpp::Time starting_time{ this->now() };

    pthread_spin_lock(&global_lock_);
    // Copy new factors and values to new variables that cannot be mutated while the update method runs.
    // As soon as we copy them empty the class member variables so no new measurements are lost.
    gtsam::NonlinearFactorGraph opt_new_factors{ slam_object_.getNewFactors() };
    gtsam::Values opt_new_variable_values{ slam_object_.getNewValues() };
    gtsam::Vector3 pre_optimization_pose{ slam_object_.getEstimatedCarPose() };
    gtsam::Symbol optimization_pose_symbol{ slam_object_.getCurrentPoseSymbol() };
    slam_object_.resetTemporaryGraph();
    pthread_spin_unlock(&global_lock_);

    // RCLCPP_WARN(get_logger(), "Starting Optimization");
    slam_object_.optimizeFactorGraph(opt_new_factors, opt_new_variable_values);
    // RCLCPP_WARN(get_logger(), "Finished Optimization");

    pthread_spin_lock(&global_lock_);
    slam_object_.imposeOptimization(optimization_pose_symbol, pre_optimization_pose);
    std::vector<gtsam::Vector3> track{ slam_object_.getEstimatedMap() };
    gtsam::Vector3 current_pose{ slam_object_.getEstimatedCarPose() };
    pthread_spin_unlock(&global_lock_);

    // Keep map log and publish map
    custom_msgs::msg::LocalMapMsg map_msg{};
    custom_msgs::msg::ConeStruct cone_msg{};
    
    if (is_mapping_)
    {
        map_log_ << optimization_pose_symbol.index() << '\n';
        for (auto cone : track)
        {
            map_log_ << cone[0] << ' ' << cone[1] << ' ' << cone[2] << '\n';

            cone_msg.color = cone[0];
            cone_msg.coords.x = cone[1];
            cone_msg.coords.y = cone[2];
            map_msg.local_map.push_back(cone_msg);
        }
    }

    map_msg.cones_count_actual = perception_count_;
    perception_count_ = 0;
    map_msg.pose.position.x = current_pose[0];
    map_msg.pose.position.y = current_pose[1];
    map_msg.pose.theta = current_pose[2];
    map_msg.pose.velocity_state = last_vel_msg_;

    if (completed_laps_ < 0) { map_msg.lap_count = 0; }
    else { map_msg.lap_count = completed_laps_; }
    map_msg.cones_count_all = slam_object_.getConeCount();

    map_publisher_->publish(map_msg);

    // Print computation time
    rclcpp::Duration total_time{ this->now() - starting_time };
    // RCLCPP_INFO_STREAM(get_logger(), "\n-- Optimization Callback --\nTime of execution " << total_time.nanoseconds() / 1000000.0 << " ms.");
}

void SlamHandler::loadParameters() {
    is_mapping_ = declare_parameter<bool>("mapping_mode", true);
    declare_parameter<std::string>("track_map", "");

    perception_range_ = declare_parameter<double>("perception_range", 14.0);
    optimization_interval_ = declare_parameter<int>("optimization_interval", 20);

    declare_parameter<double>("association_threshold", 1.9);
    declare_parameter<int>("min_observations", 3);

    declare_parameter<double>("relinearize_threshold", 0.1);
    declare_parameter<int>("relinearize_skip", 1);

    odometry_weight_ = declare_parameter<double>("odometry_covariance_weight", 10.0);
    perception_weight_ = declare_parameter<double>("perception_covariance_weight", 0.01);

    // Starting position variables
    declare_parameter<std::vector<double>>("starting_position", { -7.5, 0.0, 0.0 });
    declare_parameter<std::vector<double>>("starting_position_covariance", { 0.5, 0.1, 0.1 });

    declare_parameter<std::vector<double>>("left_orange", { 6.0, -1.5 });
    declare_parameter<std::vector<double>>("right_orange", { 6.0, 1.5 });
    cooldown_max_ = declare_parameter<int>("lap_counter_cooldown", 10);

    share_dir_ = ament_index_cpp::get_package_share_directory("slam");

    is_logging_ = declare_parameter<bool>("logger", true);
}

int SlamHandler::getNodeFrequency() {
    using namespace std::chrono_literals;

    // Instead of a timer we get the node frequency from the master node with the following client request
    auto request{ std::make_shared<custom_msgs::srv::GetFrequencies::Request>() };
    int call_counter{ 0 };
    while (!cli_->wait_for_service(1s) and call_counter < 15)
    {
        if (!rclcpp::ok())
        {
            return 0;
        }
        RCLCPP_INFO(get_logger(), "Could not get node frequency. Master service not available, waiting...");
        // call_counter++;
    }
    if (call_counter == 15)
    {
        RCLCPP_ERROR(get_logger(), "Client call timeout, the service is not available. Check master node.");
        return 0;
    }
    // Send empty request
    auto result{ cli_->async_send_request(request) };
    // Await for response (TODO: Set a timeout for response time)
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result, 5s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // If get successful response return the node frequency
        RCLCPP_INFO_STREAM(get_logger(), "Node frequency has been set to " << result.get()->velocity_estimation_frequency);
        return result.get()->velocity_estimation_frequency;
    }
    else
    {
        // Otherwise raise an error (TODO: should actually do something else, or handle the error)
        RCLCPP_ERROR(get_logger(), "Failed to get node frequency");
        return 0;
    }
}
} // namespace ns_slam


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto options{ rclcpp::ExecutorOptions() };
    rclcpp::executors::MultiThreadedExecutor executor{ options, 4 };

    auto slam_node = std::make_shared<ns_slam::SlamHandler>();
    executor.add_node(slam_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}