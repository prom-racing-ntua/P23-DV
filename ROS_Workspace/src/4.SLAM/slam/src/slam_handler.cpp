#include <iostream>
#include <stdexcept>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

#include "slam_handler.h"


namespace ns_slam
{
SlamHandler::SlamHandler(): Node("slam_from_file_node"), slam_object_(this) {
    loadParameters();
    cli_ = create_client<custom_msgs::srv::GetFrequencies>("get_frequencies");

    competed_laps_ = -1;
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
        velocity_log_.open(share_dir_ + "/../../../../velocityLog_" + std::to_string(init_time) + ".txt");
        perception_log_.open(share_dir_ + "/../../../../perceptionLog_" + std::to_string(init_time) + ".txt");
    }

    // If in localization mode load the track map
    if (!is_mapping_)
    {
        std::string track_file{ share_dir_ + get_parameter("track_map").as_string()};
        slam_object_.loadMap(track_file);
    }
    else map_log_.open(share_dir_ + "/../../../../mapLog_" + std::to_string(init_time) + ".txt");

    //Initialize global lock
    if (pthread_spin_init(&global_lock_, PTHREAD_PROCESS_SHARED) != 0)
    {
        RCLCPP_ERROR(get_logger(), "Global lock initialization failed: exit program");
        exit(1);
    }

    auto sensor_qos{ rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data) };
    slam_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Set ROS objects
    rclcpp::SubscriptionOptions options;
    options.callback_group = slam_callback_group_;

    velocity_subscriber_ = create_subscription<custom_msgs::msg::VelEstimation>(
        "velocity_estimation", 10, std::bind(&SlamHandler::odometryCallback, this, std::placeholders::_1), options
    );
    perception_subscriber_ = create_subscription<custom_msgs::msg::Perception2Slam>(
        "perception2slam_topic", 10, std::bind(&SlamHandler::perceptionCallback, this, std::placeholders::_1), options
    );

    // pose_publisher_ = create_publisher
    // map_publisher_ = create_publisher


    optimization_clock_ = create_wall_timer(std::chrono::milliseconds(1000 * optimization_interval_ / node_frequency_),
        std::bind(&SlamHandler::optimizationCallback, this), slam_callback_group_);

    if (get_parameter("telemetry").as_bool())
    {
        landmark_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("landmark_marker_array", 10);
        car_pose_publisher_ = create_publisher<visualization_msgs::msg::Marker>("car_pose_marker", 10);
        telemetry_clock_ = create_wall_timer(std::chrono::milliseconds(50),
            std::bind(&SlamHandler::visualize, this), slam_callback_group_);
    }
    RCLCPP_WARN(get_logger(), "Created SlamHandler");
}

SlamHandler::~SlamHandler() {
    if (get_parameter("telemetry").as_bool())
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
    // odometry.velocity_y = static_cast<double>(msg->velocity_y);
    odometry.velocity_y = 0.0;
    odometry.yaw_rate = static_cast<double>(msg->yaw_rate);
    auto variance_array = static_cast<std::array<double, 9>>(msg->variance_matrix);
    odometry.measurement_noise = odometry_weight_ * Eigen::Map<gtsam::Matrix3>(variance_array.data());

    pthread_spin_lock(&global_lock_);
    bool is_completed_lap{ slam_object_.addOdometryMeasurement(odometry) };
    pthread_spin_unlock(&global_lock_);

    if (is_completed_lap && (cooldown_ == 0))
    {
        competed_laps_++;
        cooldown_ = cooldown_max_;
        RCLCPP_WARN(get_logger(), "Lap Completed!");
    }
    else if (cooldown_ > 0)
    {
        cooldown_--;
    }

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

    int observation_size{ color.size() };
    std::vector<PerceptionMeasurement> landmark_list{};

    for (int i{ 0 }; i < observation_size; i++)
    {
        PerceptionMeasurement landmark{};
        gtsam::Matrix2 observation_noise{};
        landmark.range = static_cast<double>(range[i]);

        // Only accept cones that are within the specified range
        if (landmark.range <= perception_range_)
        {
            landmark.color = static_cast<ConeColor>(color[i]);
            landmark.theta = static_cast<double>(theta[i]);
            // Setting observation noise depending on type of cone
            if (landmark.color == ConeColor::LargeOrange)
            {
                observation_noise << 0.001, 0.0,
                    0.0, perception_weight_* landmark.range / 10.0;
                landmark.observation_noise = observation_noise;
            }
            else
            {
                observation_noise << 0.01, 0.0,
                    0.0, 3.0 * perception_weight_ * landmark.range / 10.0;
                landmark.observation_noise = observation_noise;
            }
            // RCLCPP_INFO_STREAM(get_logger(), "Adding cone at range " << landmark.range << " m and angle " << landmark.theta << " rad.\n");
            landmark_list.push_back(landmark);
        }
    }

    if (!landmark_list.empty())
    {
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

    RCLCPP_WARN(get_logger(), "Starting Optimization");
    slam_object_.optimizeFactorGraph(opt_new_factors, opt_new_variable_values);
    RCLCPP_WARN(get_logger(), "Finished Optimization");

    pthread_spin_lock(&global_lock_);
    slam_object_.imposeOptimization(optimization_pose_symbol, pre_optimization_pose);

    // Keep map log
    if (is_mapping_)
    {
        std::vector<gtsam::Vector3> track{ slam_object_.getEstimatedMap() };
        map_log_ << optimization_pose_symbol.index() << '\n';
        for (auto cone : track) map_log_ << cone[0] << ' ' << cone[1] << ' ' << cone[2] << '\n';
    }
    pthread_spin_unlock(&global_lock_);

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

    declare_parameter<double>("relinearize_threshold", 0.1);
    declare_parameter<int>("relinearize_skip", 1);

    odometry_weight_ = declare_parameter<double>("odometry_covariance_weight", 10.0);
    perception_weight_ = declare_parameter<double>("perception_covariance_weight", 0.01);

    // Starting position variables
    declare_parameter<std::vector<double>>("starting_position", { -7.5, 0.0, 0.0 });
    declare_parameter<std::vector<double>>("starting_position_covariance", { 0.5, 0.1, 0.1 });

    declare_parameter<std::vector<double>>("left_orange", { 6.0, -3.0 });
    declare_parameter<std::vector<double>>("right_orange", { 6.0, 3.0 });
    cooldown_max_ = declare_parameter<int>("lap_counter_cooldown", 10);

    share_dir_ = ament_index_cpp::get_package_share_directory("slam");

    is_logging_ = declare_parameter<bool>("logger", true);
    declare_parameter<bool>("telemetry", true);
}

int SlamHandler::getNodeFrequency() {
    using namespace std::chrono_literals;

    // Instead of a timer we get the node frequency from the mater node with the following client request
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

void SlamHandler::visualize() {
    // Delete all existing markers
    visualization_msgs::msg::Marker delete_all_markers{};
    delete_all_markers.header.frame_id = "map";
    delete_all_markers.header.stamp = now();
    delete_all_markers.action = delete_all_markers.DELETEALL;
    delete_all_markers.ns = "my_ns";

    // Visualize car position
    pthread_spin_lock(&global_lock_);
    gtsam::Vector3 car_pose{ slam_object_.getEstimatedCarPose() };
    pthread_spin_unlock(&global_lock_);

    visualization_msgs::msg::Marker car_marker{};
    car_marker.header.frame_id = "map";
    car_marker.header.stamp = now();
    car_marker.ns = "my_ns";
    car_marker.id = 0;

    car_marker.type = visualization_msgs::msg::Marker::CUBE;
    car_marker.action = visualization_msgs::msg::Marker::ADD;

    // Minus y variable is for left handed system
    car_marker.pose.position.x = car_pose(1);
    car_marker.pose.position.y = car_pose(0);
    car_marker.pose.position.z = 0.7;

    tf2::Quaternion car_orientation;
    car_orientation.setRPY(0.0, 0.0, M_PI / 2 - car_pose(2));
    car_orientation.normalize();

    car_marker.pose.orientation = tf2::toMsg(car_orientation);

    car_marker.scale.x = 2.0;
    car_marker.scale.y = 1.22;
    car_marker.scale.z = 0.9;

    car_marker.color.r = 0.839;
    car_marker.color.g = 0.224;
    car_marker.color.b = 0.082;
    car_marker.color.a = 1.0;

    // Visualize landmarks seen by the car
    int id{ 0 };
    visualization_msgs::msg::MarkerArray cones_array{};
    visualization_msgs::msg::Marker cone_marker{};

    cone_marker.header.frame_id = "map";
    cone_marker.header.stamp = now();
    cone_marker.ns = "my_ns";
    cone_marker.action = cone_marker.ADD;
    cone_marker.type = cone_marker.CYLINDER;

    cone_marker.color.a = 1.0;
    cone_marker.scale.x = 0.3;
    cone_marker.scale.y = 0.3;
    cone_marker.scale.z = 0.4;
    cone_marker.pose.position.z = 0.2;
    cone_marker.pose.orientation.x = 0.0;
    cone_marker.pose.orientation.y = 0.0;
    cone_marker.pose.orientation.z = 0.0;
    cone_marker.pose.orientation.w = 1.0;

    pthread_spin_lock(&global_lock_);
    std::vector<gtsam::Vector3> track{ slam_object_.getEstimatedMap() };
    pthread_spin_unlock(&global_lock_);

    for (gtsam::Vector3& cone : track)
    {
        cone_marker.id = id;
        cone_marker.pose.position.x = cone(2);
        cone_marker.pose.position.y = cone(1);
        switch (static_cast<ConeColor>(cone(0)))
        {
        case ConeColor::Yellow:
            cone_marker.color.r = 1.0;
            cone_marker.color.g = 1.0;
            cone_marker.color.b = 0.0;
            break;
        case ConeColor::Blue:
            cone_marker.color.r = 0.0;
            cone_marker.color.g = 0.0;
            cone_marker.color.b = 0.8;
            break;
        case ConeColor::SmallOrange:
            cone_marker.color.r = 247.0 / 250.0;
            cone_marker.color.g = 140.0 / 250.0;
            cone_marker.color.b = 25.0 / 250.0;
            break;
        case ConeColor::LargeOrange:
            cone_marker.color.r = 117.0 / 250.0;
            cone_marker.color.g = 59.0 / 250.0;
            cone_marker.color.b = 29.0 / 250.0;
            break;
        default:
            RCLCPP_WARN(get_logger(), "SlamHandler() -> Invalid cone color encountered when reading map");
            break;
        }
        cones_array.markers.push_back(cone_marker);
        id++;
    }

    car_pose_publisher_->publish(delete_all_markers);
    car_pose_publisher_->publish(car_marker);
    landmark_publisher_->publish(cones_array);
}
} // namespace ns_slam


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto slam_node = std::make_shared<ns_slam::SlamHandler>();
    RCLCPP_INFO_STREAM(slam_node->get_logger(), ">>Check!");
    executor.add_node(slam_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}