#include <iostream>
#include <stdexcept>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

#include "lifecycle_slam_handler.h"


namespace ns_slam
{
LifecycleSlamHandler::LifecycleSlamHandler() : LifecycleNode("slam"), slam_object_(this) {
    loadParameters();
    completed_laps_ = 0;
    cooldown_ = 0;
    perception_count_ = 0;

    accel_cone_count_ = 0;
    num_observations_ = 0;
    accel_map_.clear();

    RCLCPP_WARN(get_logger(), "\n-- SLAM Node Created");
}

LifecycleSlamHandler::~LifecycleSlamHandler() {}

void LifecycleSlamHandler::odometryCallback(const custom_msgs::msg::VelEstimation::SharedPtr msg) {
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
    pose_msg.velocity_state = *msg;
    pose_msg.lap_count = completed_laps_;
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

void LifecycleSlamHandler::perceptionCallback(const custom_msgs::msg::Perception2Slam::SharedPtr msg) {
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
        perception_count_ += landmark_list.size();
        if (!map_ready_) {
            addAccelObservations(landmark_list);
            return;
        }

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

void LifecycleSlamHandler::optimizationCallback() {
    if (!map_ready_) return;

    rclcpp::Time starting_time{ this->now() };
    // RCLCPP_INFO(get_logger(), "SLAM Optimization");

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

    if (is_mapping_) map_log_ << optimization_pose_symbol.index() << '\n';
    for (auto cone : track)
    {
        if (is_mapping_) map_log_ << cone[0] << ' ' << cone[1] << ' ' << cone[2] << '\n';

        cone_msg.color = cone[0];
        cone_msg.coords.x = cone[1];
        cone_msg.coords.y = cone[2];
        map_msg.local_map.push_back(cone_msg);
    }

    map_msg.cones_count_actual = perception_count_;
    perception_count_ = 0;
    map_msg.pose.position.x = current_pose[0];
    map_msg.pose.position.y = current_pose[1];
    map_msg.pose.theta = current_pose[2];
    map_msg.pose.velocity_state = last_vel_msg_;
    map_msg.lap_count = completed_laps_;
    map_msg.cones_count_all = slam_object_.getConeCount();

    map_publisher_->publish(map_msg);

    // Print computation time
    rclcpp::Duration total_time{ this->now() - starting_time };
    // RCLCPP_INFO_STREAM(get_logger(), "\n-- Optimization Callback --\nTime of execution " << total_time.nanoseconds() / 1000000.0 << " ms.");
}

void LifecycleSlamHandler::addAccelObservations(const std::vector<PerceptionMeasurement>& observations) {
    num_observations_++;
    for (auto& cone : observations) {
        gtsam::Vector2 observed_position;
		observed_position << cone.range * std::cos(cone.theta), cone.range * std::sin(cone.theta);

        // Data Association with already observed cones
        int best_match = -1;
	    double least_distance_square{ std::pow(1.5, 2) };

	    if (cone.color == ConeColor::LargeOrange)
	    {
	    	least_distance_square = std::pow(1.4 * 1.5, 2);
	    }

	    // Iterate through all of the cones in the current map
	    for (auto& it : accel_map_)
	    {
	    	LandmarkInfo& mapped_cone{ it.second };
	    	// Only check cones of the same color
	    	if (mapped_cone.color == cone.color)
	    	{
	    		// Calculate the distance between the observed cone and the mapped cone
	    		double current_distance_square = (observed_position - mapped_cone.estimated_pose).transpose() * (observed_position - mapped_cone.estimated_pose);
	    		// If their distance is less than the previous best, assign the current cone as the best match
	    		if (current_distance_square < least_distance_square)
	    		{
	    			least_distance_square = current_distance_square;
	    			best_match = it.first;
	    		}
	    	}
	    }

        if (best_match == -1)
        {
			// Create the landmark entry
			LandmarkInfo new_landmark;
			new_landmark.color = cone.color;
			new_landmark.estimated_pose = observed_position;
			new_landmark.range_vector.push_back(cone.range);
			new_landmark.theta_vector.push_back(cone.theta);
			accel_map_[accel_cone_count_++] = new_landmark;
		}
        else
        {
            LandmarkInfo& matched_cone{ accel_map_.at(best_match) };
            matched_cone.range_vector.push_back(cone.range);
			matched_cone.theta_vector.push_back(cone.theta);

            double avg_range{ std::accumulate(matched_cone.range_vector.begin(), matched_cone.range_vector.end(), 0.0) / matched_cone.range_vector.size() };
            double avg_theta{ std::accumulate(matched_cone.theta_vector.begin(), matched_cone.theta_vector.end(), 0.0) / matched_cone.theta_vector.size() };

			matched_cone.estimated_pose[0] = avg_range * std::cos(avg_theta);
			matched_cone.estimated_pose[1] = avg_range * std::sin(avg_theta);
        }
    }

    if (num_observations_ >= 100)
    {
        double track_width{100.0};
        for (auto& yellow_cone : accel_map_)
        {
            if (yellow_cone.second.color != ConeColor::Yellow) {continue;}
            for (auto& blue_cone : accel_map_) 
            {
                if (blue_cone.second.color != ConeColor::Blue) {continue;}
                double  dist{std::sqrt( std::pow(yellow_cone.second.estimated_pose(0)-blue_cone.second.estimated_pose(0),2) \
                    + std::pow(yellow_cone.second.estimated_pose(1)-blue_cone.second.estimated_pose(1),2) )};
                if (dist < track_width) { track_width = dist; }
            }
        }
        map_ready_ = true;
        if (track_width > 99.0) return;
        slam_object_.setAccelWidth(track_width);
    }
}


void LifecycleSlamHandler::loadParameters() {
    declare_parameter<bool>("mapping_mode", true);
    declare_parameter<std::string>("track_map", "");
    declare_parameter<bool>("dynamic_accel_map", false);

    declare_parameter<int>("velocity_estimation_frequency", 40);
    declare_parameter<int>("perception_frequency", 10);

    declare_parameter<double>("perception_range", 14.0);
    declare_parameter<int>("optimization_interval", 20);

    declare_parameter<double>("association_threshold", 1.9);
    declare_parameter<int>("min_observations", 3);

    declare_parameter<double>("relinearize_threshold", 0.1);
    declare_parameter<int>("relinearize_skip", 1);

    declare_parameter<double>("odometry_covariance_weight", 10.0);
    declare_parameter<double>("perception_covariance_weight", 0.01);

    // Starting position variables
    declare_parameter<std::vector<double>>("starting_position", { -7.5, 0.0, 0.0 });
    declare_parameter<std::vector<double>>("starting_position_covariance", { 0.5, 0.1, 0.1 });

    declare_parameter<std::vector<double>>("left_orange", { 6.0, -1.5 });
    declare_parameter<std::vector<double>>("right_orange", { 6.0, 1.5 });
    declare_parameter<int>("lap_counter_cooldown", 10);

    share_dir_ = ament_index_cpp::get_package_share_directory("slam");

    declare_parameter<bool>("logger", true);
}
} // namespace ns_slam


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto options{ rclcpp::ExecutorOptions() };
    rclcpp::executors::MultiThreadedExecutor executor{ options, 4 };
    auto slam_node = std::make_shared<ns_slam::LifecycleSlamHandler>();   
    executor.add_node(slam_node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}