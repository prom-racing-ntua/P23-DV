#include <iostream>
#include <stdexcept>
#include "slam_from_file.h"


namespace ns_slam
{
SlamFromFile::SlamFromFile() : Node("slam_from_file_node"), slam_object_(this) {
	global_index_ = 0;

	perception_eof_ = false;
	odometry_eof_ = false;

	accel_cone_count_ = 0;
    num_observations_ = 0;
    accel_map_.clear();

	loadParameters();

	readNextOdometry();
	readNextPerception();

	// Initialize slam
	slam_object_.setDeltaTime(1.0 / sampling_rate_);
	slam_object_.init();
	RCLCPP_WARN_STREAM(get_logger(), "Created SLAM object" << '\n');

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

	// Set ROS objects
	map_publisher_ = create_publisher<custom_msgs::msg::LocalMapMsg>("local_map", 10);
	pose_publisher_ = create_publisher<custom_msgs::msg::PoseMsg>("pose", 10);

	global_timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / sampling_rate_)), std::bind(&SlamFromFile::run_slam, this));
}

SlamFromFile::~SlamFromFile() {
	// Closes the files at the end of execution
	perception_file_.close();
	odometry_file_.close();
}


void SlamFromFile::run_slam() {
	std::vector<ns_slam::PerceptionMeasurement> all_observed_landmarks;
	// RCLCPP_INFO_STREAM(get_logger(), "Global Index is " << global_index_);
	rclcpp::Time starting_time{ this->now() };

	while (odometry_.index == global_index_ && !odometry_eof_)
	{
		OdometryMeasurement odometry_measurement;

		odometry_measurement.global_index = global_index_;
		odometry_measurement.velocity_x = odometry_.velocity_x;
		odometry_measurement.velocity_y = odometry_.velocity_y;
		// odometry_measurement.velocity_y = 0.0;
		odometry_measurement.yaw_rate = odometry_.yaw_rate;
		odometry_measurement.measurement_noise = odometry_weight_ * odometry_.covariance_matrix;

		slam_object_.addOdometryMeasurement(odometry_measurement);

		odometry_eof_ = readNextOdometry();
	}


	// Publish pose message
	custom_msgs::msg::PoseMsg pose_msg{};
	gtsam::Vector3 current_pose{ slam_object_.getEstimatedCarPose() };
	pose_msg.position.x = current_pose[0];
	pose_msg.position.y = current_pose[1];
	pose_msg.theta = current_pose[2];
	pose_msg.velocity_state.global_index = global_index_;
	pose_msg.velocity_state.velocity_x = odometry_.velocity_x;
	pose_msg.velocity_state.velocity_y = odometry_.velocity_y;
	pose_msg.velocity_state.yaw_rate = odometry_.yaw_rate;
	for (int i{ 0 }; i < 9;i++) { pose_msg.velocity_state.variance_matrix[i] = odometry_.covariance_matrix(i / 3, i % 3); }

	pose_publisher_->publish(pose_msg);

	rclcpp::Duration total_time{ this->now() - starting_time };
	RCLCPP_INFO_STREAM(get_logger(), "\n-- Odometry --\nTime of execution " << total_time.nanoseconds() / 1000000.0 << " ms.");
	starting_time = this->now();

	while (perception_.index == global_index_ && !perception_eof_)
	{
		int observation_size{ static_cast<int>(perception_.color.size()) };
		PerceptionMeasurement landmark;
		gtsam::Matrix2 obs_noise;

		// RCLCPP_INFO_STREAM(get_logger(), "Got Perception -> " << observation_size << '\n');
		for (int i{ 0 }; i < observation_size; i++)
		{
			landmark.color = perception_.color[i];
			landmark.range = perception_.range[i];
			landmark.theta = perception_.theta[i];
			// Setting observation noise depending on type of cone
			if (landmark.color == ConeColor::LargeOrange)
			{
				obs_noise << 0.01, 0,
					0, 3 * perception_weight_ * landmark.range / 10;
			}
			else
			{
				obs_noise << 0.001, 0,
					0, perception_weight_* landmark.range / 10;
			}
			landmark.observation_noise = obs_noise;

			// Only accept cones that are within the specified range
			if (landmark.range <= 6.0 or (landmark.theta <= 0.4 and landmark.range <= perception_range_))
			{
				// RCLCPP_INFO_STREAM(get_logger(), "Adding cone at range " << landmark.range << " m and angle " << landmark.theta << " rad.\n");
				all_observed_landmarks.push_back(landmark);
			}
		}
		perception_eof_ = readNextPerception();
	}

	if (!all_observed_landmarks.empty())
	{
		int perception_count{ all_observed_landmarks.size() };
		if (!map_ready_) {
            addAccelObservations(all_observed_landmarks);
            return;
        }
		
		if (is_mapping_) slam_object_.addLandmarkMeasurementSLAM(global_index_, all_observed_landmarks);
		else slam_object_.addLandmarkMeasurementsLocalization(global_index_, all_observed_landmarks);
		all_observed_landmarks.clear();

		// Optimize with perception
		starting_time = this->now();

		gtsam::NonlinearFactorGraph opt_new_factors{ slam_object_.getNewFactors() };
		gtsam::Values opt_new_variable_values{ slam_object_.getNewValues() };
		gtsam::Vector3 pre_optimization_pose{ slam_object_.getEstimatedCarPose() };
		gtsam::Symbol optimization_pose_symbol{ slam_object_.getCurrentPoseSymbol() };
		slam_object_.resetTemporaryGraph();

		slam_object_.optimizeFactorGraph(opt_new_factors, opt_new_variable_values);
		slam_object_.imposeOptimization(optimization_pose_symbol, pre_optimization_pose);

		// Publish map
		std::vector<gtsam::Vector3> track{ slam_object_.getEstimatedMap() };
		custom_msgs::msg::LocalMapMsg map_msg{};
		custom_msgs::msg::ConeStruct cone_msg{};

		map_msg.cones_count_actual = perception_count;
		map_msg.pose.position.x = current_pose[0];
		map_msg.pose.position.y = current_pose[1];
		map_msg.pose.theta = current_pose[2];
		map_msg.pose.velocity_state.global_index = global_index_;
		map_msg.pose.velocity_state.velocity_x = odometry_.velocity_x;
		map_msg.pose.velocity_state.velocity_y = odometry_.velocity_y;
		map_msg.pose.velocity_state.yaw_rate = odometry_.yaw_rate;
		for (int i{ 0 }; i < 9;i++) { map_msg.pose.velocity_state.variance_matrix[i] = odometry_.covariance_matrix(i / 3, i % 3); }

		if (track.empty())
		{
			map_msg.cones_count_all = 0;
		}
		else
		{
			map_msg.cones_count_all = track.size();
			for (auto cone : track)
			{
				cone_msg.color = cone[0];
				cone_msg.coords.x = cone[1];
				cone_msg.coords.y = cone[2];
				map_msg.local_map.push_back(cone_msg);
			}
		}
		map_publisher_->publish(map_msg);

		total_time = this->now() - starting_time;
		RCLCPP_INFO_STREAM(get_logger(), "\n-- Optimization --\nTime of execution " << total_time.nanoseconds() / 1000000.0 << " ms.");
	}
	total_time = this->now() - starting_time;
	RCLCPP_INFO_STREAM(get_logger(), "\n-- Perception --\nTime of execution " << total_time.nanoseconds() / 1000000.0 << " ms.");

	// if (global_index_ % optimization_interval_ == 0)
	// {
	// 	starting_time = this->now();

	// 	gtsam::NonlinearFactorGraph opt_new_factors{ slam_object_.getNewFactors() };
	// 	gtsam::Values opt_new_variable_values{ slam_object_.getNewValues() };
	// 	gtsam::Vector3 pre_optimization_pose{ slam_object_.getEstimatedCarPose() };
	// 	gtsam::Symbol optimization_pose_symbol{ slam_object_.getCurrentPoseSymbol() };
	// 	slam_object_.resetTemporaryGraph();

	// 	slam_object_.optimizeFactorGraph(opt_new_factors, opt_new_variable_values);
	// 	slam_object_.imposeOptimization(optimization_pose_symbol, pre_optimization_pose);

	// 	// Publish map
	// 	if (is_mapping_)
	// 	{
	// 		std::vector<gtsam::Vector3> track{ slam_object_.getEstimatedMap() };
	// 		custom_msgs::msg::LocalMapMsg map_msg{};
	// 		custom_msgs::msg::ConeStruct cone_msg{};

	// 		map_msg.pose.position.x = current_pose[0];
	// 		map_msg.pose.position.y = current_pose[1];
	// 		map_msg.pose.theta = current_pose[2];

	// 		if (track.empty())
	// 		{
	// 			map_msg.cone_count = 0;
	// 		}
	// 		else
	// 		{
	// 			map_msg.cone_count = track.size();
	// 			for (auto cone : track)
	// 			{
	// 				cone_msg.color = cone[0];
	// 				cone_msg.coords.x = cone[1];
	// 				cone_msg.coords.y = cone[2];
	// 				map_msg.local_map.push_back(cone_msg);
	// 			}
	// 		}
	// 		map_publisher_->publish(map_msg);
	// 	}

	// 	total_time = this->now() - starting_time;
	// 	RCLCPP_INFO_STREAM(get_logger(), "\n-- Optimization --\nTime of execution " << total_time.nanoseconds() / 1000000.0 << " ms.");
	// }

	global_index_++;

	if (perception_eof_ or odometry_eof_)
	{
		rclcpp::shutdown();
	}
}

void SlamFromFile::resetPerception() {
	perception_.index = 0;
	perception_.color.clear();
	perception_.range.clear();
	perception_.theta.clear();
}

void SlamFromFile::resetOdometry() {
	odometry_.index = 0;
	odometry_.velocity_x = 0.0;
	odometry_.velocity_y = 0.0;
	odometry_.yaw_rate = 0.0;
	odometry_.covariance_matrix.setZero();
}

bool SlamFromFile::readNextOdometry() {
	resetOdometry();
	if (odometry_file_.is_open())
	{
		odometry_file_ >> odometry_.index;
		odometry_file_ >> odometry_.velocity_x;
		odometry_file_ >> odometry_.velocity_y;
		odometry_file_ >> odometry_.yaw_rate;
		for (int i{ 0 }; i < 9; i++)
		{
			odometry_file_ >> odometry_.covariance_matrix(i / 3, i % 3);
		}
	}
	if (odometry_file_.eof()) return 1;
	return 0;
}

bool SlamFromFile::readNextPerception() {
	resetPerception();
	if (perception_file_.is_open())
	{
		std::string next_line;

		int index_temp;
		int color_temp;
		double range_temp;
		double theta_temp;

		// Read observation index
		std::getline(perception_file_, next_line);
		std::istringstream line_stream_i{next_line};
		line_stream_i >> index_temp;
		perception_.index = index_temp;
		// std::cout << perception_.index << '\n';
		next_line.clear();

		// Read observation color
		std::getline(perception_file_, next_line);
		std::istringstream line_stream_c{next_line};
		while (line_stream_c >> color_temp) perception_.color.push_back(static_cast<ConeColor>(color_temp));
		next_line.clear();

		// Read observation range
		std::getline(perception_file_, next_line);
		std::istringstream line_stream_r{next_line};
		while (line_stream_r >> range_temp) perception_.range.push_back(range_temp);
		next_line.clear();

		// Read observation theta
		std::getline(perception_file_, next_line);
		std::istringstream line_stream_t{next_line};
		while (line_stream_t >> theta_temp) perception_.theta.push_back(theta_temp);
		next_line.clear();
	}
	if (perception_file_.eof()) return 1;
	return 0;
}

void SlamFromFile::loadParameters() {
	is_mapping_ = declare_parameter<bool>("mapping_mode", false);
	declare_parameter<std::string>("track_map", "");
    declare_parameter<bool>("dynamic_accel_map", false);

	sampling_rate_ = declare_parameter<int>("sampling_rate", 40);
	perception_range_ = declare_parameter<double>("perception_range", 10.0);
	optimization_interval_ = declare_parameter<int>("optimization_interval", 50);

	declare_parameter<double>("association_threshold", 1.0);
	declare_parameter<int>("min_observations", 3);

	declare_parameter<double>("relinearize_threshold", 0.1);
	declare_parameter<int>("relinearize_skip", 10);

	odometry_weight_ = declare_parameter<double>("odometry_covariance_weight", 1.0);
	perception_weight_ = declare_parameter<double>("perception_covariance_weight", 0.02);

	// Starting position variables
	declare_parameter<std::vector<double>>("starting_position", { 0.0, 0.0, 0.0 });
	declare_parameter<std::vector<double>>("starting_position_covariance", { 0.1, 0.1, 0.1 });

	declare_parameter<std::vector<double>>("left_orange", { 6.0, -3.0 });
	declare_parameter<std::vector<double>>("right_orange", { 6.0, 3.0 });

	// Set and open files
	share_dir_ = ament_index_cpp::get_package_share_directory("slam");

	std::string odometry_file_path{ declare_parameter<std::string>("odometry_log", "") };
	std::string perception_file_path{ declare_parameter<std::string>("perception_log", "") };

	if (odometry_file_path.empty() or perception_file_path.empty())
	{
		throw std::runtime_error("SlamFromFile -> Log files were not specified");
	}

	odometry_file_.open(share_dir_ + odometry_file_path, std::fstream::in);
	perception_file_.open(share_dir_ + perception_file_path, std::fstream::in);
}


void SlamFromFile::addAccelObservations(const std::vector<PerceptionMeasurement>& observations) {
    num_observations_++;
	RCLCPP_INFO_STREAM(get_logger(), "Number of Observations " << num_observations_);
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

            std::accumulate(matched_cone.range_vector.begin(), matched_cone.range_vector.end(), 0.0) / matched_cone.range_vector.size();
            std::accumulate(matched_cone.theta_vector.begin(), matched_cone.theta_vector.end(), 0.0) / matched_cone.theta_vector.size();
        }
    }

    if (num_observations_ >= 40)
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
		RCLCPP_WARN_STREAM(get_logger(), "Setting accel width to " << track_width);
        slam_object_.setAccelWidth(track_width);
        map_ready_ = true;
    }
}
} // namespace ns_slam


int main(int argc, char** argv) {
	// Clear std::cout buffer
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	ns_slam::SlamFromFile node{};
	rclcpp::spin(node.get_node_base_interface());
	rclcpp::shutdown();

	return 0;
}