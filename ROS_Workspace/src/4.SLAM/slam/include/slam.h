#ifndef SLAM_H
#define SLAM_H

#include <iostream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <fstream>
#include <pthread.h>
#include <memory>

#include <rclcpp/rclcpp.hpp>

// GTSAM Includes
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols (Xi for poses and Li for landmarks aka cones)
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the SLAM problem incrementally
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// We will initialize the car at some location using a Prior factor.
// The Between Factor will be used for connecting car poses with each other.
// For Cone Observations we use the Bearing Range Factor for SLAM mode,
// while for localization we will use a custom Unary Factor created bellow.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include "slam_common.h"

namespace ns_slam
{
/* A custom graph factor with only one moving end. It is used for the landmark connections in localization mode,
 * and it actually corresponds to the car's pose and not the cone's.
 * Inherits from the NoiseModelFactor1 class of gtsam.
 * For more info see here: https://gtsam.org/doxygen/a04452.html#a85c649f81b216f5b5d7e321febc7762c
 */
class UnaryFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2> {
	// Measured cone range and angle with respect to the car derived from perception observations
	double measured_range;
	double measured_angle;

	// Actual coordinates of the observed cone derived from the track map, after the association of the observed cone
	double actual_cone_x;
	double actual_cone_y;

public:
	using gtsam::NoiseModelFactor1<gtsam::Pose2>::evaluateError;
	// Constructor of custom unary factor 
	UnaryFactor(gtsam::Key j, double range, double theta, double x, double y, const gtsam::SharedNoiseModel& model);

	~UnaryFactor() override {}

	gtsam::Vector evaluateError(const gtsam::Pose2& car_pose, gtsam::OptionalMatrixType H) const override;

	gtsam::NonlinearFactor::shared_ptr clone() const override;
};


template <class T>
class GraphSLAM {
private:
	T* node_handler_;

	// Map of the landmark ids to LandmarkInfo. Contains all the cones observed so far.
	std::unordered_map<int, LandmarkInfo> landmark_id_map_;

	// Symbol (e.g. "X4") of current car pose
	gtsam::Symbol current_car_pose_symbol_;

	// Used to compute the time interval between odometry measurements
	unsigned long previous_global_index_;
	int landmark_counter_;

	// iSAM Variables

	// iSAM2 solver
	gtsam::ISAM2* isam_;
	// Only relinearize variables whose linear delta magnitude is greater than this threshold (default: 0.1).
	double isam2_relinearize_thresh_;
	// Only relinearize any variables every relinearizeSkip calls to ISAM2::update (default: 10)
	int isam2_relinearize_skip_;
	// The iSAM2 estimated state
	gtsam::Values estimated_global_state_;
	// The estimated pose of the car
	gtsam::Pose2 estimated_car_pose_;

	// Create a Factor Graph and Values to hold the new factors and variables to be inserted in iSAM.
	// These graph variables are temporary and are reset after every optimization of the global graph. 
	gtsam::NonlinearFactorGraph new_factors_;
	gtsam::Values new_variable_values_;

	// Distance above which it is considered to be a different cone
	double association_distance_threshold_;
	// Minimum amount of landmark observations before it becomes verified
	int min_observations_;
	// Time quantum
	double delta_time_;
	// Total confirmed cone count
	int cone_count_;

	// Start/Finish Line
	gtsam::Vector2 left_orange_;
	gtsam::Vector2 right_orange_;

	// Initialize the factor graph
	void initializeFactorGraph();

	// Find nearest neighbor of an observed landmark to be associated with
	int findNearestNeighbor(PerceptionMeasurement& observed_landmark, gtsam::Vector2& global_position);

	// Lap counter methods
	// Source: https://bryceboe.com/2006/10/23/line-segment-intersection-algorithm/
	bool intersectFinishLine(gtsam::Vector2 old_pose, gtsam::Vector2 new_pose) {
		return (ccw(old_pose, left_orange_, right_orange_) != ccw(new_pose, left_orange_, right_orange_)) && (ccw(old_pose, new_pose, left_orange_) != ccw(old_pose, new_pose, right_orange_));
	}

	bool ccw(gtsam::Vector2 A, gtsam::Vector2 B, gtsam::Vector2 C) {
		return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0]);
	}

	gtsam::KeySet checkObservationRatio();

public:
	// Constructor
	GraphSLAM(T* nh);

	// Destructor
	~GraphSLAM();

	// Initialize SLAM with ROS parameters
	void init();

	void reset();

	// Adds an odometry measurement to iSAM2
	bool addOdometryMeasurement(OdometryMeasurement& odometry);

	// Adds/stores a landmark measurement to iSAM2 in slam mode
	void addLandmarkMeasurementSLAM(const unsigned long global_index, std::vector<PerceptionMeasurement>& landmarks);

	// Adds/stores a landmark measurement to iSAM2 in localization mode
	void addLandmarkMeasurementsLocalization(const unsigned long global_index, std::vector<PerceptionMeasurement>& landmarks);

	// Loads the track map when in localization mode
	void loadMap(std::string& map_file_path);

	// Optimizes the factor graph
	void optimizeFactorGraph(gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_variable_values);

	// Impose changes caused by optimization 
	void imposeOptimization(gtsam::Symbol& optimization_symbol, gtsam::Vector3& pre_optimization_pose);

	void resetTemporaryGraph();

	// Setters
	void setDeltaTime(const double dt) { delta_time_ = dt; }

	// Getters
	gtsam::NonlinearFactorGraph getNewFactors() { return new_factors_.clone(); }
	gtsam::Values getNewValues() { return new_variable_values_; }
	gtsam::Symbol getCurrentPoseSymbol() { return current_car_pose_symbol_; }
	gtsam::Vector3 getEstimatedCarPose();
	std::vector<gtsam::Vector3> getEstimatedMap();
	int getConeCount() { return cone_count_; }
};


template <class T>
GraphSLAM<T>::GraphSLAM(T* nh) : node_handler_(nh) {
	previous_global_index_ = 0;
	landmark_counter_ = 0;
	cone_count_ = 0;
	landmark_id_map_.clear();
}

template <class T>
GraphSLAM<T>::~GraphSLAM() {
	// Free up heap memory occupied by iSAM
	delete isam_;
}

template <class T>
void GraphSLAM<T>::init() {
	association_distance_threshold_ = node_handler_->get_parameter("association_threshold").as_double();
	min_observations_ = node_handler_->get_parameter("min_observations").as_int();

	// Initialize iSAM2 with specified parameters
	gtsam::ISAM2Params parameters;
	parameters.relinearizeThreshold = node_handler_->get_parameter("relinearize_threshold").as_double();
	parameters.relinearizeSkip = node_handler_->get_parameter("relinearize_skip").as_int();
	isam_ = new gtsam::ISAM2(parameters);

	auto orange1 = node_handler_->get_parameter("left_orange").as_double_array();
	left_orange_ << orange1[0], orange1[1];
	auto orange2 = node_handler_->get_parameter("right_orange").as_double_array();
	right_orange_ << orange2[0], orange2[1];

	// Initializes the factor graph
	initializeFactorGraph();
	RCLCPP_WARN_STREAM(node_handler_->get_logger(), "Created SLAM object" << '\n');
}

template <class T>
void GraphSLAM<T>::reset() {
	delete isam_;
}

// Initializes the factor graph
template <class T>
void GraphSLAM<T>::initializeFactorGraph() {
	current_car_pose_symbol_ = gtsam::Symbol('I', 0);

	// Set the initial car_pose and create a Prior Factor for it
	auto initial_pose{ node_handler_->get_parameter("starting_position").as_double_array() };
	estimated_car_pose_ = gtsam::Pose2(initial_pose[0], initial_pose[1], initial_pose[2]);

	// Create the Gaussian noise model as a diagonal matrix
	auto initial_variances{ node_handler_->get_parameter("starting_position_covariance").as_double_array() };
	gtsam::noiseModel::Diagonal::shared_ptr noise_model{
		gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(initial_variances[0], initial_variances[1], initial_variances[2]))
	};

	// Make sure new factors and variables are empty and add the Prior Pose to the temporary graph variables
	new_factors_ = gtsam::NonlinearFactorGraph{};
	new_variable_values_ = gtsam::Values{};

	new_factors_.add(gtsam::PriorFactor<gtsam::Pose2>(current_car_pose_symbol_, estimated_car_pose_, noise_model));
	new_variable_values_.insert(current_car_pose_symbol_, estimated_car_pose_);
}

// Adds an odometry measurement to iSAM2 and computes the current estimated state
template <class T>
bool GraphSLAM<T>::addOdometryMeasurement(OdometryMeasurement& odometry) {
	// RCLCPP_INFO_STREAM(node_handler_->get_logger(), odometry.global_index);

	// Create the symbol of the new pose variable that is to be created
	gtsam::Symbol next_car_pose_symbol{ 'X', odometry.global_index };

	// Calculate time interval from last received odometry measurement
	double time_interval{ (odometry.global_index - previous_global_index_) * delta_time_ };
	if (previous_global_index_ == 0) time_interval = delta_time_;
	previous_global_index_ = odometry.global_index;

	// Compute the distance (Factor length) between the current and the previous poses
	gtsam::Pose2 pose_distance{ odometry.velocity_x* time_interval, odometry.velocity_y* time_interval, odometry.yaw_rate* time_interval };

	// Create the pose distance covariance matrix and the noise model for the Between Factor
	gtsam::Matrix3 distance_covariance;
	distance_covariance << time_interval, 0, 0,
		0, time_interval, 0,
		0, 0, time_interval;

	gtsam::noiseModel::Gaussian::shared_ptr noise_model{
		gtsam::noiseModel::Gaussian::Covariance(distance_covariance* odometry.measurement_noise* distance_covariance.transpose())
	};

	// Add the new Between Factor to the Factors temporary variable and update pose symbol 
	new_factors_.add(gtsam::BetweenFactor<gtsam::Pose2>(current_car_pose_symbol_, next_car_pose_symbol, pose_distance, noise_model));

	// Calculate an initial estimate of the new pose variable that was just created
	double new_x{
	  estimated_car_pose_.x() + (odometry.velocity_x * std::cos(estimated_car_pose_.theta()) - odometry.velocity_y * std::sin(estimated_car_pose_.theta())) * time_interval
	};
	double new_y{
	  estimated_car_pose_.y() + (odometry.velocity_x * std::sin(estimated_car_pose_.theta()) + odometry.velocity_y * std::cos(estimated_car_pose_.theta())) * time_interval
	};
	double new_theta{ estimated_car_pose_.theta() + odometry.yaw_rate * time_interval };
	gtsam::Pose2 new_pose_estimate{ new_x, new_y, new_theta };

	// Add that estimate to the Values temporary variable and update the car's current pose
	new_variable_values_.insert(next_car_pose_symbol, new_pose_estimate);

	gtsam::Vector2 previous_pose;
	gtsam::Vector2 new_pose;
	previous_pose << estimated_car_pose_.x(), estimated_car_pose_.y();
	new_pose << new_x, new_y;

	bool completed_lap{ intersectFinishLine(previous_pose, new_pose) };
	current_car_pose_symbol_ = next_car_pose_symbol;
	estimated_car_pose_ = new_pose_estimate;

	// RCLCPP_INFO_STREAM(node_handler_->get_logger(), "Car Position: X = " << estimated_car_pose_.x() << ", Y = " << estimated_car_pose_.y());
	return completed_lap;
}

// Adds landmark measurements in SLAM mode
template <class T>
void GraphSLAM<T>::addLandmarkMeasurementSLAM(const unsigned long global_index, std::vector<PerceptionMeasurement>& landmarks) {
	RCLCPP_INFO_STREAM(node_handler_->get_logger(),"Perception callback at index: " << global_index);
	gtsam::Symbol observation_pose_symbol{ 'X', global_index };
	gtsam::Pose2 observation_pose;

	if (isam_->valueExists(observation_pose_symbol)) \
		observation_pose = isam_->calculateEstimate(observation_pose_symbol).cast<gtsam::Pose2>();
	else if (new_variable_values_.exists(observation_pose_symbol)) \
		observation_pose = new_variable_values_.at(observation_pose_symbol).cast<gtsam::Pose2>();
	else
	{
		RCLCPP_ERROR_STREAM(node_handler_->get_logger(), "addLandmark() -> Perception index does not match any poses. global_index = " << global_index);
		return;
	}

	for (PerceptionMeasurement& cone : landmarks)
	{
		// RCLCPP_INFO_STREAM(node_handler_->get_logger(), cone.observation_noise);

		// Estimate position of newly observed cone in global map (x,y)
		gtsam::Vector2 observed_position;
		observed_position << observation_pose.x() + cone.range * std::cos(cone.theta + observation_pose.theta()),
			observation_pose.y() + cone.range * std::sin(cone.theta + observation_pose.theta());

		int best_match_id{ findNearestNeighbor(cone, observed_position) };

		/* Case where the landmark has not been observed before.
		 * Here we don't put the observed landmark in the graph in case it is a false positive of
		 * the perception pipeline (i.e. a ghost cone). We just add it to the landmark_id_map_ variable
		 * and wait to observe the same cone a second time. Then we will add this first measurement in
		 * the graph along with its initial estimate and then update that estimate according to the second
		 * measurement and every other measurement thereafter.
		 */
		if (best_match_id == -1)
		{
			// Creating the new landmark symbol and putting it in the dictionary
			gtsam::Symbol new_landmark_symbol{'L', landmark_counter_};

			// Create the landmark entry
			LandmarkInfo new_landmark;

			new_landmark.is_verified = false;
			new_landmark.symbol = new_landmark_symbol;
			new_landmark.color = cone.color;
			new_landmark.estimated_pose = observed_position;
			new_landmark.times_observed = 0;

			// new_landmark.car_pose_symbols.push_back(observation_pose_symbol);
			// new_landmark.range_vector.push_back(cone.range);
			// new_landmark.theta_vector.push_back(cone.theta);
			// new_landmark.variance_vector.push_back(cone.observation_noise);

			// if (cone.range > 12) new_landmark.score = 2;
			// else if (cone.range > 8) new_landmark.score = 4;
			// else if (cone.range > 2) new_landmark.score = 10;
			// else new_landmark.score = 5;

			landmark_id_map_[landmark_counter_] = new_landmark;
			best_match_id = landmark_counter_++;

			new_variable_values_.insert(new_landmark.symbol, gtsam::Point2(new_landmark.estimated_pose[0], new_landmark.estimated_pose[1]));
		}
		// Case where the landmark has been observed before
		LandmarkInfo* best_match{ &landmark_id_map_.at(best_match_id) };

		// Check how many times the landmark has been observed before
		best_match->times_observed++;
		if (best_match->times_observed >= min_observations_) { best_match->is_verified = true; }

		// Construct the current landmark observation Factor and add it to the temporary variable
		gtsam::noiseModel::Gaussian::shared_ptr noise_model {
			gtsam::noiseModel::Gaussian::Covariance(cone.observation_noise)
		};

		new_factors_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
			observation_pose_symbol, best_match->symbol, gtsam::Rot2(cone.theta), cone.range, noise_model)
		);

		// If the landmark was observed only once, put the previous landmark measurement into the factor graph and add the initial estimate 
		// of the landmark's position (x,y)
		// if (!best_match->is_verified)
		// {
		// 	best_match->car_pose_symbols.push_back(observation_pose_symbol);
		// 	best_match->range_vector.push_back(cone.range);
		// 	best_match->theta_vector.push_back(cone.theta);
		// 	best_match->variance_vector.push_back(cone.observation_noise);
		// 	best_match->estimated_pose[0] = (best_match->estimated_pose[0] + observed_position[0]) / 2.0;
		// 	best_match->estimated_pose[1] = (best_match->estimated_pose[1] + observed_position[1]) / 2.0;

		// 	if (cone.range > 12) best_match->score += 2;
		// 	else if (cone.range > 8) best_match->score += 4;
		// 	else if (cone.range > 2) best_match->score += 10;
		// 	else best_match->score += 5;

		// 	if (best_match->score >= 15)
		// 	{
		// 		best_match->is_verified = true;
		// 		cone_count_++;

		// 		// Add the previous landmark measurements to the factor graph from the car_pose_symbol with the first observation values
		// 		for (int i{ 0 }; i < best_match->car_pose_symbols.size(); i++)
		// 		{
		// 			gtsam::noiseModel::Gaussian::shared_ptr noise_model {
		// 				gtsam::noiseModel::Gaussian::Covariance(best_match->variance_vector[i])
		// 			};

		// 			new_factors_.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
		// 				best_match->car_pose_symbols[i], best_match->symbol, gtsam::Rot2(best_match->theta_vector[i]), best_match->range_vector[i], noise_model)
		// 			);
		// 		}
		// 		// Add the initial estimate of the newly added landmark pose
		// 		new_variable_values_.insert(best_match->symbol, gtsam::Point2(best_match->estimated_pose[0], best_match->estimated_pose[1]));
		// 	}
		// }
		// else
		// {
		// 	best_match->car_pose_symbols.push_back(observation_pose_symbol);
		// 	best_match->range_vector.push_back(cone.range);
		// 	best_match->theta_vector.push_back(cone.theta);
		// 	best_match->variance_vector.push_back(cone.observation_noise);
		// }
	}
}


// Adds landmark measurements in LOCALIZATION mode
template <class T>
void GraphSLAM<T>::addLandmarkMeasurementsLocalization(const unsigned long global_index, std::vector<PerceptionMeasurement>& landmarks) {
	RCLCPP_INFO_STREAM(node_handler_->get_logger(), global_index);
	gtsam::Symbol observation_pose_symbol{ 'X', global_index };
	gtsam::Pose2 observation_pose;

	if (isam_->valueExists(observation_pose_symbol)) \
		observation_pose = isam_->calculateEstimate(observation_pose_symbol).cast<gtsam::Pose2>();
	else if (new_variable_values_.exists(observation_pose_symbol)) \
		observation_pose = new_variable_values_.at(observation_pose_symbol).cast<gtsam::Pose2>();
	else
	{
		RCLCPP_ERROR_STREAM(node_handler_->get_logger(), "addLandmark() -> Perception index does not match any poses. global_index = " << global_index);
		return;
	}

	// Find best match for every cone observation arriving from perception
	for (PerceptionMeasurement& cone : landmarks)
	{
		gtsam::Vector2 observed_position;
		observed_position << observation_pose.x() + cone.range * std::cos(cone.theta + observation_pose.theta()),
			observation_pose.y() + cone.range * std::sin(cone.theta + observation_pose.theta());
		int best_match_id = findNearestNeighbor(cone, observed_position);

		if (best_match_id != -1)
		{
			LandmarkInfo* best_match{ &landmark_id_map_.at(best_match_id) };

			gtsam::noiseModel::Gaussian::shared_ptr noise_model{
				gtsam::noiseModel::Gaussian::Covariance(cone.observation_noise)
			};

			new_factors_.add(UnaryFactor(
				observation_pose_symbol, cone.range, cone.theta, best_match->estimated_pose[0], best_match->estimated_pose[1], noise_model
			));
		}
	}
}

template <class T>
void GraphSLAM<T>::loadMap(std::string& map_file_path) {
	std::fstream map_file{ map_file_path };
	while (!map_file.eof())
	{
		gtsam::Symbol cone_symbol{ 'L', landmark_counter_ };
		LandmarkInfo cone{};
		cone.is_verified = true;
		cone.symbol = cone_symbol;
		double color_temp{};
		map_file >> color_temp;
		cone.color = static_cast<ConeColor>(color_temp);
		map_file >> cone.estimated_pose[0];
		map_file >> cone.estimated_pose[1];

		landmark_id_map_[landmark_counter_++] = cone;
	}
	cone_count_ = landmark_id_map_.size();
	map_file.close();
}

template <class T>
int GraphSLAM<T>::findNearestNeighbor(PerceptionMeasurement& observed_landmark, gtsam::Vector2& global_position) {
	// Check already known cones for closest observed cone (Euler distance), best_match is the index of the closest neighbor.
	// If best_match == -1 then it is a phantom cone.
	int best_match = -1;
	double least_distance_square{ std::pow(association_distance_threshold_, 2) };

	if (observed_landmark.color == ConeColor::LargeOrange)
	{
		least_distance_square = std::pow(1.4 * association_distance_threshold_, 2);
	}

	// Iterate through all of the cones in the current map
	for (auto& it : landmark_id_map_)
	{
		LandmarkInfo& cone{ it.second };
		// Only check cones of the same color
		if (cone.color == observed_landmark.color)
		{
			// Calculate the distance between the observed cone and the mapped cone
			double current_distance_square = (global_position - cone.estimated_pose).transpose() * (global_position - cone.estimated_pose);
			// If their distance is less than the previous best, assign the current cone as the best match
			if (current_distance_square < least_distance_square)
			{
				least_distance_square = current_distance_square;
				best_match = it.first;
			}
		}
	}
	return best_match;
}

// Optimizes the factor graph
template <class T>
void GraphSLAM<T>::optimizeFactorGraph(gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_variable_values) {
	// Update the current estimated robot pose
	isam_->update(new_factors, new_variable_values);
	estimated_global_state_ = isam_->calculateEstimate();
}

// Used to be part of optimization, split for ROS purposes
template <class T>
void GraphSLAM<T>::imposeOptimization(gtsam::Symbol& optimization_symbol, gtsam::Vector3& pre_optimization_pose) {
	// std::function<bool(gtsam::Key)> isPose{ gtsam::Symbol::ChrTest('X') };
	// std::function<bool(gtsam::Key)> isLandmark{ gtsam::Symbol::ChrTest('L') };

	gtsam::Pose2 optimized_pose{ estimated_global_state_.at<gtsam::Pose2>(optimization_symbol) };

	// Just an idea...
	// double delta_x{ optimized_pose.x() - pre_optimization_pose(0) };
	// double delta_y{ optimized_pose.y() - pre_optimization_pose(1) };
	// double delta_theta{ optimized_pose.theta() - pre_optimization_pose(2) };
	// gtsam::Matrix3 transformation_matrix{ gtsam::Pose2(delta_x, delta_y, delta_theta).matrix() };

	// // Rotate and translate every value variable that was added during optimization
	// for (auto it : new_variable_values_)
	// {
	// 	if (isPose(it.key))
	// 	{
	// 		new_variable_values_.update(it.key, transformation_matrix * it.value.cast<gtsam::Pose2>().matrix());
	// 	}
	// 	else if (isLandmark(it.key))
	// 	{
	// 		new_variable_values_.update(it.key, transformation_matrix.block(0, 0, 2, 3) * it.value.cast<gtsam::Point2>());
	// 	}
	// }

	// Update the current estimated robot pose
	double new_x{
	  optimized_pose.x() + std::cos(optimized_pose.theta() - pre_optimization_pose(2)) * (estimated_car_pose_.x() - pre_optimization_pose(0)) - std::sin(optimized_pose.theta() - pre_optimization_pose(2)) * (estimated_car_pose_.y() - pre_optimization_pose(1))
	};

	double new_y{
	  optimized_pose.y() + std::sin(optimized_pose.theta() - pre_optimization_pose(2)) * (estimated_car_pose_.x() - pre_optimization_pose(0)) + std::cos(optimized_pose.theta() - pre_optimization_pose(2)) * (estimated_car_pose_.y() - pre_optimization_pose(1))
	};

	double new_theta{ optimized_pose.theta() + estimated_car_pose_.theta() - pre_optimization_pose(2) };
	estimated_car_pose_ = gtsam::Pose2(new_x, new_y, new_theta);

	// For every landmark that has been optimized update its estimated position
	for (auto& it : landmark_id_map_)
	{
		LandmarkInfo& cone{ it.second };
		if (estimated_global_state_.exists<gtsam::Point2>(cone.symbol))
		{
			cone.estimated_pose = estimated_global_state_.at<gtsam::Point2>(cone.symbol);
		}
	}
}

template <class T>
void GraphSLAM<T>::resetTemporaryGraph() {
	new_factors_.resize(0);
	new_variable_values_.clear();
	return;
}

// Returns the current estimate of the car's pose
template <class T>
gtsam::Vector3 GraphSLAM<T>::getEstimatedCarPose() {
	return gtsam::Vector3(estimated_car_pose_.x(), estimated_car_pose_.y(), estimated_car_pose_.theta());
}

// Returns the current estimate of the track map
template <class T>
std::vector<gtsam::Vector3> GraphSLAM<T>::getEstimatedMap() {
	std::vector<gtsam::Vector3> estimated_map;

	// Iterate through the HashMap of cones and add them to the return vector if the cone is verified
	for (auto& it : landmark_id_map_)
	{
		LandmarkInfo& cone{ it.second };
		if (cone.is_verified)
		{
			gtsam::Vector3 mapped_cone{ static_cast<double>(cone.color), cone.estimated_pose[0], cone.estimated_pose[1] };
			estimated_map.push_back(mapped_cone);
		}
	}
	return estimated_map;
}
}  // namespace ns_slam

#endif // SLAM_H