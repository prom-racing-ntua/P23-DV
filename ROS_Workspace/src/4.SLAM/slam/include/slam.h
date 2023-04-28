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
class UnaryFactor: public gtsam::NoiseModelFactor1<gtsam::Pose2> {
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


class GraphSLAM {
private:
	rclcpp::Node* node_handler_;

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
	GraphSLAM(rclcpp::Node* nh);

	// Destructor
	~GraphSLAM();

	// Initialize SLAM with ROS parameters
	void init();

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
}  // namespace ns_slam

#endif // SLAM_H