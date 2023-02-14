#ifndef SLAM_H
#define SLAM_H

#include <iostream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <fstream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

// GTSAM Stuff
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the structure-from-motion problem incrementally, so
// include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>

namespace slam_namespace {

// Landmark Info for HashMap
struct LandmarkInfo {
  // Seen second time?
  bool verified;

  // Symbol of the landmark
  gtsam::Symbol land_sym;
  // Estimated pose of landmark in world
  gtsam::Matrix12 est_pos;

  // Symbol of the robot pose at first observation
  gtsam::Symbol robot_pose_sym;
  // Measured transform from the robot (at obs time) to landmark (range, theta)
  double first_range;
  double first_theta; //in radians
  // First observation variance matrix (range, theta)
  gtsam::Matrix2 first_obs_var;
};

// Factor with only one moving end
class UnaryFactor: public gtsam::NoiseModelFactor1<gtsam::Pose2> {
  private:
    double m_range, m_theta; ///< Range and Theta measurements
    double cone_x, cone_y;

  public:
    UnaryFactor(gtsam::Key j, double range, double theta, double x, double y, const gtsam::SharedNoiseModel& model);
    gtsam::Vector evaluateError(const gtsam::Pose2& q, boost::optional<gtsam::Matrix&> H) const override;
};

class SLAM_handler;
using Handle = SLAM_handler*;

class GraphSLAM
{
  public:

    // The estimated pose of the robot
    gtsam::Pose2 est_robot_pose;

    // Create a Factor Graph and Values to hold the new data
    gtsam::NonlinearFactorGraph factor_graph;
    gtsam::Values init_est;

    // Constructor
    explicit GraphSLAM(Handle nh);
    // Destructor
    // ~GraphSLAM();

    // Initializes factor graph
    void initialize_factor_graph(gtsam::Symbol start_robot_sym);

    // Returns the estimated map
    std::vector<gtsam::Matrix13> get_est_map();

    // Adds an odometry measurement to iSAM2
    bool add_odom_measurement(double odom_Ux, double odom_Uy, double odom_omega, gtsam::Matrix3 odom_noise, gtsam::Symbol current_robot_sym, gtsam::Symbol next_robot_sym, int stride);

    // Adds/stores a landmark measurement to iSAM2 in localization mode
    // void add_landmark_measurements_loc(std::vector<PerceptionMeasurement> land_rel);

    // Adds/stores a landmark measurement to iSAM2 in slam mode
    void add_landmark_measurements_slam(std::vector<int> color_list, std::vector<float> range_list, std::vector<float> theta_list, gtsam::Symbol current_robot_sym, gtsam::Pose2 current_pose);

    // Optimizes the factor graph
    void optimize_factor_graph(gtsam::NonlinearFactorGraph factor_graph, gtsam::Values init_est);

    // Impose changes caused by optimization 
    void impose_optimization(gtsam::Symbol opt_robot_sym, gtsam::Pose2 pre_opt_pose);


  private:
    // pointer to handler class
    Handle node_handler_; 

    // Find nearest neighbor
    int findNN(int color, float range, float theta, gtsam::Pose2 current_pose);

    // Determines whether two segments intersect
    bool intersect(gtsam::Matrix12 old_pose_matrix, gtsam::Matrix12 new_pose_matrix);
    bool ccw(gtsam::Matrix12 A, gtsam::Matrix12 B, gtsam::Matrix12 C);

    // Map of the landmark ids to LandmarkInfo
    std::unordered_map<int, LandmarkInfo> landmark_id_map[4]; 

    // Counters for the robot pose and landmarks
    int land_obs_counter[4];

    // isam2 solver
    gtsam::ISAM2 isam2;

    // The iSAM2 estimated state
    gtsam::Values est_state;

    // Distance above which it is considered to be a different cone
    double small_dist_threshold;
    double large_dist_threshold;

    // Time quantum
    double dt;

    // Mapping int color to corresponding character
    char color_char[4];

    // Approximate positions of large orange cones
    gtsam::Matrix12 left_orange, right_orange;

    // Weight of previous observations of same landmark
    double weight;

    // Maximum acceptable observation range
    double range_limit;

    // Prior pose variance matrix
    gtsam::Matrix3 prior_pose_noise;

};

}  // namespace slam_namespace

#endif // SLAM_H
