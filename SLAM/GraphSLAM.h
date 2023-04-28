#ifndef GRAPHSLAM_H
#define GRAPHSLAM_H

#include <iostream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <fstream>


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

namespace slam {

// Factor with only one moving end
class UnaryFactor: public gtsam::NoiseModelFactor1<gtsam::Pose2> {
  private:
    double m_range, m_theta; ///< Range and Theta measurements
    double cone_x, cone_y;

  public:
    UnaryFactor(gtsam::Key j, double range, double theta, double x, double y, const gtsam::SharedNoiseModel& model);
    gtsam::Vector evaluateError(const gtsam::Pose2& q, boost::optional<gtsam::Matrix&> H) const override;
};

class GraphSLAM
{
  public:
    // Landmark Info for HashMap
    struct LandmarkInfo {
      // Seen second time?
      bool verified;

      // Symbol of the landmark
      gtsam::Symbol land_sym;
      // Cone color
      int color;
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

    struct PerceptionMeasurement {
      int color; // or enum
      double range;
      double theta; //in radians
      gtsam::Matrix2 land_obs_noise_;
    };

    // Constructor
    GraphSLAM(double isam2_relinearize_thresh, double isam2_relinearize_skip, double dist_threshold, double dt);
    // Destructor
    ~GraphSLAM();

    // Returns the estimated robot pose
    gtsam::Matrix13 get_est_robot_pose();

    // Returns the estimated map
    std::vector<gtsam::Matrix13> get_est_map();

    // Adds an odometry measurement to iSAM2
    void add_odom_measurement(double odom_Ux, double odom_Uy, double odom_omega, gtsam::Matrix3 odom_noise_);

    // Adds/stores a landmark measurement to iSAM2 in localization mode
    void add_landmark_measurements_loc(std::vector<PerceptionMeasurement> land_rel);

    // Adds/stores a landmark measurement to iSAM2 in slam mode
    void add_landmark_measurements_slam(std::vector<PerceptionMeasurement> land_rel);

    // Optimizes the factor graph
    void optimize_factor_graph();

    // Impose changes caused by optimization 
    void impose_optimization();

    // REad Measurements from txt file
    void read_measurements();

    // Adds landmarks to a txt file so a python xcript can make a visual
    void Visualize();

  private:

    // Initialize the factor graph
    void initialize_factor_graph();

    // Initialize the noise models
    void initialize_noise_models();

    // Find nearest neighbor
    int findNN(PerceptionMeasurement meas);

    // Map of the landmark ids to LandmarkInfo
    std::unordered_map<int, GraphSLAM::LandmarkInfo> landmark_id_map_;

    // Symbol (e.g. "x4") of current robot pose
    gtsam::Symbol current_robot_sym_;
    gtsam::Symbol opt_robot_sym_;

    // Counters for the robot pose and landmarks
    int robot_pose_counter_;
    int landmark_obs_counter_;

    // Only relinearize variables whose linear delta magnitude is greater than this threshold (default: 0.1).
    double isam2_relinearize_thresh_;
    // Only relinearize any variables every relinearizeSkip calls to ISAM2::update (default: 10)
    int isam2_relinearize_skip_;

    // isam2 solver
    gtsam::ISAM2* isam2_;
    // gtsam::LevenbergMarquardtOptimizer* lmo;

    // The iSAM2 estimated state
    gtsam::Values est_state_;
    // The estimated pose of the robot
    gtsam::Pose2 est_robot_pose_;
    gtsam::Pose2 pre_opt_pose;

    // Create a Factor Graph and Values to hold the new data
    gtsam::NonlinearFactorGraph* factor_graph_;
    gtsam::Values init_est_;

    // Distance above which it is considered to be a different cone
    double dist_threshold;

    // Time quantum
    double dt;

};

}  // namespace slam

#endif // GRAPHSLAM_H