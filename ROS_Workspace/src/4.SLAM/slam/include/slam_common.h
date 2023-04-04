#ifndef SLAM_COMMON_H
#define SLAM_COMMON_H

#include <vector>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Symbol.h>


namespace ns_slam
{
// Enum of different cone colors
enum ConeColor {
    Yellow,
    Blue,
    SmallOrange,
    LargeOrange,
    ConeColorSize,
};


// A structure containing all the landmark info for the GraphSLAM HashMap of cone observations.
struct LandmarkInfo {
    // A way to now if the landmark is actually observed and not a ghost cone.
    // For now we set it to true if a cone is seen for a second time.
    bool is_verified;
    // Estimated pose of landmark in world
    gtsam::Vector2 estimated_pose;

    // Previous Observations
    // Symbol of the car's pose when the landmark was first observed
    std::vector<gtsam::Symbol> car_pose_symbols;
    // Measured transform from the car (at observation time) to landmark (range, theta)
    std::vector<double> range_vector;
    std::vector<double> theta_vector; //in radians
    // First observation variance matrix (range, theta)
    std::vector<gtsam::Matrix2> variance_vector;

    int score;
    double observation_ratio;
};


struct PerceptionMeasurement {
    ConeColor color;
    double range;
    double theta; //in radians
    gtsam::Matrix2 observation_noise;
};


struct OdometryMeasurement {
    unsigned long global_index;
    double velocity_x;
    double velocity_y;
    double yaw_rate;
    gtsam::Matrix3 measurement_noise;
};


// Contains the structure of the perception data file
struct PerceptionMsgStructure {
    unsigned long index;
    std::vector<ConeColor> color;
    std::vector<double> range;
    std::vector<double> theta;
};


// Contains the structure of the odometry data file
struct OdometryMsgStructure {
    unsigned long index;
    double velocity_x;
    double velocity_y;
    double yaw_rate;
    gtsam::Matrix3 covariance_matrix;
};
} // namespace ns_slam

#endif // SLAM_COMMON_H