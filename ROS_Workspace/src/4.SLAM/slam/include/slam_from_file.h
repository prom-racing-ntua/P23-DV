#ifndef SLAM_FROM_FILE_H
#define SLAM_FROM_FILE_H

#include <string>
#include <limits>
#include <chrono>
#include <functional>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"

#include "slam.h"


namespace ns_slam
{
class SlamFromFile: public rclcpp::Node {
public:
    // The file paths are passed as arguments to the constructor and can be modified in the main function
    SlamFromFile();

    ~SlamFromFile();

private:
    GraphSLAM<SlamFromFile> slam_object_;

    std::string share_dir_;
    std::fstream perception_file_;
    std::fstream odometry_file_;

    bool perception_eof_;
    bool odometry_eof_;

    // Dynamic accel map variables
    bool map_ready_;
    int accel_cone_count_;
    int num_observations_;
	std::unordered_map<int, LandmarkInfo> accel_map_;

    PerceptionMsgStructure perception_;
    OdometryMsgStructure odometry_;

    // Slam topics publishers
    rclcpp::Publisher<custom_msgs::msg::LocalMapMsg>::SharedPtr map_publisher_;
    rclcpp::Publisher<custom_msgs::msg::PoseMsg>::SharedPtr pose_publisher_;

    rclcpp::TimerBase::SharedPtr global_timer_;

    bool is_mapping_;
    double sampling_rate_;
    unsigned long global_index_;

    double perception_range_;
    int optimization_interval_;

    double odometry_weight_;
    double perception_weight_;

    // Load ROS parameters from config files
    void loadParameters();

    // Sets the perception struct to all zeros
    void resetPerception();

    // Sets the odometry struct to all zeros
    void resetOdometry();

    // Reads the next odometry measurement from the specified file and inserts data into the odometry struct
    bool readNextOdometry();

    // Reads the next perception measurement from the specified file and inserts data into the perception struct
    bool readNextPerception();

    void run_slam();

    void addAccelObservations(const std::vector<PerceptionMeasurement>& observations);

};
} // namespace ns_slam

#endif // SLAM_FROM_FILE_H