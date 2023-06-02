#ifndef SLAM_FROM_FILE_H
#define SLAM_FROM_FILE_H

#include <string>
#include <limits>
#include <chrono>
#include <functional>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"
#include "custom_msgs/msg/velocity_to_mpc.hpp"

#include "slam_common.h"
#include "slam.h"


namespace ns_slam
{
class GraphSLAM;

class SlamFromFile: public rclcpp::Node {
public:
    // The file paths are passed as arguments to the constructor and can be modified in the main function
    SlamFromFile();

    ~SlamFromFile();

private:
    ns_slam::GraphSLAM slam_object_;

    std::string share_dir_;
    std::fstream perception_file_;
    std::fstream odometry_file_;

    bool perception_eof_;
    bool odometry_eof_;

    PerceptionMsgStructure perception_;
    OdometryMsgStructure odometry_;

    // Slam topics publishers
    rclcpp::Publisher<custom_msgs::msg::LocalMapMsg>::SharedPtr map_publisher_;
    rclcpp::Publisher<custom_msgs::msg::PoseMsg>::SharedPtr pose_publisher_;
    rclcpp::Publisher<custom_msgs::msg::VelocityToMpc>::SharedPtr velocity_publisher_;

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
};
} // namespace ns_slam

#endif // SLAM_FROM_FILE_H