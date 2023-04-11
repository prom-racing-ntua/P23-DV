#include <rclcpp/rclcpp.hpp>
#include "lidar.h"

#include <string>
#include <unistd.h>
#include <filesystem>
#include <chrono>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace lidar_namespace {
LidarNode::LidarNode(): Node("lidar_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing Lidar Inference node");

    declare_params();
    horizontal_resolution = get_parameter("hor_res").as_double();
    da = horizontal_resolution * 2 * M_PI / 360;
    rj = get_parameter("rj").as_int();

    uint64_t sec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    char dir[50];
    mkdir("LidarLog", 0777);
    sprintf(dir, "LidarLog/Run_%ld.csv", sec);
    myfile.open(dir);
    frame_index=0;
    myfile << "x,y,z,ring,intensity,timestamp\n";

    auto sensor_qos{
        rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data)
    };
    using std::placeholders::_1;
    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/hesai/pandar", sensor_qos, std::bind(&LidarNode::segment_point_cloud, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "Initialization Lidar Inference node finished");

    

}

LidarNode::~LidarNode() {
    segments.clear();
    prototype_points.clear();
}

void LidarNode::declare_params() {
    declare_parameter("hor_res", 1.0);
    declare_parameter("rj", 1);
}

void LidarNode::segment_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "sub");
    myfile << "--,--,FRAME"<<frame_index<<",--,--,--\n";
    RCLCPP_INFO(this->get_logger(), "wrote-");

    sensor_msgs::PointCloud2Iterator<float> xyzIter(*msg, "x");
    sensor_msgs::PointCloud2Iterator<uint16_t> ringIter(*msg, "ring");
    sensor_msgs::PointCloud2Iterator<float> iIter(*msg, "intensity");
    sensor_msgs::PointCloud2Iterator<double> timeIter(*msg, "timestamp");
    RCLCPP_INFO(this->get_logger(), "initialized iterators");

    // for (int i=0; i<msg->fields.size(); i++) {
    //     std::cout << msg->fields[i].name << std::endl;
    // }

    for (; xyzIter != xyzIter.end(); ++xyzIter, ++ringIter, ++iIter, ++timeIter) {

        myfile << xyzIter[0] << "," << xyzIter[1] << "," << xyzIter[2]  << "," << ringIter[0] << "," << iIter[0] << "," << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << "," << timeIter[0] << std::endl;
    }
    RCLCPP_INFO(this->get_logger(), "finished cloud");
    
    std::cout << frame_index << std::endl;
    frame_index++;
}
}