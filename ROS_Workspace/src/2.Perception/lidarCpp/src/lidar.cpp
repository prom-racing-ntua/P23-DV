#include <rclcpp/rclcpp.hpp>
#include "lidar.h"

namespace lidar_namespace {
    LidarNode::LidarNode() : Node("lidar_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing CAN2USB node");

        declare_params();
        horizontal_resolution = get_parameter("hor_res").as_double();
        da = horizontal_resolution*2*M_PI/360;
        rj = get_parameter("rj").as_int();

        auto sensor_qos {
            rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data)
        };
        using std::placeholders::_1;
        point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/hesai/pandar", 10, std::bind(&LidarNode::segment_point_cloud, this, _1)
        );
    
    }

    LidarNode::~LidarNode() {
        segments.clear();
        prototype_points.clear();
    }

    void LidarNode::declare_params() {
        declare_parameter("hor_res", 1);
        declare_parameter("rj", 1);
    }

    void LidarNode::segment_point_cloud(const sensor_msgs::msg::PointCloud2 msg){
        segments.clear();
        prototype_points.clear();
        // sensor_msgs::PointCloud2Iterator<float> pcIter(msg, "x");

        // for (; pcIter != pcIter.end(); ++pcIter) {
        //     std::cout << pcIter[0] << " " << pcIter[1] << " " << pcIter[2] << std::endl;
        // }
    }
}