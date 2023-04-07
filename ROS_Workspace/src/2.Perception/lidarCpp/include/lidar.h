#ifndef LIDAR_H
#define LIDAR_H

#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "custom_msgs/msg/node_sync.hpp"
#include <math.h>

namespace lidar_namespace {

    struct Point {
        double x,y,z;
    };

    class LidarNode: public rclcpp::Node {
        private:
            double da;
            double horizontal_resolution;
            void declare_params();
            int rj;
            std::unordered_map<int, std::unordered_map<int, std::vector<std::tuple<Point, double, double>>>> segments;
            std::unordered_map<int, std::unordered_map<int, std::array<double, 2>>> prototype_points;

            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

        public:
            LidarNode();
            ~LidarNode();

            void segment_point_cloud(const sensor_msgs::msg::PointCloud2 msg);

    };
}


#endif