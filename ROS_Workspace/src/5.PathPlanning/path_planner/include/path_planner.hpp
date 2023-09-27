
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cstdio>

//#include <algorithm>

#include "custom_msgs/msg/cone_struct.hpp"
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/point2_struct.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/waypoints_msg.hpp"

#include "triangulation_cgal.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Path_Planner_Node: public rclcpp::Node
{
    public:
        Path_Planner_Node();
        ~Path_Planner_Node();
    private:
        rclcpp::Subscription<custom_msgs::msg::LocalMapMsg>::SharedPtr sub_mapper;
        //rclcpp::Subscription<custom_msgs::msg::PoseMsg>::SharedPtr sub_odometry;
        rclcpp::Publisher<custom_msgs::msg::WaypointsMsg>::SharedPtr pub_waypoints;
        //void pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr) const;
        void mapping_callback(const custom_msgs::msg::LocalMapMsg::SharedPtr);
        Triangulation waymaker;
        int selection_radius_small, selection_radius_big, selection_angle;
        void parameter_load();
        double total_execution_time;
        custom_msgs::msg::WaypointsMsg last_path;
        float last_length;
        Point last_position;
        float average_angle;
        float get_length(std::vector<Point> path)const;
        float get_angle_avg(std::vector<Point> path)const;
        //Point last_direction;

        FILE *timestamp_log;
};