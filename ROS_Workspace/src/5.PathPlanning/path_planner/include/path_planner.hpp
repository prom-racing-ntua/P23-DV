
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
        /* Publishers and subscribers */
        rclcpp::Subscription<custom_msgs::msg::LocalMapMsg>::SharedPtr sub_mapper;
        rclcpp::Publisher<custom_msgs::msg::WaypointsMsg>::SharedPtr pub_waypoints;

        /* ROS callback */
        void mapping_callback(const custom_msgs::msg::LocalMapMsg::SharedPtr);

        /* Object from triangulation_cgal */
        Triangulation waymaker;

        /* Cone selection variables */
        int selection_radius_small, selection_radius_big, selection_angle;

        /* ROS parameter loading */
        void parameter_load();

        /* Other variables and methods*/
        double total_execution_time;
        custom_msgs::msg::WaypointsMsg last_path;
        float last_length;
        Point last_position;
        float average_angle;
        float get_length(std::vector<Point> path)const;
        float get_angle_avg(std::vector<Point> path)const;

        /* Waypoints total path creation */
        std::vector<Point> added_waypoints;
        std::ofstream save_total_path;
        bool has_completed_total_path;
        int prev_lap;
        custom_msgs::msg::WaypointsMsg finalized;
};