#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "custom_msgs/msg/cone_struct.hpp"
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/point2_struct.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/waypoints_msg.hpp"

#include "triangulation_cgal.hpp"
#include "node_logger.hpp"


#include <chrono>
#include <functional>
#include <filesystem>
#include <cstdio>
#include <memory>
#include <string>

namespace path_planner
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class LifecyclePathPlanner: public rclcpp_lifecycle::LifecycleNode {
        private:
            rclcpp::Subscription<custom_msgs::msg::LocalMapMsg>::SharedPtr sub_mapper;
            rclcpp_lifecycle::LifecyclePublisher<custom_msgs::msg::WaypointsMsg>::SharedPtr pub_waypoints;
            void mapping_callback(const custom_msgs::msg::LocalMapMsg::SharedPtr);
            Triangulation waymaker;
            int selection_radius_small, selection_radius_big, selection_angle;
            void loadParameters();
            double total_execution_time;
            custom_msgs::msg::WaypointsMsg last_path;
            float last_length;
            Point last_position;
            float average_angle;
            float get_length(std::vector<Point> path)const;
            float get_angle_avg(std::vector<Point> path)const;
            Logger timestamp_log;
            double pub_time_1, pub_time_2;
        public:
            explicit LifecyclePathPlanner();
            ~LifecyclePathPlanner();
        protected:
            path_planner::CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
            path_planner::CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
            path_planner::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
            path_planner::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) ;
            path_planner::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
            path_planner::CallbackReturn on_error(const rclcpp_lifecycle::State & state);
    };
}