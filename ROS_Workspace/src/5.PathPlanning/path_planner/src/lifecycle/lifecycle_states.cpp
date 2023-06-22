#include "rclcpp/rclcpp.hpp"
#include "rmw/qos_profiles.h"
#include <rclcpp/qos.hpp>

#include "lifecycle_pathplanning_node.hpp"

namespace path_planner
{
    path_planner::CallbackReturn 
        LifecyclePathPlanner::on_configure(const rclcpp_lifecycle::State & state)
    {
        waymaker.init
        (
            get_parameter("maximum_angle").as_int(),
            get_parameter("maximum_edge_angle").as_int(),
            get_parameter("maximum_distance").as_int(),
            0, 0,
            get_parameter("target_depth").as_int(),
            get_parameter("same_edge_penalty").as_int(),
            get_parameter("length_penalty").as_double(),
            get_parameter("angle_penalty").as_double(),
            get_parameter("total_length_reward").as_double(),
            get_parameter("filtering_threshold").as_int()
        );

        using std::placeholders::_1;
        sub_mapper = this->create_subscription<custom_msgs::msg::LocalMapMsg>("local_map", 10, std::bind(&LifecyclePathPlanner::mapping_callback, this, _1));
        pub_waypoints = this->create_publisher<custom_msgs::msg::WaypointsMsg>("waypoints", 10);
        RCLCPP_WARN(get_logger(), "\n-- Path Planner Configured!");

        return path_planner::CallbackReturn::SUCCESS;
    }

    path_planner::CallbackReturn 
        LifecyclePathPlanner::on_activate(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_WARN(get_logger(), "\n-- Path Planner Activated!");
        pub_waypoints->on_activate();
        return path_planner::CallbackReturn::SUCCESS;
    }

    path_planner::CallbackReturn 
        LifecyclePathPlanner::on_deactivate(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_WARN(get_logger(), "\n-- Path Planner De-Activated!");
        pub_waypoints->on_deactivate();
        return path_planner::CallbackReturn::SUCCESS;
    }

    path_planner::CallbackReturn 
        LifecyclePathPlanner::on_cleanup(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_WARN(get_logger(), "\n-- Path Planner Un-Configured!");
        pub_waypoints.reset();
        sub_mapper.reset();
        return path_planner::CallbackReturn::SUCCESS;
    }

    path_planner::CallbackReturn 
        LifecyclePathPlanner::on_shutdown(const rclcpp_lifecycle::State & state)
    {   
        /* If current state is UNCONFIGURED, then just return */
        if (state.id() == 1) {
            return path_planner::CallbackReturn::SUCCESS;
        }

        pub_waypoints.reset();
        sub_mapper.reset();
        return path_planner::CallbackReturn::SUCCESS;
    }

    path_planner::CallbackReturn 
        LifecyclePathPlanner::on_error(const rclcpp_lifecycle::State & state)
    {
        return path_planner::CallbackReturn::SUCCESS;
    }
}