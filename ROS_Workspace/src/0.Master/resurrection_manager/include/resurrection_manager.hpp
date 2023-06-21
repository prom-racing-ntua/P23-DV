#ifndef RESURRECTION_MANAGER_HPP
#define RESURRECTION_MANAGER_HPP

/* C/C++ Imports */
#include <memory>
#include <unordered_map>
#include <vector>
#include <string>

#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

/* ROS2 Libraries */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include "rclcpp/utilities.hpp"

/* Services */
#include "custom_msgs/srv/resurrect_node.hpp"
#include "custom_msgs/srv/resurrect_order.hpp"


const std::unordered_map<std::string, std::string> packageMap{
    {"velocity_estimation", "velocity_estimation"},
    {"acquisition_left", "perception"},
    {"acquisition_right", "perception"},
    {"inference", "perception"},
    {"slam", "slam"},
    {"path_planning", "path_planner"},
    {"mpc", "mpc"},
    {"pure_pursuit", "pid_pp_controller"}
};

const std::unordered_map<std::string, std::string> executableMap{
    {"velocity_estimation", "lifecycle_velocity_estimation"},
    {"acquisition_left", "lifecycle_acquisition --ros-args -r __node:=acquisition_left"},
    {"acquisition_right", "lifecycle_acquisition --ros-args -r __node:=acquisition_right"},
    {"inference", "lifecycle_inference"},
    {"slam", "lifecycle_slam"},
    {"path_planning", "lifecycle_path_planner"},
    {"mpc", "lifecycle_mpc"},
    {"pure_pursuit", "lifecycle_pid_pp_controller"}
};

namespace resurrection_manager_namespace
{
    class ResurrectionManagerNode : public rclcpp::Node {
        private:
            std::vector<std::string> nodeList;
            std::unordered_map<std::string, pid_t> nodePIDMap;

            rclcpp::Service<custom_msgs::srv::ResurrectNode>::SharedPtr resurrectionService;
            rclcpp::Service<custom_msgs::srv::ResurrectOrder>::SharedPtr resurrectionOrderService;

            /* Basic Node initialization functions */
            void loadParameters();
            void initializeServices();

            /* Functions to control the nodes that are managed */
            pid_t resurrectNode(std::string nodeName);
            void sendSignalToNode(std::string nodeName, uint8_t signal);

            /* Service handling functions */
            void handleResurrection(const std::shared_ptr<custom_msgs::srv::ResurrectNode::Request> request, std::shared_ptr<custom_msgs::srv::ResurrectNode::Response> response);
            void multicastOrder(const std::shared_ptr<custom_msgs::srv::ResurrectOrder::Request> request, std::shared_ptr<custom_msgs::srv::ResurrectOrder::Response> response);
        public:
            explicit ResurrectionManagerNode();
    };
}
#endif