#ifndef LIFECYCLE_MANAGER_NODE_HPP
#define LIFECYCLE_MANAGER_NODE_HPP

#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <string>
#include <cstring>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include "custom_msgs/srv/driverless_status.hpp"
#include "custom_msgs/msg/driverless_status.hpp"
#include "custom_msgs/msg/lifecycle_node_status.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "yaml-cpp/yaml.h"

typedef enum Mission{
    ACCELERATION = 1,
    SKIDPAD = 2,
    TRACKDRIVE = 3,
    EBS_TEST = 4,
    INSPECTION = 5,
    AUTOX = 6,
    MANUAL = 7
} Mission;

typedef enum AS_Status{
    AS_OFF = 1,
    AS_READY = 2,
    AS_DRIVING = 3,
    AS_FINISHED = 4,
    AS_EMERGENCY = 5
} AS_Status;

typedef enum DV_Status{
    STARTUP = 0,
    LV_ON = 1,
    MISSION_SELECTED = 2,
    DV_READY = 3,
    DV_DRIVING = 4,
    NODE_PROBLEM = 5
} DV_Status;

template<typename T>
void printVector(std::vector<T>& vector)
{
    for (T element: vector)
    {
        std::cout << element << std::endl;
    }
}

template<typename T>
void removeElement(std::vector<T>& vector, T element)
{
    auto iterator = std::find(vector.begin(), vector.end(), element);
    if (iterator != vector.end())
        vector.erase(iterator);
}

namespace lifecycle_manager_namespace
{
    class LifecycleManagerNode : public rclcpp::Node {
    private:
        Mission currentMission;
        AS_Status currentASStatus;
        DV_Status currentDVStatus;

        // Configuration and Launch folders
        std::string packageShareDirectory, configFolder, launchFolder;

        // List of the node names that should run when having selected a mission and are managed by this Lifecycle Manager.
        std::vector<std::string> nodeList;
        std::vector<std::string> nodesToShutdown;

        std::string controlsNode;

        /*
            The first 2 dictionaries are used for the get_state and change_state services that the lifecycle nodes provide
            automatically upon creation. The 3rd dictionary is used to load the configuration file to each node that it 
            manages.
        */

        std::unordered_map<std::string,rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> lifecycleGetStateMap;
        std::unordered_map<std::string,rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> lifecycleChangeStateMap;
        std::unordered_map<std::string, bool> nodeStateMap;

        // std::unordered_map<std::string, rclcpp::AsyncParametersClient::SharedPtr> parameterClientMap;
        std::unordered_map<std::string, pid_t> nodePIDMap;

        rclcpp::Service<custom_msgs::srv::DriverlessStatus>::SharedPtr dvStatusService_;
        rclcpp::Publisher<custom_msgs::msg::LifecycleNodeStatus>::SharedPtr node_state_publisher_;

        // Timer to check if the nodes that we are managing are still alive
        rclcpp::TimerBase::SharedPtr heartbeatTimer;
        int heartbeatTimeoutPeriod, heartbeatFrequency, heartbeatTimerDuration;

        void initializeServices();
        void initializeLifecycleClients(std::vector<std::string> nodeList);
        void loadConfigurationFileToNode(std::string nodeName, std::string configFile);        

        /*
            Functions for controlling the lifecycle nodes, individually. Each lifecycle node creates two major services:
            1. {node_name}/get_state
            2. {node_name}/change_state

            In the Lifecycle Manager node, we create 2 clients for every node that handle one of the above services

            The initialization happens with the above function. We create the services and put them in a dictionary with
            the node_name as the key.
        */
       
        void getNodeState(std::string nodeName);
        void changeNodeState(std::uint8_t transition, std::string nodeName);
        
        void verifyDVState();
        void loadParameters();

        // 4 Main Functions for controlling the whole state machine of P23
        void LV_On();
        void Mission_Selected(Mission mission);
        void DV_Ready();
        void DV_Driving();

        void reselectMission(Mission newMission);
        void shutdownSelectedNodes(std::vector<std::string> nodesToShutdown);

        /* Experimental Code - Should not be used by anyone (yet). Egw den metraw... */
        pid_t launchNode(std::string nodeName, std::string packageName, std::string runCommand);
        void resurrectNode(std::string nodeName);

    public:
        explicit LifecycleManagerNode();
        void changeDVState(const std::shared_ptr<custom_msgs::srv::DriverlessStatus::Request> request,
            std::shared_ptr<custom_msgs::srv::DriverlessStatus::Response> response);
    };
}

#endif