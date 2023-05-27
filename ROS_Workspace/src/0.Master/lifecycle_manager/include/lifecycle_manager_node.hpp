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
#include "p23_common.h"


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
        p23::Mission currentMission;
        p23::AS_Status currentASStatus;
        p23::DV_Status currentDVStatus;

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
        
        //Transitions from Startup to LV_On. Just activate the heartbeat clock
        void startup();
        // Transitions from Mission_Selected or DV_Ready to LV_On. All nodes should go to unconfigured state.
        // void cleanupNodes();
        // Transitions from Mission Selected to DV_Ready. All nodes should be inactive (set parameters)
        void configureNodes(p23::Mission mission);
        // Is called when we enter AS_Ready. All nodes except controls should be active.
        void activateSystem();
        // Transition from DV_Ready to DV_Driving. Called when we enter AS_Driving (have received Go-Signal). Control node should get activated here
        void activateControls();
        
        void shutdownSelectedNodes(std::vector<std::string> nodesToShutdown);
        
        // Probably to be used instead of cleanupNodes() or configureNodes() ...
        void reselectMission(p23::Mission newMission);

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