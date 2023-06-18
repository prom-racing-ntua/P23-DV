#ifndef LIFECYCLE_MANAGER_NODE_HPP
#define LIFECYCLE_MANAGER_NODE_HPP

/* C/C++ Imports */
#include <memory>
#include <chrono>
#include <atomic>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <string>
#include <cstring>
#include <thread>
#include <unistd.h>

/* ROS2 Libraries */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

/* Messages */
#include "custom_msgs/msg/driverless_transition.hpp"
#include "custom_msgs/msg/lifecycle_node_status.hpp"
#include "lifecycle_msgs/msg/state.hpp"

/* Services */
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "custom_msgs/srv/driverless_transition.hpp"
#include "custom_msgs/srv/resurrect_node.hpp"
#include "custom_msgs/srv/resurrect_order.hpp"

/* Actions */
#include "custom_msgs/action/driverless_transition.hpp"

/* Custom Libraries */
#include "p23_common.h"


/* Taken ready from a ROS2 example, used to safely wait for a response from a service */
template<typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait)
{
    auto end = std::chrono::steady_clock::now() + time_to_wait;
    std::chrono::milliseconds wait_period(100);
    std::future_status status = std::future_status::timeout;
    do {
        auto now = std::chrono::steady_clock::now();
        auto time_left = end - now;
        if (time_left <= std::chrono::seconds(0)) {break;}
        status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
    } while (rclcpp::ok() && status != std::future_status::ready);
    return status;
}

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
    using DVTransition = custom_msgs::action::DriverlessTransition;
    using GoalHandle = rclcpp_action::ServerGoalHandle<DVTransition>;
    using lifecycle_msgs::msg::Transition;

    class LifecycleManagerNode : public rclcpp::Node {
    private:
        // Configuration and Launch folders
        std::string packageShareDirectory, configFolder, launchFolder;

        // Lists of managed nodes
        std::vector<std::string> nodeList;
        std::vector<std::string> nodesToShutdown;
        std::string controlsNode;

        // The first 2 dictionaries are used for the get_state and change_state services that the lifecycle nodes provide
        // automatically upon creation.
        std::unordered_map<std::string,rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> lifecycleGetStateMap;
        std::unordered_map<std::string,rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> lifecycleChangeStateMap;
        std::unordered_map<std::string, bool> nodeStateMap;

        /* Node Initialization Functions*/
        void initializeServices();
        void initializeLifecycleClients(std::vector<std::string> nodeList);
        void loadParameters();

        /* Publisher and timer that send to P23 Status the error state of the managed nodes. */
        rclcpp::Publisher<custom_msgs::msg::LifecycleNodeStatus>::SharedPtr node_state_publisher_;
        rclcpp::TimerBase::SharedPtr heartbeatTimer;
        int heartbeatTimeoutPeriod, heartbeatFrequency, heartbeatTimerDuration;
        /* All Action based things for the Lifecycle Manager */
        rclcpp_action::Server<DVTransition>::SharedPtr dv_status_service;
        std::shared_ptr<GoalHandle> ongoing_goal_handle;
        uint8_t goalCounter, failedTransitionCounter;
        // When receiving new action goal
        rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DVTransition::Goal> goal);
        // When current goal is canceled
        rclcpp_action::CancelResponse handleCancellation(const std::shared_ptr<GoalHandle> goal_handle);
        // Executed after new goal is accepted
        void handleAccept(const std::shared_ptr<GoalHandle> goal_handle);
        void publishActionFeedback();
        // Timer to check goal progression so to not block in the while loop
        rclcpp::CallbackGroup::SharedPtr timer_cb_group;
        rclcpp::TimerBase::SharedPtr goalTimer;

        /*
            Functions for controlling the lifecycle nodes, individually. Each lifecycle node creates two major services:
            1. {node_name}/get_state
            2. {node_name}/change_state

            In the Lifecycle Manager node, we create 2 clients for every node that handle one of the above services

            The initialization happens with the above function. We create the services and put them in a dictionary with
            the node_name as the key. The configuration loading happens with a great Alvania (that works)
        */
        void getNodeState(std::string nodeName);
        void changeNodeState(std::uint8_t transition, std::string nodeName);
        

        /* Configuration handling */
        std::unordered_map<std::string, std::shared_ptr<rclcpp::AsyncParametersClient>> parameterClientMap;
        std::vector<std::string> listNodeParameters(std::string nodeName);
        bool loadConfigurationFileToNode(std::string nodeName, std::string configurationYamlFilePath);

        // void loadConfigurationFileToNode(std::string nodeName, std::string configFile);  
        /*
            5 Main Functions for controlling the whole state machine of P23

            0. heartbeatCheck: Checks the status of all the nodes. If someone is dead, then either go to NODE_ERROR or try to revive them.
                Send a message to P23 Status with the error state of all the managed nodes.
            1. startup: Transitions from Startup to LV_On. Just activate the heartbeat clock
            2. cleanupNodes: Transitions from Mission_Selected or DV_Ready to LV_On. All nodes should go to unconfigured state.
            3. configureNodes: Transitions from Mission Selected to DV_Ready. All nodes should be inactive (set parameters)
            4. activateSystem: Is called when we enter AS_Ready. All nodes except controls should be active.
            5. activateControls: Transition from DV_Ready to DV_Driving. Called when we enter AS_Driving (have received Go-Signal). 
                Control node should get activated here
        */
        void heartbeatCheck();      
        void startup();
        void cleanupNodes();
        void configureNodes(p23::Mission mission);
        void activateSystem();
        void activateControls();

        /* Called when you need to shutdown specific Nodes: Either the managed node list when in error or mission finished or 
            the nodesToBeShutdown when going to AS_Ready*/
        void shutdownSelectedNodes(std::vector<std::string> nodesToShutdown, uint8_t shutdownTransition);

        /* Node Resurrection feature. Calls from another node */
        bool resurrectionEnabled;
        int heartbeatFailuresBeforeResurrection;
        rclcpp::Client<custom_msgs::srv::ResurrectNode>::SharedPtr nodeResurrectionClient;
        rclcpp::Client<custom_msgs::srv::ResurrectOrder>::SharedPtr nodeResurrectionOrderClient;
        void initializeResurrectionClients();
        void sendResurrectionRequest(std::string nodeName);

    public:
        explicit LifecycleManagerNode();
    };
}

#endif