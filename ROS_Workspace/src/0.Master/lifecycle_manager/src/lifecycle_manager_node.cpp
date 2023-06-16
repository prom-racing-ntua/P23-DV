#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "lifecycle_manager_node.hpp"


namespace lifecycle_manager_namespace
{
    LifecycleManagerNode::LifecycleManagerNode() : Node("lifecycle_manager")
    {
        RCLCPP_INFO(get_logger(), "Initializing Lifecycle Manager");
        
        loadParameters();
        initializeServices();
        /*
            Setup folders for controlling the configuartion and the launch files of the nodes through
            the lifecycle manager.
        */
        packageShareDirectory = ament_index_cpp::get_package_share_directory("lifecycle_manager");
        configFolder = packageShareDirectory + std::string("/config");
        launchFolder = packageShareDirectory + std::string("/launch");

        get_parameter("managing_node_list", nodeList);
        get_parameter("heartbeat_timeout_period", heartbeatTimeoutPeriod);
        get_parameter("heartbeat_frequency", heartbeatFrequency);

        heartbeatTimerDuration = (1000/heartbeatFrequency);

        timer_cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        goalTimer = create_wall_timer(std::chrono::seconds(1), std::bind(&LifecycleManagerNode::publishActionFeedback, this), timer_cb_group);
        goalTimer->cancel();

        initializeLifecycleClients(nodeList);
        // startup();
        RCLCPP_INFO(get_logger(), "Lifecycle Manager Initialized");
    }

    void LifecycleManagerNode::initializeServices()
    {
        using namespace std::placeholders;

        // dvStatusService_ = create_service<custom_msgs::srv::DriverlessTransition>(
        //     std::string(get_name()) + std::string("/change_driverless_status"),
        //     std::bind(&LifecycleManagerNode::changeDVState, this, _1, _2)
        //     );
        
        dv_status_service = rclcpp_action::create_server<DVTransition>(
            this,
            std::string(get_name()) + std::string("/change_driverless_status"),
            std::bind(&LifecycleManagerNode::handleGoal, this, _1, _2),
            std::bind(&LifecycleManagerNode::handleCancellation, this, _1),
            std::bind(&LifecycleManagerNode::handleAccept, this, _1));
    }

    void LifecycleManagerNode::initializeLifecycleClients(std::vector<std::string> nodeList)
    {
        for (auto s: nodeList) {
            std::string getStateServiceName = s + std::string("/get_state");
            std::string changeStateServiceName = s + std::string("/change_state");

            lifecycleGetStateMap[s] = create_client<lifecycle_msgs::srv::GetState>(getStateServiceName);
            lifecycleChangeStateMap[s] = create_client<lifecycle_msgs::srv::ChangeState>(changeStateServiceName);
            /* Make the assumption that every node that you manage is alive and well :) */
            nodeStateMap[s] = false;
        }

        node_state_publisher_ = create_publisher<custom_msgs::msg::LifecycleNodeStatus>(
            std::string(get_name()) + std::string("/lifecycle_node_status"), 10);
    }

    void LifecycleManagerNode::getNodeState(std::string nodeName)
    {
        using namespace std::chrono_literals;
        using lifecycle_msgs::msg::State;

        auto getStateServiceHandler = lifecycleGetStateMap.at(nodeName);
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        
        if (!getStateServiceHandler->wait_for_service(std::chrono::milliseconds(heartbeatTimeoutPeriod))) {
            RCLCPP_ERROR_STREAM(get_logger(), "Service " << getStateServiceHandler->get_service_name() << " is not available");
            nodeStateMap[nodeName] = true;
            return;
        }

        using ServiceResponseFuture = rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture;
        auto response_received_callback = [this, nodeName](ServiceResponseFuture future) {
            auto result = future.get();
            bool nodeError = true;
            if (result->current_state.id == State::PRIMARY_STATE_UNKNOWN)
            {
                RCLCPP_ERROR_STREAM(get_logger(), "Cannot get the state of Node " << nodeName << ". Going to NODE_PROBLEM State");
                // Actually go to node problem...
                // Vasilis: This is going to happen from the P23 Status Node
                nodeError = true;
            }
            else
            {
                std::string nodeStatus{ result->current_state.label };
                // RCLCPP_INFO_STREAM(get_logger(), "Status of node "<< nodeName << " is "<< nodeStatus);
                nodeError = false;
            }
            nodeStateMap[nodeName] = nodeError;
        };

        auto future_result = getStateServiceHandler->async_send_request(request, response_received_callback);
    }

    void LifecycleManagerNode::changeNodeState(std::uint8_t transition, std::string nodeName)
    {
        using namespace std::chrono_literals;

        auto changeStateServiceHandler = lifecycleChangeStateMap.at(nodeName);
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

        request->transition.id = transition;

        if (!changeStateServiceHandler->wait_for_service(1s)) {
             RCLCPP_ERROR(
                get_logger(),
                "Service %s is not available.",
                changeStateServiceHandler->get_service_name());
            nodeStateMap[nodeName] = true;
            failedTransitionCounter++;
            return;
        }

        using ServiceResponseFuture = rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture;
        auto response_received_callback = [this, nodeName](ServiceResponseFuture future) {
            auto result = future.get();
            bool nodeStatusChangeSuccess = result.get()->success;
            // RCLCPP_INFO(get_logger(), "Node %s returned callback with success status: %i", nodeName.c_str(), nodeStatusChangeSuccess);

            if (nodeStatusChangeSuccess) {
                goalCounter--;
                RCLCPP_INFO(get_logger(), "Changed Status of node %s, new goal counter: %u", nodeName.c_str(), goalCounter);
            }
            else {
                RCLCPP_INFO(get_logger(), "Couldn't change status of node %s", nodeName.c_str());
                /* This will throw up a NODE_PROBLEM error */
                nodeStateMap[nodeName] = true;
                failedTransitionCounter++;
            }
        };
        // RCLCPP_INFO(get_logger(), "Changing state of node %s to %u", nodeName.c_str(), transition);
        auto future_result = changeStateServiceHandler->async_send_request(request, response_received_callback);
    }

    // void LifecycleManagerNode::changeDVState(const std::shared_ptr<custom_msgs::srv::DriverlessTransition::Request> request,
    //     std::shared_ptr<custom_msgs::srv::DriverlessTransition::Response> response)
    // {
    //     p23::DV_Transitions newDVStatus = static_cast<p23::DV_Transitions>(request->transition.id);
    //     p23::Mission missionSent = static_cast<p23::Mission>(request->mission.id);

    //     switch(newDVStatus) {
    //         case(p23::DV_Transitions::ON_STARTUP):
    //             RCLCPP_INFO(get_logger(), "Received Startup signal, start a heartbeat check for each node managed");
    //             startup();
    //             break;
    //         case(p23::DV_Transitions::SHUTDOWN_NODES):
    //             RCLCPP_INFO(get_logger(), "Either Mission Finished or AS Emergency, shutting down currently running nodes");
    //             shutdownSelectedNodes(nodeList, Transition::TRANSITION_ACTIVE_SHUTDOWN);
    //             break;
    //         case(p23::DV_Transitions::ON_MISSION_LOCKED):
    //             RCLCPP_INFO(get_logger(), "Received mission, configure the nodes");
    //             configureNodes(missionSent);
    //             break;
    //         case(p23::DV_Transitions::ON_MISSION_UNLOCKED):
    //             RCLCPP_INFO(get_logger(), "Unlocking mission, waiting for new mission to arrive, cleanup nodes");
    //             cleanupNodes();
    //             currentMission = p23::MISSION_UNLOCKED;
    //             break;
    //         case(p23::DV_Transitions::ON_AS_READY):
    //             RCLCPP_INFO(get_logger(), "Received AS Ready, shutdown the nodes that are not used and wait for AS driving");
    //             activateSystem();
    //             break;
    //         case(p23::DV_Transitions::ON_AS_DRIVING):
    //             RCLCPP_INFO(get_logger(), "Received AS Driving, activate control node");
    //             activateControls();
    //             break;
    //     }
    //     response->success = true;
    // }

    void LifecycleManagerNode::loadParameters() {
        nodeList = declare_parameter<std::vector<std::string>>("managing_node_list",
            { "acquisition_left", "acquisition_right", "inference",
                "velocity_estimation", "slam", "saltas", "path_planning", "mpc", "pure_pursuit"
            });
        heartbeatTimeoutPeriod = declare_parameter<int>("heartbeat_timeout_period", 2000);
        heartbeatFrequency = declare_parameter<int>("heartbeat_frequency", 5);
    }
}