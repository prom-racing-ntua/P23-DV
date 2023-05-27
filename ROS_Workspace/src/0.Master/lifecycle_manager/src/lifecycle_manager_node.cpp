#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "lifecycle_manager_node.hpp"

/*
    TODO List:
        2. Make the configuration loading client safer.
        3. Better Error Handling, all around.
*/

namespace lifecycle_manager_namespace
{
    LifecycleManagerNode::LifecycleManagerNode() : Node("lifecycle_manager")
    {
        RCLCPP_INFO(get_logger(), "Initializing Lifecycle Manager");

        currentDVStatus = p23::DV_Status::STARTUP;
        currentASStatus = p23::AS_Status::AS_OFF;
        currentMission = p23::Mission::MISSION_UNLOCKED;

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

        initializeLifecycleClients(nodeList);
        RCLCPP_INFO(get_logger(), "Lifecycle Manager Initialized");
    }

    void LifecycleManagerNode::initializeServices()
    {
        using namespace std::placeholders;

        dvStatusService_ = create_service<custom_msgs::srv::DriverlessStatus>(
            std::string(get_name()) + std::string("/change_driverless_status"),
            std::bind(&LifecycleManagerNode::changeDVState, this, _1, _2)
            );
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
        }

        using ServiceResponseFuture = rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture;
        auto response_received_callback = [this, nodeName](ServiceResponseFuture future) {
            auto result = future.get();
            bool nodeError{ true };

            if (result->current_state.id == State::PRIMARY_STATE_UNKNOWN)
            {
                RCLCPP_ERROR_STREAM(get_logger(), "Cannot get the state of Node " << nodeName << ". Going to NODE_PROBLEM State");
                // Actually go to node problem...
                nodeError = true;
            }
            else
            {
                std::string nodeStatus{ result->current_state.label };
                RCLCPP_INFO_STREAM(get_logger(), "Status of node "<< nodeName << " is "<< nodeStatus);
                bool nodeError = false;
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

        if (!changeStateServiceHandler->wait_for_service(5s)) {
             RCLCPP_ERROR(
                get_logger(),
                "Service %s is not available.",
                changeStateServiceHandler->get_service_name());
        }

        using ServiceResponseFuture = rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture;
        auto response_received_callback = [this, nodeName](ServiceResponseFuture future) {
            auto result = future.get();
            bool nodeStatusChangeSuccess = result.get()->success;

            if (nodeStatusChangeSuccess) {
                RCLCPP_INFO(get_logger(), "Changed Status of node %s", nodeName.c_str());
            }
            else {
                RCLCPP_INFO(get_logger(), "Couldn't change status of node %s", nodeName.c_str());
            }
        };

        auto future_result = changeStateServiceHandler->async_send_request(request, response_received_callback);
    }

    void LifecycleManagerNode::changeDVState(const std::shared_ptr<custom_msgs::srv::DriverlessStatus::Request> request,
        std::shared_ptr<custom_msgs::srv::DriverlessStatus::Response> response)
    {
        p23::DV_Transitions newDVStatus = static_cast<p23::DV_Transitions>(request->new_status.id);
        p23::Mission missionSent = static_cast<p23::Mission>(request->mission.id);

        // These checks should be done by p23 status
        if ((currentDVStatus == newDVStatus) && (newDVStatus == p23::MISSION_SELECTED) && (missionSent != currentMission)) {
            RCLCPP_INFO(get_logger(), "Re-selecting mission, reset every other node and re-configure");
            reselectMission(missionSent);
        }
        else if (currentDVStatus == newDVStatus) {
            RCLCPP_INFO(get_logger(), "Already in this state, skipping request");
        }

        else if (currentDVStatus > newDVStatus) {
            RCLCPP_INFO(get_logger(), "Should not go back to a previous state");
        }

        else {
            switch(newDVStatus) {
                case(p23::DV_Transitions::ON_STARTUP):
                    // RCLCPP_INFO(get_logger(), "Should never get here... :3");
                    startup();
                    break;
                case(p23::DV_Transitions::SHUTDOWN_NODES):
                    // RCLCPP_INFO(get_logger(), "Received LV_ON state change. Make sure that everyone is alive but unconfigured");
                    break;
                case(p23::DV_Transitions::ON_MISSION_LOCKED):
                    RCLCPP_INFO(get_logger(), "Received MISSION_SELECTED state change. Configure the nodes based on mission selection");
                    configureNodes(missionSent);
                    break;
                case(p23::DV_Transitions::ON_MISSION_UNLOCKED):
                    RCLCPP_INFO(get_logger(), "Received DV_READY state change. Activate everyone except Controls");
                    // cleanupNodes();
                    break;
                case(p23::DV_Transitions::ON_AS_READY):
                    RCLCPP_INFO(get_logger(), "Received DV_DRIVING state change. Activate controls and pray that P23 doesn't unalive anyone");
                    activateSystem();
                    break;
                case(p23::DV_Transitions::ON_AS_DRIVING):
                    // RCLCPP_INFO(get_logger(), "TODO!");
                    activateControls();
                    break;
            }
        }

        // These may be unecessary
        // currentDVStatus = newDVStatus;
        if (currentDVStatus == p23::MISSION_SELECTED)
            currentMission = missionSent;

        response->success = true;
    }

    void LifecycleManagerNode::loadParameters() {
        nodeList = declare_parameter<std::vector<std::string>>("managing_node_list",
            { "acquisition_left", "acquisition_right", "acquisition_center", "inference",
                "velocity_estimation", "slam", "saltas", "path_planning"
            });
        heartbeatTimeoutPeriod = declare_parameter<int>("heartbeat_timeout_period", 2000);
        heartbeatFrequency = declare_parameter<int>("heartbeat_frequency", 5);
    }
}
