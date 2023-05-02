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

        currentDVStatus = STARTUP;
        currentASStatus = AS_OFF;
        currentMission = INSPECTION;

        initializeServices();
        loadParameters();

        /*
            Setup folders for controlling the configuartion and the launch files of the nodes through
            the lifecycle manager.
        */
        packageShareDirectory = ament_index_cpp::get_package_share_directory("lifecycle_manager");
        configFolder = packageShareDirectory + std::string("/config");
        launchFolder = packageShareDirectory + std::string("/launch");

        RCLCPP_INFO(get_logger(), "Config Share Directory %s", configFolder.c_str());

        // Read Node List from parameters
        get_parameter("managing_node_list", nodeList);

        initializeLifecycleClients(nodeList);

        /*
            Create a timer that checks for a heartbeat of every node every X milliseconds. Also
            verifies the state.
        */

        // heartbeatTimer = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&LifecycleManagerNode::verifyDVState, this));

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
        // auto callbackGroup = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        
        for (auto s: nodeList) {
            std::string getStateServiceName = s + std::string("/get_state");
            std::string changeStateServiceName = s + std::string("/change_state");
            lifecycleGetStateMap[s] = create_client<lifecycle_msgs::srv::GetState>(getStateServiceName);
            lifecycleChangeStateMap[s] = create_client<lifecycle_msgs::srv::ChangeState>(changeStateServiceName);
            // parameterClientMap[s] = std::make_shared<rclcpp::AsyncParametersClient>(this, s, 10, callbackGroup);
        }
    }

    void LifecycleManagerNode::verifyDVState()
    {
        for (auto node: nodeList) {
            getNodeState(node);
        }
    }

    void LifecycleManagerNode::getNodeState(std::string nodeName)
    {
        using namespace std::chrono_literals;

        auto getStateServiceHandler = lifecycleGetStateMap.at(nodeName);
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

        if (!getStateServiceHandler->wait_for_service(5s)) {
        RCLCPP_ERROR(
            get_logger(),
            "Service %s is not available.",
            getStateServiceHandler->get_service_name());
        }

        using ServiceResponseFuture = rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture;
        auto response_received_callback = [this, nodeName](ServiceResponseFuture future) {
            auto result = future.get();

            std::string nodeStatus = result->current_state.label.c_str();
            RCLCPP_INFO(get_logger(), "Status of node %s is %s", nodeName.c_str(), nodeStatus.c_str());
            return result->current_state.id;
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
                // Handle error
            }
        };

        auto future_result = changeStateServiceHandler->async_send_request(request, response_received_callback);
    }

    void LifecycleManagerNode::changeDVState(const std::shared_ptr<custom_msgs::srv::DriverlessStatus::Request> request,
        std::shared_ptr<custom_msgs::srv::DriverlessStatus::Response> response)
    {
        DV_Status newDVStatus = static_cast<DV_Status>(request->new_status.id);
        Mission missionSent = static_cast<Mission>(request->mission.id);

        // check if dv status changes successfully
        bool success = false;

        if (currentDVStatus == newDVStatus) {
            RCLCPP_INFO(get_logger(), "Already in this state, skipping request");
        }
        else if (currentDVStatus > newDVStatus) {
            RCLCPP_INFO(get_logger(), "Should not go back to a previous state");
        }
        else {
            switch(newDVStatus){
                case(STARTUP):
                    RCLCPP_INFO(get_logger(), "Should never get here... :3");
                    break;
                case(LV_ON):
                    RCLCPP_INFO(get_logger(), "Received LV_ON state change. Make sure that everyone is alive but unconfigured");
                    success = LV_On();
                    break;
                case(MISSION_SELECTED):
                    RCLCPP_INFO(get_logger(), "Received MISSION_SELECTED state change. Configure the nodes based on mission selection");
                    success = Mission_Selected(missionSent);
                    break;
                case(DV_READY):
                    RCLCPP_INFO(get_logger(), "Received DV_READY state change. Activate everyone except Controls");
                    success = DV_Ready();
                    break;
                case(DV_DRIVING):
                    RCLCPP_INFO(get_logger(), "Received DV_DRIVING state change. Activate controls and pray that P23 doesn't unalive anyone");
                    success = DV_Driving();
                    break;
                case(NODE_PROBLEM):
                    RCLCPP_INFO(get_logger(), "TODO!");
                    break;
            }
        }

        if (success) {
            currentDVStatus = newDVStatus;
            if (currentDVStatus == MISSION_SELECTED)
                currentMission = missionSent;
            
            RCLCPP_INFO(get_logger(), "Successfully Changed DV Status to %d", currentDVStatus);
        }
        response->success = success;
    }

    void LifecycleManagerNode::loadParameters()
    {
        declare_parameter<std::vector<std::string>>("managing_node_list",
        {   "acquisition_left", "acquisition_right", "acquisition_center", "inference",
            "velocity_estimation", "slam", "saltas", "path_planning"
        });
    }
}
