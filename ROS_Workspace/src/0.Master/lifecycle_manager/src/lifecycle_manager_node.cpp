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
        RCLCPP_INFO(get_logger(), "Lifecycle Manager Initialized");
    }

    void LifecycleManagerNode::initializeServices()
    {
        using namespace std::placeholders;
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
            // nodeStateMap[nodeName] = true;
            failedTransitionCounter++;
            return;
        }

        auto future_result = changeStateServiceHandler->async_send_request(request);

        /* Instead of using a callback (which doesn't always work out check for the future status every once in a while)*/
        auto future_status = wait_for_result(future_result, std::chrono::seconds(5));

        if (future_status != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "Server time out while changing state for node %s", nodeName.c_str());
            return;
        }

        // We have an answer, let's print our success.
        if (future_result.get()->success) {
            RCLCPP_INFO(get_logger(), "Changed Status of node %s, new goal counter: %u", nodeName.c_str(), goalCounter);
            goalCounter--;
        } else {
            RCLCPP_INFO(get_logger(), "Couldn't change status of node %s", nodeName.c_str());
            // nodeStateMap[nodeName] = true;
            failedTransitionCounter++;
        }

        return;
    }

    void LifecycleManagerNode::loadParameters() {
        nodeList = declare_parameter<std::vector<std::string>>("managing_node_list",
            { "acquisition_left", "acquisition_right", "inference",
                "velocity_estimation", "slam", "saltas", "path_planning", "mpc", "pure_pursuit"
            });
        heartbeatTimeoutPeriod = declare_parameter<int>("heartbeat_timeout_period", 2000);
        heartbeatFrequency = declare_parameter<int>("heartbeat_frequency", 5);
    }
}