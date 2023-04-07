#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "lifecycle_manager_node.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/utilities.hpp"

#include "custom_msgs/srv/driverless_status.hpp"
#include "custom_msgs/msg/driverless_status.hpp"

/*
    TODO List:
        2. Make the configuration loading client safer.
        3. Better Error Handling, all around.
*/

using lifecycle_msgs::msg::Transition;

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

        heartbeatTimer = create_wall_timer(std::chrono::milliseconds(5000), std::bind(&LifecycleManagerNode::verifyDVState, this));

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
            parameterClients[s] = std::make_shared<rclcpp::AsyncParametersClient>(this, s);
        }
    }

    void LifecycleManagerNode::loadConfigurationFileToNode(std::string nodeName, std::string configFile)
    {
        auto client = parameterClients.at(nodeName);

        client->load_parameters(configFile);   
    }

    unsigned int LifecycleManagerNode::getNodeState(std::string nodeName)
    {
        using namespace std::chrono_literals;

        auto getStateServiceHandler = lifecycleGetStateMap.at(nodeName);

        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

        if (!getStateServiceHandler->wait_for_service(1s)) {
        RCLCPP_ERROR(
            get_logger(),
            "Service %s is not available.",
            getStateServiceHandler->get_service_name());
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        using ServiceResponseFuture = rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture;
        auto response_received_callback = [this, nodeName](ServiceResponseFuture future) {
            auto result = future.get();

            std::string nodeStatus = result->current_state.label.c_str();
            RCLCPP_INFO(get_logger(), "Status of node %s is %s", nodeName.c_str(), nodeStatus.c_str());
            return result->current_state.id;
        };

        auto future_result = getStateServiceHandler->async_send_request(request, response_received_callback);

        return 1;
    }

    bool LifecycleManagerNode::changeNodeState(std::uint8_t transition, std::string nodeName)
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
            return false;
        }

        using ServiceResponseFuture = rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture;
        auto response_received_callback = [this, nodeName](ServiceResponseFuture future) {
            auto result = future.get();

            bool nodeStatusChangeSuccess = result.get()->success;

            if (nodeStatusChangeSuccess) {
                RCLCPP_INFO(get_logger(), "Changed Status of node %s", nodeName.c_str());
                return true;
            }
            else {
                RCLCPP_INFO(get_logger(), "Couldn't change status of node %s", nodeName.c_str());
                return false;
            }
        };

        auto future_result = changeStateServiceHandler->async_send_request(request, response_received_callback);

        return true;
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
                //Configure Nodes
            
            RCLCPP_INFO(get_logger(), "Successfully Changed DV Status to %d", currentDVStatus);
        }

        response->success = success;
    }

    bool LifecycleManagerNode::verifyDVState()
    {

        return true;
        /*
            for node in nodes():
                if (!getNodeState)
                    node is dead or not in correct state
        */
    }

    /*
        The 4 Main DV States that the car should be in.
    */

    bool LifecycleManagerNode::LV_On()
    {
        /*
            Nodes should be in this state:
            Acquisition: Unconfigured 
            Inference: Uconfigured
            Velocity Estimation: Unconfigured
            SLAM: Unconfigured
            Pathplanning: Unconfigured
            Controls: Unconfigured

            1. Send a getNodeState call to make sure that everything is in this state.
        */
        return true;
    }

    bool LifecycleManagerNode::Mission_Selected(Mission mission)
    {
        /*
            Shutdown nodes that are no longer necessary and load the parameter files
            depending on which mission you selected.
                1. Send a shutdown transition - DONE
                2. Delete the clients - DONE
                3. Delete them from the map and the node list - DONE

            Configure Nodes (changeNodeState(Transition::TRANSITION_CONFIGURE))
                1. Select the correct configuartion file based on the mission selected - DONE
                2. Send a changeNodeState transition call to every node remaining - DONE
        */

       std::string configurationFileSelected = configFolder;
       std::vector<std::string> nodesToShutdown = {};

       switch(mission){
            case(ACCELERATION):
                configurationFileSelected += std::string("/acceleration_config.yaml");
                nodesToShutdown = {};
                break;
            case(SKIDPAD):
                configurationFileSelected += std::string("/skidpad_config.yaml");
                nodesToShutdown = {};
                break;
            case(TRACKDRIVE):
                configurationFileSelected += std::string("/trackdrive_config.yaml");
                nodesToShutdown = {};
                break;
            case(EBS_TEST):
                configurationFileSelected += std::string("/ebs_test_config.yaml");
                nodesToShutdown = {};
                break;
            case(INSPECTION):
                configurationFileSelected += std::string("/inspection_config.yaml");
                nodesToShutdown = {};
                break;
            case(AUTOX):
                configurationFileSelected += std::string("/autox_config.yaml");
                nodesToShutdown = {};
                break;
            case(MANUAL):
            // The PC will shutdown so no one cares what happens here...
                break;
       }
            /*
                Shutdown nodes that are no longer necessary due to specific mission being selected.
                Do not forget to delete them from the dictionaries + the node list.
            */

            for (auto node: nodesToShutdown) {
                bool success = changeNodeState(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, node);

                if (!success) {
                    RCLCPP_INFO(get_logger(),"Failed to shutdown Node %s, cancelling MISSION_SELECTED change", node.c_str());
                    break;
                }

                /*
                    I think that it would be smart to deallocate the clients instead of leaving them hanging. However, I can not
                    find anywhere how to deallocate a client/topic/server e.t.c in C++...
                */

                // ~lifecycleGetStateMap.at(node);
                lifecycleGetStateMap.erase(node);

                // ~lifecycleChangeStateMap.at(node);
                lifecycleChangeStateMap.erase(node);

                removeElement(nodeList, node);
            }

            /*
                Select the correct mission file based on mission, send a configuration signal and 
                load parameter file.
            */
            for (auto node: nodeList) {
                if (!changeNodeState(Transition::TRANSITION_CONFIGURE, node)) {
                    RCLCPP_INFO(get_logger(),"Failed to Configure Node %s, cancelling MISSION_SELECTED change", node.c_str());
                    break;
                }

                loadConfigurationFileToNode(node, configurationFileSelected);
            }

        return true;
    }


    bool LifecycleManagerNode::DV_Ready()
    {
        /*
        Activate every other node except Controls
            1. Send an activate transition call to every node except Controls
        */    
        bool success = false;
        for (auto node: nodeList) {
            if (node == "controls")
                continue;
            
            if (!changeNodeState(Transition::TRANSITION_ACTIVATE, node)) {
                RCLCPP_INFO(get_logger(),"Failed to activate Node %s, cancelling DV_READY change", node.c_str());
                break;
            }
        }
        success = true;
        RCLCPP_INFO(get_logger(), "DV_Ready change complete, every node except controls is ACTIVE");
        return success;
    }

    bool LifecycleManagerNode::DV_Driving()
    {
        /*
        Activate Controls Node
            1. Send an activate transition call to the Controls node.
        */
        bool success = changeNodeState(Transition::TRANSITION_ACTIVATE, "controls");

        if (!success) {
            RCLCPP_INFO(get_logger(), "Failed to activate controls node, DV_DRIVING cancelled");
        }

        return success;
    }

    void LifecycleManagerNode::loadParameters()
    {
        declare_parameter<std::vector<std::string>>("managing_node_list",
        {   "acquisition_left", "acquisition_right", "acquisition_center", "inference",
            "velocity_estimation", "slam", "saltas", "path_planning"
        });
    }
}
