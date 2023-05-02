#include "lifecycle_manager_node.hpp"

/*
    The 4 Main DV States that the car should be in.
*/
using lifecycle_msgs::msg::Transition;

namespace lifecycle_manager_namespace{

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
        verifyDVState();
        heartbeatTimer = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&LifecycleManagerNode::verifyDVState, this));
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

        switch(mission) {
            case(ACCELERATION):
                configurationFileSelected += std::string("/acceleration_config.yaml");
                nodesToShutdown = {};
                break;
            case(SKIDPAD):
                configurationFileSelected += std::string("/skidpad_config.yaml");
                nodesToShutdown = {"path_planner"};
                break;
            case(TRACKDRIVE):
                configurationFileSelected += std::string("/trackdrive_config.yaml");
                nodesToShutdown = {};
                break;
            case(EBS_TEST):
                configurationFileSelected += std::string("/ebs_test_config.yaml");
                nodesToShutdown = {"path_planner"};
                break;
            case(INSPECTION):
                configurationFileSelected += std::string("/inspection_config.yaml");
                nodesToShutdown = {"path_planner"};
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

            RCLCPP_INFO(get_logger(), "Configuration File Selected: %s", configurationFileSelected.c_str());  
            RCLCPP_INFO(get_logger(), "Before Remove Element");

            printVector(nodeList);

            if (!nodesToShutdown.empty()) {
                for (auto node: nodesToShutdown) {
                    bool success = true;
                    changeNodeState(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, node);

                    if (!success) {
                        RCLCPP_INFO(get_logger(),"Failed to shutdown Node %s, cancelling MISSION_SELECTED change", node.c_str());
                        break;
                    }

                    // delete lifecycleGetStateMap.at(node);
                    lifecycleGetStateMap.erase(node);
                    // delete lifecycleChangeStateMap.at(node);
                    lifecycleChangeStateMap.erase(node);
                    removeElement(nodeList, node);        
                }
            }

            RCLCPP_INFO(get_logger(), "After Remove Element");
            printVector(nodeList);
            /*
                Select the correct mission file based on mission, send a configuration signal and 
                load parameter file.
            */
            for (auto node: nodeList) {
                /*
                    This works only if all the parameters are set on the initialization of the node.
                    I think that this is the best way to setup things.
                */
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
            changeNodeState(Transition::TRANSITION_ACTIVATE, node);
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
        changeNodeState(Transition::TRANSITION_ACTIVATE, "controls");

        return true;
    }
}