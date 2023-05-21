#include "lifecycle_manager_node.hpp"

/*
    The 4 Main DV States that the car should be in.
*/
using lifecycle_msgs::msg::Transition;

namespace lifecycle_manager_namespace{

    void LifecycleManagerNode::LV_On()
    {
        /*
            Nodes should be in this state:
            Acquisition: Unconfigured 
            Inference: Uconfigured
            Velocity Estimation: Unconfigured
            SLAM: Unconfigured
            Pathplanning: Unconfigured
            Controls: Unconfigured
        */
        heartbeatTimer = create_wall_timer(std::chrono::milliseconds(heartbeatTimerDuration), std::bind(&LifecycleManagerNode::verifyDVState, this));
    }

    void LifecycleManagerNode::Mission_Selected(Mission mission)
    {
        /*
            Shutdown nodes that are no longer necessary and load the parameter files
            depending on which mission you selected.
                1. Add them in a shutdown vector 
                2. Shut them down on DV Ready
                3. Delete them from the map and the node list

            Configure Nodes (changeNodeState(Transition::TRANSITION_CONFIGURE))
                1. Select the correct configuartion file based on the mission selected - DONE
                2. Send a changeNodeState transition call to every node remaining - DONE
        */
        std::string configurationFileSelected = configFolder;

        switch(mission) {
            case(ACCELERATION):
                configurationFileSelected += std::string("/acceleration_config.yaml");
                // nodesToShutdown = {"path_planning", "mpc"};
                nodesToShutdown = {"path_planning"};
                controlsNode = {"pure_pursuit"};
                break;
            case(SKIDPAD):
                configurationFileSelected += std::string("/skidpad_config.yaml");
                nodesToShutdown = {"path_planner", "pure_pursuit"};
                controlsNode = {"mpc"};
                break;
            case(TRACKDRIVE):
                configurationFileSelected += std::string("/trackdrive_config.yaml");
                nodesToShutdown = {"pure_pursuit", "path_planner"};
                controlsNode = {"mpc"};
                break;
            case(EBS_TEST):
                configurationFileSelected += std::string("/ebs_test_config.yaml");
                nodesToShutdown = {"path_planner", "mpc"};
                controlsNode = {"pure_pursuit"};
                break;
            case(INSPECTION):
                configurationFileSelected += std::string("/inspection_config.yaml");
                nodesToShutdown = {"path_planner", "mpc", "pure_pursuit", "slam", "velocity_estimation", "acquisition_left", "acquisition_right", "inference"};
                controlsNode = {"inspection"};
                break;
            case(AUTOX):
                configurationFileSelected += std::string("/autox_config.yaml");
                nodesToShutdown = {"mpc"};
                controlsNode = {"pure_pursuit"};
                break;
            case(MANUAL):
                RCLCPP_INFO(get_logger(), "Mission is in manual mode, PC will shutdown");
                break;
            }

            for (auto nodeToRemove: nodesToShutdown) {
                removeElement(nodeList, nodeToRemove);
            }

            /*
                Select the correct mission file based on mission, send a configuration signal and 
                load parameter file. This works only if all the parameters are set on the initialization of the node.
                I think that this is the best way to setup things.
            */
            
            for (auto node: nodeList) {
                loadConfigurationFileToNode(node, configurationFileSelected);
                changeNodeState(Transition::TRANSITION_CONFIGURE, node);
            }

        currentMission = mission;
    }

    void LifecycleManagerNode::DV_Ready()
    {
        /*
        Activate every other node except Controls
            1. Send an activate transition call to every node except Controls

            Mission reselection should not happen in this state, shutdown the nodes that should
            not run in this mission. If you want to reselect mission, do an LV reset.
       */
        shutdownSelectedNodes(nodesToShutdown);

        // Activate the rest (except controls)
        for (auto node: nodeList) {
            if (node == controlsNode)
                continue;
            changeNodeState(Transition::TRANSITION_ACTIVATE, node);
        }

        RCLCPP_INFO(get_logger(), "DV_Ready change complete, every node except controls is ACTIVE");
    }

    void LifecycleManagerNode::DV_Driving()
    {
        /*
        Activate Controls Node
            1. Send an activate transition call to the Controls node.
        */
        changeNodeState(Transition::TRANSITION_ACTIVATE, controlsNode);
        RCLCPP_INFO(get_logger(), "DV_Driving change complete, Good Luck Have Fun :3");
    }

    void LifecycleManagerNode::reselectMission(Mission newMission)
    {
        /*
            1. If you have already selected a mission, then you have already
            shutdown the other nodes and removed them from your node list. cleanup the
            already open nodes and reconfigure your managing node list.
            2. Purge the client/service maps and re-initialize them.
            3. Re run the Mission_Selection function.
        */

        for (auto node: nodeList) {
            changeNodeState(Transition::TRANSITION_CLEANUP, node);
        }
        get_parameter("managing_node_list", nodeList);
        initializeLifecycleClients(nodeList);
        Mission_Selected(newMission);
    }

    void LifecycleManagerNode::shutdownSelectedNodes(std::vector<std::string> nodesToShutdown) 
    {
        /*
            This functions is only to be called when going to DV Ready mode. If you shutdown a node
            then you cannot re-open it (only through some very questionable methods which might get
            implemented in the future)
        */
        uint8_t shutdownTransition = Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;

        for (auto node: nodesToShutdown) {
            changeNodeState(shutdownTransition, node);
            //TODO: Find a way to actually de-allocate the services
            lifecycleGetStateMap.erase(node);
            lifecycleChangeStateMap.erase(node);
            removeElement(nodeList, node);        
        }
    }

    void LifecycleManagerNode::verifyDVState()
    {
        for (auto node: nodeList) {
            getNodeState(node);
        }

        custom_msgs::msg::LifecycleNodeStatus msg;

        msg.clock_ok = nodeStateMap["saltas"];
        msg.inference_ok = nodeStateMap["inference"];
        msg.velocity_estimation_ok = nodeStateMap["velocity_estimation"];
        msg.slam_ok = nodeStateMap["slam"];
        msg.mpc_controls_ok = nodeStateMap["mpc"];
        msg.path_planning_ok = nodeStateMap["path_planning"];
        msg.pi_pp_controls_ok = nodeStateMap["pure_pursuit"];
        msg.camera_right_ok = nodeStateMap["acquisition_right"];
        msg.camera_left_ok = nodeStateMap["acquisition_left"];

        node_state_publisher_->publish(msg);
    }
}