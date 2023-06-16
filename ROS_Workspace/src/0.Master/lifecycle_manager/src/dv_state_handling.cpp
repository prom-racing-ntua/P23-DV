#include "lifecycle_manager_node.hpp"

/*
    The 4 Main DV States that the car should be in.
*/
using lifecycle_msgs::msg::Transition;

namespace lifecycle_manager_namespace{

    void LifecycleManagerNode::startup()
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
        heartbeatTimer = create_wall_timer(std::chrono::milliseconds(heartbeatTimerDuration), std::bind(&LifecycleManagerNode::heartbeatCheck, this));
    }

    void LifecycleManagerNode::configureNodes(p23::Mission mission)
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
            case(p23::ACCELERATION):
                configurationFileSelected += std::string("/acceleration_config.yaml");
                // nodesToShutdown = {"path_planning", "mpc"};
                // controlsNode = {"pure_pursuit"};
                break;
            case(p23::SKIDPAD):
                configurationFileSelected += std::string("/skidpad_config.yaml");
                // nodesToShutdown = {"path_planning", "pure_pursuit"};
                nodesToShutdown = {"path_planning"};
                controlsNode = {"mpc"};
                break;
            case(p23::TRACKDRIVE):
                configurationFileSelected += std::string("/trackdrive_config.yaml");
                nodesToShutdown = {"pure_pursuit", "path_planning"};
                controlsNode = {"mpc"};
                break;
            case(p23::EBS_TEST):
                configurationFileSelected += std::string("/ebs_test_config.yaml");
                nodesToShutdown = {"path_planning", "mpc"};
                controlsNode = {"pure_pursuit"};
                break;
            case(p23::INSPECTION):
                configurationFileSelected += std::string("/inspection_config.yaml");
                nodesToShutdown = {"path_planning", "mpc", "pure_pursuit", "slam", "velocity_estimation", "acquisition_left", "acquisition_right", "inference"};
                controlsNode = {"inspection"};
                break;
            case(p23::AUTOX):
                configurationFileSelected += std::string("/autox_config.yaml");
                // nodesToShutdown = {"mpc"};
                // controlsNode = {"pure_pursuit"};
                break;
            case(p23::MANUAL):
                RCLCPP_INFO(get_logger(), "Mission is in manual mode, PC will shutdown");
                return;
            case(p23::MISSION_UNLOCKED):
                RCLCPP_INFO(get_logger(), "Mission unlocked... should never get here");
                return;
            }

        for (auto nodeToRemove: nodesToShutdown) {
            removeElement(nodeList, nodeToRemove);
        }

        /*
            Select the correct mission file based on mission, send a configuration signal and 
            load parameter file. This works only if all the parameters are set on the initialization of the node.
            I think that this is the best way to setup things.
        */
        goalCounter -= nodesToShutdown.size();
        for (auto node: nodeList) { loadConfigurationFileToNode(node, configurationFileSelected); }
        for (auto node: nodeList) { changeNodeState(Transition::TRANSITION_CONFIGURE, node); }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    void LifecycleManagerNode::activateSystem()
    {
        /*
        Activate every other node except Controls
            1. Send an activate transition call to every node except Controls

            Mission reselection should not happen in this state, shutdown the nodes that should
            not run in this mission. If you want to reselect mission, do an LV reset.
       */
        shutdownSelectedNodes(nodesToShutdown, Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);

        // Activate the rest (except controls)
        for (auto node: nodeList) {
            if (node == controlsNode or node == "saltas") continue;
            changeNodeState(Transition::TRANSITION_ACTIVATE, node);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        if (std::count(nodeList.begin(), nodeList.end(), "saltas")) { changeNodeState(Transition::TRANSITION_ACTIVATE, "saltas"); }

        RCLCPP_INFO(get_logger(), "DV_Ready change complete, every node except controls is ACTIVE");
    }

    void LifecycleManagerNode::activateControls()
    {
        /*
        Activate Controls Node
            1. Send an activate transition call to the Controls node.
        */
        changeNodeState(Transition::TRANSITION_ACTIVATE, controlsNode);
        RCLCPP_INFO(get_logger(), "DV_Driving change complete, Good Luck Have Fun :3");
    }

    void LifecycleManagerNode::cleanupNodes()
    {
        /*  1. Cleanup the already open nodes and reconfigure your managing node 
            list.
            2. Purge the client/service maps and re-initialize them.
        */
    
        for (auto node: nodeList) {
            changeNodeState(Transition::TRANSITION_CLEANUP, node);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        get_parameter("managing_node_list", nodeList);
        initializeLifecycleClients(nodeList);
    }

    void LifecycleManagerNode::shutdownSelectedNodes(std::vector<std::string> nodesToShutdown, uint8_t shutdownTransition) 
    {
        /*
            This functions is only to be called when going to DV Ready mode. If you shutdown a node
            then you cannot re-open it (only through some very questionable methods which might get
            implemented in the future).
        */
       
        for (auto node: nodesToShutdown) {
            
            changeNodeState(shutdownTransition, node);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            // lifecycleGetStateMap.erase(node);
            // lifecycleChangeStateMap.erase(node);
        }
    }

    void LifecycleManagerNode::heartbeatCheck()
    {
        for (auto node: nodeList) {
            getNodeState(node);
        }

        custom_msgs::msg::LifecycleNodeStatus msg;

        msg.clock_error = nodeStateMap["saltas"];
        msg.inference_error = nodeStateMap["inference"];
        msg.velocity_estimation_error = nodeStateMap["velocity_estimation"];
        msg.slam_error = nodeStateMap["slam"];
        msg.mpc_controls_error = nodeStateMap["mpc"];
        msg.path_planning_error = nodeStateMap["path_planning"];
        msg.pi_pp_controls_error = nodeStateMap["pure_pursuit"];
        msg.camera_right_error = nodeStateMap["acquisition_right"];
        msg.camera_left_error = nodeStateMap["acquisition_left"];

        node_state_publisher_->publish(msg);
    }
}