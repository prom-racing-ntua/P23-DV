#include "lifecycle_manager_node.hpp"

namespace lifecycle_manager
{
    pid_t LifecycleManagerNode::launchNode(std::string nodeName, std::string packageName, std::string runCommand)
    {
        pid_t nodePID;
        nodePID = fork();

        if (nodePID < 0) {
            RCLCPP_ERROR(get_logger(), "Could not fork - stopping launch");
        }
        else if (nodePID == 0) {
            /* Child Code - The new process will block in this command. This is EXTREMELLY CURSED code and should not
                be used. This is only a proof of concept. This works extremelly well though, especially since I can
                send a SIGINT signal and cleanup the node with extreme ease. You can save the PIDs in a map with 
                key=nodename, value=pid and control the processes as subprocesses through the lifecycle manager. 
                Maybe if things get extremelly bleak... 
            */
            std::string command = std::string("ros2 run ") + packageName + std::string(" ") + runCommand;
            std::system(command.c_str());

            /* Should never get here...*/
            exit(1);
        }
        nodePIDMap[nodeName] = nodePID;
        return nodePID;
    }

    /* This is called when a heartbeat fails (twice). */
    void LifecycleManagerNode::resurrectNode(std::string nodeName)
    {
        /*
            1. Relaunch the node
            2. Pass the configuration file
            3. Configure it -> Activate it.
        */
        pid_t launchedNodePID = launchNode(nodeName, "8a doume", "8a doume...");
    }
}