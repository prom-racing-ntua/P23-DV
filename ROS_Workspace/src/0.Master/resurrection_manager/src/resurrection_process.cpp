#include "resurrection_manager.hpp"

namespace resurrection_manager_namespace
{
    pid_t ResurrectionManagerNode::resurrectNode(std::string nodeName)
    {
        pid_t resurrectedPID;
        std::string packageName, executableName;

        packageName = packageMap.at(nodeName);
        executableName = executableMap.at(nodeName);

        resurrectedPID = fork();

        if (resurrectedPID < 0) {
            RCLCPP_ERROR(get_logger(), "Could not fork - cancel resurrection");
        }
        else if (resurrectedPID == 0) {
            /* Child Code - The new process will block in this command. This is EXTREMELLY CURSED code and should not
                be used. This is only a proof of concept. This works extremelly well though, especially since I can
                send a SIGINT signal and cleanup the node with extreme ease. You can save the PIDs in a map with 
                key=nodename, value=pid and control the processes as subprocesses through the lifecycle manager. 
                Maybe if things get extremelly bleak... 
            */
            std::string command = std::string("ros2 run ") + packageName + std::string(" ") + executableName;
            std::system(command.c_str());

            /* Should never get here...*/
            exit(1);
        }
        return resurrectedPID;
    }

    void ResurrectionManagerNode::sendSignalToNode(std::string nodeName, uint8_t signal)
    {
        /* Get the PID of the node that you want to send a signal to and then send a signal through kill() */
        pid_t nodePID = nodePIDMap.at(nodeName);
        kill(nodePID, SIGINT);
        return;
    }

    void ResurrectionManagerNode::handleResurrection(const std::shared_ptr<custom_msgs::srv::ResurrectNode::Request> request,
        std::shared_ptr<custom_msgs::srv::ResurrectNode::Response> response)
    {   
        std::string nodeToResurrect = request->node_to_resurrect;

        pid_t resurrectionPID = resurrectNode(nodeToResurrect);
        if (resurrectionPID < 0) {
            RCLCPP_ERROR(get_logger(), "Resurrection of node %s has failed, cancel resurrection and shutdown :(", nodeToResurrect.c_str());
            response->success = false;
        }
        
        nodePIDMap[nodeToResurrect] = resurrectionPID;
        response->success = true;
    }

    void ResurrectionManagerNode::multicastOrder(const std::shared_ptr<custom_msgs::srv::ResurrectOrder::Request> request,
        std::shared_ptr<custom_msgs::srv::ResurrectOrder::Response> response)
    {
        /* For now, we only support the shutting down of nodes, meaning that when we receive a multicast order, we just shutdown the nodes.
            If we feel very safe about our DV, we can implement a stop, a continue and a shutdown signal. Makari na ftasoume mexri ekei. */

        /*Sending a signal is not reliable (but is our only method here) and we do not receive a response. So we just straight up send a successful response 
            even though we can not confirm it.  
            1. Iterate through map 
            2. Send a SIGINT signal to every process inside the map */
        for (auto const& pair: nodePIDMap)
        {   
            std::string nodeToOrder = pair.first;
            sendSignalToNode(nodeToOrder, 0);
        }
        response->success = true;
    }
}