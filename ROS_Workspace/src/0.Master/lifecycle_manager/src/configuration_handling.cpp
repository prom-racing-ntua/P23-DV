#include "lifecycle_manager_node.hpp"


namespace lifecycle_manager_namespace
{
    // void LifecycleManagerNode::loadConfigurationFileToNode(std::string nodeName, std::string configFile)
    // {
    //     /* :3 */
    //     std::string command = std::string("ros2 param load ") + nodeName + std::string(" ") + configFile;
    //     RCLCPP_INFO(get_logger(), "Command: %s", command.c_str());
    //     std::system(command.c_str());
    // }

    std::vector<std::string> LifecycleManagerNode::listNodeParameters(std::string nodeName)
    {
        RCLCPP_INFO(get_logger(), "Listing the parameters of node %s", nodeName.c_str());
        auto client = parameterClientMap.at(nodeName);

        auto parameters_result = client->list_parameters({}, 0);

        auto future_status = wait_for_result(parameters_result, std::chrono::seconds(2));

        if (future_status != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "Server time out while listing parameters of node %s", nodeName.c_str());
            return {};
        }

        auto result = parameters_result.get();
        printVector(result.names);

        return result.names;
    }

    bool LifecycleManagerNode::loadConfigurationFileToNode(std::string nodeName, std::string configurationYamlFilePath)
    {
        bool success;
        RCLCPP_INFO(get_logger(), "Loading configuration file to nodeName %s", nodeName.c_str());

        /* Check if node actually has any parameters. By default nodes only have 1 paramter, use_sim_time */
        if (listNodeParameters(nodeName).size() <= 1) {
            RCLCPP_INFO(get_logger(), "Node %s has no parameters to load, only use_sim_time", nodeName.c_str());
            return true;
        }

        auto client = parameterClientMap.at(nodeName);

        /* Send a config signal and wait for result */
        auto load_result = client->load_parameters(configurationYamlFilePath);
        auto future_status = wait_for_result(load_result, std::chrono::seconds(5));

        /* Check if server timed out */
        if (future_status != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "Server time out while loading parameters of node %s", nodeName.c_str());
            return false;
        }

        /* This is a vector containing set parameter results. Check each one if it was successful or not. If not, then idk :( */
        auto parameter_results = load_result.get();
        for (auto result: parameter_results) {
            bool successfullyLoaded = result.successful;
            if (!successfullyLoaded) {
                RCLCPP_ERROR(get_logger(), "Parameter failed to load with reason: %s", result.reason.c_str());
                success = false;
                break;
            }
            success = true;
        }
        
        return success;
    }

    void LifecycleManagerNode::sendResurrectionRequest(std::string nodeName)
    {
        auto request = std::make_shared<custom_msgs::srv::ResurrectNode::Request>();

        request->node_to_resurrect = nodeName;

        if (!nodeResurrectionClient->wait_for_service(std::chrono::milliseconds(heartbeatTimeoutPeriod))) {
            RCLCPP_ERROR_STREAM(get_logger(), "Service " << nodeResurrectionClient->get_service_name() << " is not available");
            return;
        }

        auto future_result = nodeResurrectionClient->async_send_request(request);
        auto future_status = wait_for_result(future_result, std::chrono::seconds(2));

        if (future_status != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "Server time out while resurrecting node %s", nodeName.c_str());
            return;
        }

        if (future_result.get()->success) {
            RCLCPP_INFO(get_logger(), "Successfully resurrected node %s, change its status ASAP", nodeName.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Couldn't resurrect  node %s", nodeName.c_str());
        }
    }
}