#include "lifecycle_manager_node.hpp"


namespace lifecycle_manager_namespace
{
    void LifecycleManagerNode::loadConfigurationFileToNode(std::string nodeName, std::string configFile)
    {
        /* :3 */
        std::string command = std::string("ros2 param load /") + nodeName + std::string(" ") + configFile;
        std::system(command.c_str());
    }

    // void LifecycleManagerNode::loadConfigurationFileToNode(std::string nodeName, std::string configFile)
    // {
    //     // RCLCPP_INFO(get_logger(), "Loading %s configuration", nodeName.c_str());

    //     // auto client = parameterClientMap.at(nodeName);

    //     // using ListParametersResponseFuture = std::shared_future<rcl_interfaces::msg::ListParametersResult>;

    //     // auto list_parameters_callback = [this, nodeName, client, configFile](ListParametersResponseFuture future) {
    //     //     auto result = future.get();
    //     //     for (auto name: result.names) {
    //     //         RCLCPP_INFO(get_logger(), "Name: %s", name.c_str());
    //     //     }
    //     //     if (result.names.size() == 1) {
    //     //         RCLCPP_INFO(get_logger(), "Node %s has no parameters to load.", nodeName.c_str());
    //     //     }
    //     //     else {
    //     //         RCLCPP_INFO(get_logger(), "Number of parameters: %ld", result.names.size());
    //     //         client->load_parameters(configFile);
    //     //     }
    //     // };
    //     // RCLCPP_INFO(get_logger(), "Calling list parameters for %s", nodeName.c_str());
    //     // auto parameters = client->list_parameters({}, 0, list_parameters_callback);
    // }
}