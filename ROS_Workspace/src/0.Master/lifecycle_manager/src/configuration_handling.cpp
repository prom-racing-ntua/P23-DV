#include "lifecycle_manager_node.hpp"


namespace lifecycle_manager_namespace
{
    void LifecycleManagerNode::loadConfigurationFileToNode(std::string nodeName, std::string configFile)
    {
        /* :3 */
        std::string command = std::string("ros2 param load ") + nodeName + std::string(" ") + configFile;
        RCLCPP_INFO(get_logger(), "Command: %s", command.c_str());
        std::system(command.c_str());
    }
}