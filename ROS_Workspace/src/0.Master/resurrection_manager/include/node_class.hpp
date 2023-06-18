#include <string>
#include <iostream>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"


/* Currently not used. After creating the Lifecycle Manager I noticed that things are reaaaally not organized well. 
    This class could help things out. Need to refactor the manager though. Will only do if I have time available.
*/

class ManagedNode
{
    public:
        std::string nodeName;
        std::string packageName;
        pid_t nodePID;
        bool nodeError;
        bool toBeShutdown;
        uint8_t currentState;
        
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr getStateClient;
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr changeStateClient;

        explicit ManagedNode();
        ~ManagedNode();
    private:
        
};