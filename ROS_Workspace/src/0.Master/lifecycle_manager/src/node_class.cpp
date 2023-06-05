#include "node_class.hpp"

ManagedNode::ManagedNode(std::string passedName, std::string passedPackageName) {
    nodeName=passedName;
    packageName=passedPackageName;
    nodeError = false;
    currentState = 0;
    toBeShutdown = false;
    uint8_t = 0;
    
    std::string getStateServiceName = nodeName + std::string("/get_state");
    std::string changeStateServiceName = nodeName + std::string("/change_state");

    getServiceClient = create_client<lifecycle_msgs::srv::GetState>(getStateServiceName);
    changeStateClient = create_client<lifecycle_msgs::srv::ChangeState>(changeStateServiceName);
}

ManagedNode::~ManagedNode() {

}