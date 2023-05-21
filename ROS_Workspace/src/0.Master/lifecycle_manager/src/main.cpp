#include "lifecycle_manager_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto lifecycleManagerNode = std::make_shared<lifecycle_manager_namespace::LifecycleManagerNode>();
    rclcpp::spin(lifecycleManagerNode);
    rclcpp::shutdown();
    return 0;
}