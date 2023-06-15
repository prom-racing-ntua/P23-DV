#include "lifecycle_manager_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto lifecycleManagerNode = std::make_shared<lifecycle_manager_namespace::LifecycleManagerNode>();
    executor.add_node(lifecycleManagerNode);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}