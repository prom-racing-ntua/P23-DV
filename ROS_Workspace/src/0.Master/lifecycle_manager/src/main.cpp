#include "lifecycle_manager_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto options = rclcpp::ExecutorOptions();
    rclcpp::executors::MultiThreadedExecutor executor(options, 4);
    auto lifecycleManagerNode = std::make_shared<lifecycle_manager_namespace::LifecycleManagerNode>();
    executor.add_node(lifecycleManagerNode->get_node_base_interface());
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}