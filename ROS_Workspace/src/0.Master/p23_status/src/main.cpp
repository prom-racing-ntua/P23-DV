#include "p23_status_node.hpp"


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto options = rclcpp::ExecutorOptions();
    rclcpp::executors::MultiThreadedExecutor executor(options, 4);
    auto statusNode = std::make_shared<p23_status_namespace::P23StatusNode>();
    executor.add_node(statusNode->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}