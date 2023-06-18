#include "resurrection_manager.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto resurrectionManagerNode = std::make_shared<resurrection_manager_namespace::ResurrectionManagerNode>();
    rclcpp::spin(resurrectionManagerNode->get_node_base_interface());
    
    rclcpp::shutdown();
    return 0;
}