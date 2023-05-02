#include "p23_status_node.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto statusNode = std::make_shared<p23_status_namespace::P23StatusNode>();
  executor.add_node(statusNode);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}