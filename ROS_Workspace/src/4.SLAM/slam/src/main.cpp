#include "slam_handler.h"
#include "slam.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto slam_node = std::make_shared<slam_namespace::SLAM_handler>();
  executor.add_node(slam_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}