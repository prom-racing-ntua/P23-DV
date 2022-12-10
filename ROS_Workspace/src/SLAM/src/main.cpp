#include "perception_message_handler.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SLAMNode>());
  rclcpp::shutdown();
  return 0;
}