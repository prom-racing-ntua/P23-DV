#include <memory>

#include "rclcpp/rclcpp.hpp"
using std::placeholders::_1;

class SLAMHandler : public rclcpp::Node
{
  public:
    SLAMHandler()
    : Node("SLAM_Main_Node")
    {
      subscription_ = this->create_subscription<perception_msgs::msg::Perception2Slam>(
      "perception2slam_topic", 10, std::bind(&SLAMHandler::perceptionMessageCallback, this, _1));
    }

  private:
    void perceptionMessageCallback(const perception_msgs::msg::String::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "Stis pou mono pespi");
      RCLCPP_INFO(this->get_logger(), "Array: %d", 12);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};



