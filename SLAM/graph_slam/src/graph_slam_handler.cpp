#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"
using std::placeholders::_1;

class GraphSLAM_handler : public rclcpp::Node
{
  public:
    GraphSLAM_handler() : Node("graph_slam")
    {
      odom_subscription = this->create_subscription<custom_msgs::msg::VelEstimation>("state_estimation/vel_est", 10, std::bind(&GraphSLAM_handler::new_odom, this, _1));
    }

  private:
    void new_odom(const custom_msgs::msg::VelEstimation::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->counter);
    }
    rclcpp::Subscription<custom_msgs::msg::VelEstimation>::SharedPtr odom_subscription;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraphSLAM_handler>());
  rclcpp::shutdown();
  return 0;
}

