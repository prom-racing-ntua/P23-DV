#include "mpc_solver.h"
#include "custom_msgs/msg/cone_struct.hpp"
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/point2_struct.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/waypoints_msg.hpp"
#include "custom_msgs/msg/tx_control_command.hpp"


/* ROS Releated Imports */
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

using namespace mpc;
using namespace path_planning;
using namespace std::chrono_literals;

namespace mpc {
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  class LifecycleMpcHandler : public rclcpp_lifecycle::LifecycleNode {
    public: 
      mpc::MpcSolver mpc_solver;
      LifecycleMpcHandler();
      ~LifecycleMpcHandler();
    private:
      void velocity_callback(const custom_msgs::msg::VelEstimation::SharedPtr vel_msg);
      void pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr pose_msg);
      void path_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr path_msg);
      void mpc_callback();
      void loadParameters();
      void ReadKnownTrack();
      void setSubscribers();
      
      int node_freq_;
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Subscription<custom_msgs::msg::VelEstimation>::SharedPtr velocity_subscriber_;
      rclcpp::Subscription<custom_msgs::msg::PoseMsg>::SharedPtr pose_subscriber_;
      rclcpp::Subscription<custom_msgs::msg::WaypointsMsg>::SharedPtr path_subscriber_;
      rclcpp_lifecycle::LifecyclePublisher<custom_msgs::msg::TxControlCommand>::SharedPtr mpc_publisher_;
      rclcpp::TimerBase::SharedPtr mpc_clock_;
      size_t count_;
      std::ofstream outputFile;
      int path_flag = 0;
      double total_execution_time;
      int global_int = -1;
    protected:
      mpc::CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
      mpc::CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
      mpc::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
      mpc::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
      mpc::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
      mpc::CallbackReturn on_error(const rclcpp_lifecycle::State &state);
  };
}//namespace mpc