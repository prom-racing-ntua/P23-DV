#include "mpc_solver.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_msgs/msg/cone_struct.hpp"
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/point2_struct.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/waypoints_msg.hpp"
#include "custom_msgs/msg/tx_control_command.hpp"
#include "custom_msgs/srv/set_total_laps.hpp"

using namespace mpc;
using namespace path_planning;
using namespace std::chrono_literals;

namespace mpc {
class MpcHandler : public rclcpp::Node {
  public: 
    mpc::MpcSolver mpc_solver;
    MpcHandler();
    ~MpcHandler();
  private:
    void pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr pose_msg);
    void path_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr path_msg);
    void loadParameters();
    void declareParameters();
    void ReadKnownTrack();
    void setSubscribers();
    void setPublishers();
    void setClient();
    void setLogger();
    double node_freq_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<custom_msgs::msg::VelEstimation>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<custom_msgs::msg::PoseMsg>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<custom_msgs::msg::WaypointsMsg>::SharedPtr path_subscriber_;
    rclcpp::Publisher<custom_msgs::msg::TxControlCommand>::SharedPtr mpc_publisher_;
    rclcpp::Client<custom_msgs::srv::SetTotalLaps>::SharedPtr total_laps_client;
    rclcpp::TimerBase::SharedPtr mpc_clock_;
    std::ofstream data_logger;
    std::string data_logger_txt;
    size_t count_;
    std::ofstream outputFile;
    int path_flag = 0;
    double total_execution_time;
    int global_int = -1;
};
}//namespace mpc