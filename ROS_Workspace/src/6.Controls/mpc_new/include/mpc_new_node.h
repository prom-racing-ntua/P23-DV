#include "mpc_new.h"
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

/*CHECK THE LIFECYCLE NODE, THIS ONE WAS USED FOR TESTS IN THE SIMULATION ONLY*/

using namespace mpc_new;
using namespace path_planning;
using namespace std::chrono_literals;
using std::placeholders::_1;

class Handler : public rclcpp::Node{
    public:
        //Inside the constructor i get the declared parameters, i create publishers and subscirbers
        Handler();
        ~Handler();

    private:
        //The main functions i will have to use
        void pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr pose_msg);
        void waypoints_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr path_msg);
        void parameterload();
        rclcpp::TimerBase::SharedPtr timer_;
        //Subscribers
        rclcpp::Subscription<custom_msgs::msg::PoseMsg>::SharedPtr pose_subscriber;
        rclcpp::Subscription<custom_msgs::msg::WaypointsMsg>::SharedPtr path_subscriber;
        //Publishers
        rclcpp::Publisher<custom_msgs::msg::TxControlCommand>::SharedPtr dc_publisher;
        new_MpcSolver mpc_solver;
        path_planning::ArcLengthSpline *spline;
};

