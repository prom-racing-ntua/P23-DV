#include "mpc_new.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "custom_msgs/msg/cone_struct.hpp"
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/point2_struct.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/waypoints_msg.hpp"
#include "custom_msgs/msg/tx_control_command.hpp"
#include "custom_msgs/srv/set_total_laps.hpp"
#include "node_logger.hpp"

using namespace path_planning;
using namespace std::chrono_literals;
using std::placeholders::_1;
namespace mpc_new{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class LifecycleHandler : public rclcpp_lifecycle::LifecycleNode{
        public:
            //Inside the constructor i get the declared parameters, i create publishers and subscirbers
            LifecycleHandler();
            ~LifecycleHandler();

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
            rclcpp_lifecycle::LifecyclePublisher<custom_msgs::msg::TxControlCommand>::SharedPtr dc_publisher;
            // CLIENT
            rclcpp::Client<custom_msgs::srv::SetTotalLaps>::SharedPtr total_laps_cli;
            new_MpcSolver mpc_solver;
            path_planning::ArcLengthSpline *spline;
            ::Logger waypoints_timestamp_log, pose_timestamp_log;
            double pub_time_1, pub_time_2;
            bool should_exit = false;
        protected:
            mpc_new::CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
            mpc_new::CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
            mpc_new::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
            mpc_new::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
            mpc_new::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
            mpc_new::CallbackReturn on_error(const rclcpp_lifecycle::State &state);
    };
}

