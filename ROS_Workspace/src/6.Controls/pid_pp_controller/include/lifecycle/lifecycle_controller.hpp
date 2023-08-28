#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_msgs/msg/point2_struct.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/waypoints_msg.hpp"
#include "custom_msgs/msg/tx_control_command.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"
#include "custom_msgs/srv/set_total_laps.hpp"

#include "pid_pp_module.hpp"

using namespace std::chrono_literals;
using namespace pid_pp;
using std::placeholders::_1;

namespace pid_pp{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecyclePID_PP_Node : public rclcpp_lifecycle::LifecycleNode
{
public:
    LifecyclePID_PP_Node();
    ~LifecyclePID_PP_Node();

private:
    // SUBSCRIBERS
    rclcpp::Subscription<custom_msgs::msg::WaypointsMsg>::SharedPtr sub_waypoints;
    rclcpp::Subscription<custom_msgs::msg::VelEstimation>::SharedPtr sub_velocity;
    rclcpp::Subscription<custom_msgs::msg::PoseMsg>::SharedPtr sub_pose;
    // PUBLISHER
    rclcpp_lifecycle::LifecyclePublisher<custom_msgs::msg::TxControlCommand>::SharedPtr pub_actuators;
    rclcpp_lifecycle::LifecyclePublisher<custom_msgs::msg::Point2Struct>::SharedPtr pub_target;
    // CLIENT
    rclcpp::Client<custom_msgs::srv::SetTotalLaps>::SharedPtr total_laps_cli;

    // OBJECTS
    VelocityProfile *profile;
    path_planning::ArcLengthSpline *spline;

    Model model;
    PurePursuit pp_controller;
    PID pid_controller;

    // CALLBACKS
    void waypoints_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr msg);
    void pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr msg);

    // OTHER METHODS
    void parameter_load();
    Point find_target(/*TBD*/);
    void known_map_substitute(int lap, int total_laps);

    // VARIABLES
    double v_x, v_y, r, a_x, a_y;
    double total_execution_time;
    double emergency_threshold;
    double safety_factor;
    double max_speed, spline_res_per_meter;
    double safe_speed_to_break, braking_distance;
    double last_steering, last_torque;
    int laps_to_do;
    int prev_lap;
    string discipline, midpoints;
    std::ifstream mids;

    bool is_end;
    bool switch_br;
    std::ofstream log;

    //Multithreading shit -- Ntroph :(
    rclcpp::CallbackGroup::SharedPtr mutexCallbackGroup;

    // BEGINNING SAFETY CHECKS
    bool has_run_waypoints; // needed to run pose_callback
    int count_wp;
protected:
    pid_pp::CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
    pid_pp::CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
    pid_pp::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
    pid_pp::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
    pid_pp::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
    pid_pp::CallbackReturn on_error(const rclcpp_lifecycle::State &state);
};
}