#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "custom_msgs/msg/point2_struct.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/waypoints_msg.hpp"
#include "custom_msgs/msg/tx_control_command.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"

#include "pid_pp_module.hpp"

using namespace std::chrono_literals;
using namespace pid_pp;
using std::placeholders::_1;

class PID_PP_Node : public rclcpp::Node
{
public:
    PID_PP_Node();
    ~PID_PP_Node();

private:
    // SUBSCRIBERS
    rclcpp::Subscription<custom_msgs::msg::WaypointsMsg>::SharedPtr sub_waypoints;
    rclcpp::Subscription<custom_msgs::msg::VelEstimation>::SharedPtr sub_velocity;
    rclcpp::Subscription<custom_msgs::msg::PoseMsg>::SharedPtr sub_pose;
    // PUBLISHER
    rclcpp::Publisher<custom_msgs::msg::TxControlCommand>::SharedPtr pub_actuators;

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

    // VARIABLES
    double v_x, v_y, r, a_x, a_y;
    double total_execution_time;
    double emergency_threshold;
    double safety_factor;
    double max_speed, spline_res_per_meter;
    double safe_speed_to_break;
    int laps_to_do;

    bool is_end;

    //Multithreading shit
    pthread_spinlock_t global_lock_;
    rclcpp::CallbackGroup::SharedPtr callback_group_waypoints;
    rclcpp::CallbackGroup::SharedPtr callback_group_pose;

    // BEGINNING SAFETY CHECKS
    bool has_run_waypoints; // needed to run pose_callback
    int count_wp;
};

/*
CONSTRUCTOR:
    ->load parameters
    ->initialize controllers

WAYPOINTS CALLBACK:
    ->save midpoints
    ->fit spline
    ->create velocity profile
    ... potentially ...
    -> send off initial commands

VELOCITY/POSE CALLBACK:
    ->project onto spline
    ->calculate crosstrack error
    ->find in velocity profile the u_reference
    ->calculate pid output and pp output
    ->publish
*/