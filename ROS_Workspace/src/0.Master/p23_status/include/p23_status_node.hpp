#ifndef P23_STATUS_NODE_HPP
#define P23_STATUS_NODE_HPP

/* C/C++ Imports */
#include <memory>
#include <chrono>
#include <atomic>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <string>
#include <cstring>
#include <thread>
#include <unistd.h>

/* ROS2 Libraries */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/utilities.hpp"

/* Messages */
#include "custom_msgs/msg/mission_selection.hpp"
#include "custom_msgs/msg/autonomous_status.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/tx_system_state.hpp"
#include "custom_msgs/msg/tx_vehicle_variables.hpp"
#include "custom_msgs/msg/lifecycle_node_status.hpp"

#include "custom_msgs/srv/driverless_transition.hpp"
#include "custom_msgs/srv/ins_mode.hpp"
#include "custom_msgs/srv/set_total_laps.hpp"

#include "custom_msgs/action/driverless_transition.hpp"

#include "p23_common.h"


template<typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait)
{
    auto end = std::chrono::steady_clock::now() + time_to_wait;
    std::chrono::milliseconds wait_period(100);
    std::future_status status = std::future_status::timeout;
    do {
        auto now = std::chrono::steady_clock::now();
        auto time_left = end - now;
        if (time_left <= std::chrono::seconds(0)) {break;}
        status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
    } while (rclcpp::ok() && status != std::future_status::ready);
    return status;
}

namespace p23_status_namespace
{

using Transition = custom_msgs::action::DriverlessTransition;
using GoalHandle = rclcpp_action::ClientGoalHandle<Transition>;

class P23StatusNode : public rclcpp::Node {
private:
    // Thewrw oti conesActual einai oi kwnoi tou sygkekrimenou lap
    // From SLAM
    uint16_t conesCountAll;
    uint8_t conesActual;
    int currentLap;
    int maxLaps;

    // From Vectornav Service
    uint insMode;
    bool nodesReady;

    int sensorCheckupFrequency, systemStateFrequency;

    // Current state of the car
    p23::Mission currentMission;
    p23::AS_Status currentAsStatus;
    p23::DV_Status currentDvStatus;
    bool missionLocked, missionFinished, pcError;

    // Standstill variables
    bool standstill;
    double standstill_time;

    /* Map of all the current statuses of the nodes. Might not keep it and just
        send a message directly from the lifecycle manager, will see */
    std::unordered_map<std::string, bool> nodeStatusMap;
    std::vector<std::string> nodeList;

    //Node Subscriptions
    rclcpp::Subscription<custom_msgs::msg::MissionSelection>::SharedPtr canbus_mission_subscription_;
    rclcpp::Subscription<custom_msgs::msg::AutonomousStatus>::SharedPtr canbus_status_subscription_;
    rclcpp::Subscription<custom_msgs::msg::LocalMapMsg>::SharedPtr slam_subscription_;
    rclcpp::Subscription<custom_msgs::msg::LifecycleNodeStatus>::SharedPtr lifecycle_node_status_subscription_;

    //Node Publishers
    rclcpp::Publisher<custom_msgs::msg::TxSystemState>::SharedPtr canbus_system_state_publisher_;

    //Clients
    rclcpp::Client<custom_msgs::srv::InsMode>::SharedPtr ins_mode_client_;
    rclcpp::Client<custom_msgs::srv::InsMode>::SharedPtr vectornav_heartbeat_client_;

    // Max Laps Service
    rclcpp::Service<custom_msgs::srv::SetTotalLaps>::SharedPtr total_laps_server_;

    //Actions
    rclcpp_action::Client<Transition>::SharedPtr dv_transition_client_;
    void TransitionResponse(GoalHandle::SharedPtr goal_handle);
    void TransitionFeedback(GoalHandle::SharedPtr, const std::shared_ptr<const Transition::Feedback> feedback);
    void TransitionResult(const GoalHandle::WrappedResult &result, const p23::DV_Status& transtion_to);

    // Timers of node
    rclcpp::TimerBase::SharedPtr sensorCheckupTimer_;
    rclcpp::TimerBase::SharedPtr systemStateTimer_;

    // Callback Groups
    rclcpp::CallbackGroup::SharedPtr mission_selection_group_;

    // Basic Initialization functions that are called when generating the node
    void initializeNode();
    void setServices();
    void setSubscribers();
    void setPublishers();
    void loadParameters();

    /*
        These are called when receiving message from the CANBUS node
        As well as from the other nodes whose information we need to send
        to the data logger.
    */

    void updateMission(const custom_msgs::msg::MissionSelection::SharedPtr msg);
    void updateASStatus(const custom_msgs::msg::AutonomousStatus::SharedPtr msg);
    void updateSLAMInformation(const custom_msgs::msg::LocalMapMsg::SharedPtr msg);
    void receiveNodeStatus(const custom_msgs::msg::LifecycleNodeStatus::SharedPtr msg);

    // Callbacks that send information to the VCU
    void sendSystemState();

    /* Sensor Checkup function */
    void checkVectornav();

    /* DV Status Client function */
    bool changeDVStatus(p23::DV_Transitions newStatus);

    /* Callback to set total laps*/
    void setTotalLaps(const std::shared_ptr<custom_msgs::srv::SetTotalLaps::Request> request,
          std::shared_ptr<custom_msgs::srv::SetTotalLaps::Response> response);

public:
    explicit P23StatusNode();
};
} // p23_status_namespace

#endif