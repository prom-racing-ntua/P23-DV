#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "custom_msgs/msg/mission_selection.hpp"
#include "custom_msgs/msg/autonomous_status.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/tx_system_state.hpp"
#include "custom_msgs/msg/tx_vehicle_variables.hpp"
#include "custom_msgs/msg/lifecycle_node_status.hpp"

#include "custom_msgs/srv/driverless_transition.hpp"
#include "custom_msgs/srv/ins_mode.hpp"

#include "p23_common.h"


namespace p23_status_namespace
{
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
    bool missionFinished, standstill, pcError;

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
    rclcpp::Client<custom_msgs::srv::DriverlessTransition>::SharedPtr p23_status_client_;
    rclcpp::Client<custom_msgs::srv::InsMode>::SharedPtr ins_mode_client_;
    rclcpp::Client<custom_msgs::srv::InsMode>::SharedPtr vectornav_heartbeat_client_;

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
    void changeDVStatus(p23::DV_Transitions newStatus);

public:
    explicit P23StatusNode();
};
} // p23_status_namespace