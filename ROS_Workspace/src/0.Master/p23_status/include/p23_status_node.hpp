#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "custom_msgs/msg/mission_selection.hpp"
#include "custom_msgs/msg/autonomous_status.hpp"
#include "custom_msgs/srv/driverless_status.hpp"
#include "custom_msgs/msg/mission.hpp"
#include "custom_msgs/srv/ins_mode.hpp"
#include "custom_msgs/msg/vel_estimation.hpp"

#include "custom_msgs/msg/can_control_command.hpp"
#include "custom_msgs/msg/can_system_state.hpp"
#include "custom_msgs/msg/can_vehicle_variables.hpp"

typedef enum Mission{
    ACCELERATION = 1,
    SKIDPAD = 2,
    TRACKDRIVE = 3,
    EBS_TEST = 4,
    INSPECTION = 5,
    AUTOX = 6,
    MANUAL = 7
} Mission;

typedef enum AS_Status{
    AS_OFF = 1,
    AS_READY = 2,
    AS_DRIVING = 3,
    AS_FINISHED = 4,
    AS_EMERGENCY = 5
} AS_Status;

typedef enum DV_Status{
    STARTUP = 0,
    LV_ON = 1,
    MISSION_SELECTED = 2,
    DV_READY = 3,
    DV_DRIVING = 4,
    NODE_PROBLEM = 5
} DV_Status;

namespace p23_status_namespace
{
    class P23StatusNode : public rclcpp::Node{
    private:

        // Thewrw oti conesActual einai oi kwnoi tou sygkekrimenou lap
        // From SLAM
        uint32_t conesCountAll, conesActual;
        int currentLap;

        // From Velocity Estimation - Speed Actual
        double velocityX, velocityY, yawRate; 
        double accelerationX, accelerationY;

        // From Vectornav Service
        uint insMode;
        
        // State input from Controls - Speed Target
        double targetVelocityX, targetVelocityY, targetYawRate; 
        double motorTorqueTarget, steeringAngleTarget, brakeHydraulicTarget;

        // Current state of the car
        Mission currentMission;
        AS_Status currentASStatus;
        DV_Status currentDVStatus;
        bool missionLocked;
        bool missionFinished, standstill, pcError;

        //Node Subscriptions
        rclcpp::Subscription<custom_msgs::msg::MissionSelection>::SharedPtr canbus_mission_subscription_;
        rclcpp::Subscription<custom_msgs::msg::AutonomousStatus>::SharedPtr canbus_status_subscription_;
        rclcpp::Subscription<custom_msgs::msg::VelEstimation>::SharedPtr velocity_estimation_subscription_;
        // rclcpp::Subscription<custom_msg::msg::Slam>::SharedPtr slam_subscription_;
        // rclcpp::Subscription<custom_msg::msg::Controls>::SharedPtr controls_subscription_;

        //Node Publishers
        rclcpp::Publisher<custom_msgs::msg::CanSystemState>::SharedPtr canbus_system_state_publisher_;
        rclcpp::Publisher<custom_msgs::msg::CanVehicleVariables>::SharedPtr canbus_vehicle_variables_publisher_;
        rclcpp::Publisher<custom_msgs::msg::CanControlCommand>::SharedPtr canbus_controls_publisher_;
        
        //Clients
        rclcpp::Client<custom_msgs::srv::DriverlessStatus>::SharedPtr p23_status_client_;
        rclcpp::Client<custom_msgs::srv::InsMode>::SharedPtr ins_mode_client_;

        // Timers of node
        rclcpp::TimerBase::SharedPtr sensorCheckupTimer_;
        rclcpp::TimerBase::SharedPtr pcToVCU_slow_;
        rclcpp::TimerBase::SharedPtr pcToVCU_medium_;

        // Callback Groups
        rclcpp::CallbackGroup::SharedPtr mission_selection_group_;

        // Basic Initialization functions that are called when generating the node
        void initializeNode();
        void setServices();
        void setSubscribers();
        void setPublishers();

        // These are called when receiving message from the CANBUS node
        void updateMission(const custom_msgs::msg::MissionSelection::SharedPtr msg);
        void updateASStatus(const custom_msgs::msg::AutonomousStatus::SharedPtr msg);
        void updateVelocityInformation(const custom_msgs::msg::VelEstimation::SharedPtr msg);
        // void updateSLAMInformation(const custom_msgs::msg::kati::SharedPtr msg);
        // void updateControlsInformation(const custom_msgs::msg::kati::SharedPtr msg);

        // Callbacks that send information to the VCU
        void PCtoVCU_slow();
        void PCtoVCU_medium();
        void PCtoVCU_controls();

        //TODO: Include camera checks, imu checks etc...
        void checkSensors();
        void requestINSStatus();
        void changeDVStatus(DV_Status newStatus);

        std::vector<std::string> autonomousStatusList = {"padding","AS_OFF", "AS_READY",
            "AS_DRIVING", "AS_FINISHED", "AS_EMERGENCY"};

        std::vector<std::string> driverlessStatusList = {"STARTUP", "LV_ON",
            "MISSION_SELECTED", "DV_READY", "DV_DRIVING", "NODE_PROBLEM"};
        
        std::vector<std::string> missionList = {"padding", "ACCELERATION", "SKIDPAD",
            "TRACKDRIVE", "EBS_TEST", "INSPECTION", "AUTOX", "MANUAL"};

    public:
        explicit P23StatusNode();
    };
}