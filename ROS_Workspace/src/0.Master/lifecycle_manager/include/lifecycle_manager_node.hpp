#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "custom_msgs/srv/driverless_status.hpp"
#include "custom_msgs/msg/driverless_status.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"


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

namespace lifecycle_manager_namespace
{
    
    class LifecycleManagerNode : public rclcpp::Node {
    private:
        Mission currentMission;
        AS_Status currentASStatus;
        DV_Status currentDVStatus;

        // std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

        // List of the node names that should run when having selected a mission
        std::vector<std::string> nodeList;

        std::unordered_map<std::string,rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> lifecycleGetStateMap;
        std::unordered_map<std::string,rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> lifecycleChangeStateMap;

        rclcpp::Service<custom_msgs::srv::DriverlessStatus>::SharedPtr dvStatusService_;

        void initializeServices();
        void initializeLifecycleClients(std::vector<std::string> nodeList);

        /*
            Functions for controlling the lifecycle nodes, individually. Each lifecycle node creates two major services:
            1. {node_name}/get_state
            2. {node_name}/change_state

            In the Lifecycle Manager node, we create 2 clients for every node that handle one of the above services

            The initialization happens with the above function. We create the services and put them in a dictionary with
            the node_name as the key.
        */

        unsigned int getNodeState(std::string nodeName);
        bool changeNodeState(std::uint8_t transition, std::string nodeName);
        
        bool verifyDVState();
        void loadParameters();

        // 4 Main Functions for controlling the whole pipeline of P23
        bool LV_On();
        bool Mission_Selected();
        bool DV_Ready();
        bool DV_Driving();

    public:
        explicit LifecycleManagerNode();
        void changeDVState(const std::shared_ptr<custom_msgs::srv::DriverlessStatus::Request> request,
            std::shared_ptr<custom_msgs::srv::DriverlessStatus::Response> response);
    };
}