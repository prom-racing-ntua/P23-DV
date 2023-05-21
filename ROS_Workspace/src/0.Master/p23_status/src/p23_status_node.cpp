#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include "p23_status_node.hpp"

/*
    TODO List:
    1. Write a callback to receive SLAM message and update current Laps e.t.c. This means that SLAM must be complete first
*/

namespace p23_status_namespace
{
    P23StatusNode::P23StatusNode() : Node("p23_status")
    {
        RCLCPP_INFO(get_logger(), "Initializing P23 Status Node");

        initializeNode();
        loadParameters();
        setSubscribers();
        setServices();
        setPublishers();

        get_parameter("managing_node_list", nodeList);
        get_parameter("sensor_checkup_frequency", sensorCheckupFrequency);
        get_parameter("system_state_publish_frequency", systemStateFrequency);

        systemStatePeriod = 1000/systemStateFrequency;
        sensorCheckupPeriod = 1000/sensorCheckupFrequency;

        RCLCPP_INFO(get_logger(), "P23 Status Node Initialized");

        /*
            We need to get the Lifecycle Manager to an LV_ON state. This will ensure that every node is alive but unconfigured.
            The configuration will happen when the Mission Selected DV Change happens.
        */
        RCLCPP_INFO(get_logger(), "Sending LV_ON state change");
        changeDVStatus(LV_ON);
    }

    void P23StatusNode::initializeNode()
    {
        /*
            Initialize System state variables.
        */ 
        conesCountAll = 0;
        conesActual = 0;
        insMode = 0;
        currentLap = -1;
        maxLaps = 0;
        pcError = 0;

        missionFinished = false;
        currentASStatus = AS_OFF;
        currentMission = MANUAL;
        currentDVStatus = STARTUP;
        
        sensorCheckupTimer_ = create_wall_timer(std::chrono::milliseconds(sensorCheckupPeriod), std::bind(&P23StatusNode::checkSensors, this));
        systemStateTimer_ = create_wall_timer(std::chrono::milliseconds(systemStatePeriod), std::bind(&P23StatusNode::sendSystemState, this));

        for (std::string node: nodeList) {
            nodeStatusMap[node] = false;
        }
    }

    void P23StatusNode::setSubscribers()
    {   
        using std::placeholders::_1;

        mission_selection_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options;
        options.callback_group = mission_selection_group_;

        canbus_mission_subscription_ = create_subscription<custom_msgs::msg::MissionSelection>(
            "canbus/mission_selection", 10, std::bind(&P23StatusNode::updateMission, this, _1), options);

        // Dimitris: AS State will probably be in the same message along with other variables
        canbus_status_subscription_ = create_subscription<custom_msgs::msg::AutonomousStatus>(
            "canbus/as_status", 10, std::bind(&P23StatusNode::updateASStatus, this, _1));
        
        slam_subscription_ = create_subscription<custom_msgs::msg::PoseMsg>(
            "pose", 10, std::bind(&P23StatusNode::updateSLAMInformation, this, _1));
        
        lifecycle_node_status_subscription_ = create_subscription<custom_msgs::msg::LifecycleNodeStatus>(
            "lifecycle_manager/lifecycle_node_status", 10, std::bind(&P23StatusNode::receiveNodeStatus, this, _1));
    }

    void P23StatusNode::setPublishers()
    {
        // Set publisher towards CANBUS writer topic
        canbus_system_state_publisher_ = create_publisher<custom_msgs::msg::TxSystemState>(
            std::string(get_name()) + std::string("/system_state"), 10);
    }

    void P23StatusNode::setServices()
    {
        // Set a service to change DV Status and to receive IMU Mode
        p23_status_client_ = create_client<custom_msgs::srv::DriverlessStatus>("lifecycle_manager/change_driverless_status");
        ins_mode_client_ = create_client<custom_msgs::srv::InsMode>("vn_300/update_ins_mode");
        vectornav_heartbeat_client_ = create_client<custom_msgs::srv::VectornavHeartbeat>("vn_200/vectornav_heartbeat");
    }

    void P23StatusNode::updateMission(const custom_msgs::msg::MissionSelection::SharedPtr msg)
    {
        currentMission = static_cast<Mission>(msg->mission_selected);

        switch (currentMission) {
            case(ACCELERATION):
                maxLaps = 1;
                break;
            case(SKIDPAD):
                maxLaps = 0;
                break;
            case(TRACKDRIVE):
                maxLaps = 0;
                break;
            case(EBS_TEST):
                maxLaps = 0;
                break;
            case(INSPECTION):
                maxLaps = 0;
                break;
            case(AUTOX):
                maxLaps = 0;
                break;
            case(MANUAL):
                // The PC will shutdown so no one cares what happens here...
                std::system("shutdown -n 0.5");
                break;
            default:
                maxLaps = 100;
                break;
        }
        changeDVStatus(MISSION_SELECTED);
    }

    void P23StatusNode::updateASStatus(const custom_msgs::msg::AutonomousStatus::SharedPtr msg)
    {
        AS_Status statusReceived = static_cast<AS_Status>(msg->id);

        switch(statusReceived) {
            case(AS_OFF):
                RCLCPP_INFO(get_logger(), "Den paizei re...");
                break;
            case(AS_READY):
                RCLCPP_INFO(get_logger(), "Received AS_READY change. Go to DV_READY mode if you can");
                changeDVStatus(DV_READY);
                break;
            case(AS_DRIVING):
                RCLCPP_INFO(get_logger(), "Received AS_DRIVING change. Go to DV_DRIVING mode if you can");
                changeDVStatus(DV_DRIVING);
                break;
            case(AS_FINISHED):
                RCLCPP_INFO(get_logger(), "Den paizei re... egw ta stelnw afta");
                break;
            case(AS_EMERGENCY):
                /* Send a deactivation signal to the nodes.*/
                RCLCPP_INFO(get_logger(), "H egw ekana malakia, h oi eletronix kapsane kouti :(");
                break;    
        }
        currentASStatus = statusReceived;
    }

    void P23StatusNode::changeDVStatus(DV_Status newStatus)
    {
        using namespace std::chrono_literals;

        // Create a service to communicate with the Lifecycle Manager to change the status to whatever is specified
        auto request = std::make_shared<custom_msgs::srv::DriverlessStatus::Request>();

        request->new_status.id = newStatus;
        request->new_status.label = driverlessStatusList[newStatus];

        if (newStatus == MISSION_SELECTED) {
            request->mission.id = currentMission;
            request->mission.label = missionList[currentMission];
            RCLCPP_INFO(get_logger(), "Mission Selected is %s", missionList[currentMission].c_str());
        }

        RCLCPP_INFO(get_logger(), "Sending a %s request to Lifecycle Manager", driverlessStatusList[newStatus].c_str());

        while (!p23_status_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {   
                // Error-Handling point
                return;
            }
            RCLCPP_INFO(get_logger(), "Lifecycle Manager Service not available, waiting...");
        }

        // Send request to Lifecycle Manager
        using ServiceResponseFuture = rclcpp::Client<custom_msgs::srv::DriverlessStatus>::SharedFuture;
        auto response_received_callback = [this, newStatus](ServiceResponseFuture future) {

            auto result = future.get();
            RCLCPP_INFO(get_logger(), "Received result from Lifecycle Manager Service for %s request", driverlessStatusList[newStatus].c_str());

            bool success = result.get()->success;

            if (success)
            {
                RCLCPP_INFO(get_logger(), "Driverless Status successfully changed to: %s", driverlessStatusList[newStatus].c_str());
                currentDVStatus = newStatus;
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Could not change Driverless Status to: %s", driverlessStatusList[newStatus].c_str());
                // Error-Handling point
            }
        };

        auto future_result = p23_status_client_->async_send_request(request, response_received_callback);
        RCLCPP_INFO(get_logger(), "%s request sent to Lifecycle Manager, waiting for response...", driverlessStatusList[newStatus].c_str());
    }

    void P23StatusNode::loadParameters()
    {
        declare_parameter<std::vector<std::string>>("managing_node_list",
        {   "acquisition_left", "acquisition_right", "acquisition_center", "inference",
            "velocity_estimation", "slam", "saltas", "path_planning"
        });
        declare_parameter<int>("system_state_publish_frequency", 10);
        declare_parameter<float>("sensor_checkup_frequency", 1);
    }
}