#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include "p23_status_node.hpp"

/*
    TODO List:
    1. Write a callback to receive controls message, convert it to the needed scale and send it to the canbus node. This
        means that controls need to be complete first.
    2. Write a callback to receive SLAM message and update current Laps e.t.c. This means that SLAM must be complete first
    3. Convert data to correct scale before senting it to VCU in byte arrays.
*/

namespace p23_status_namespace
{
    P23StatusNode::P23StatusNode() : Node("p23_status")
    {
        RCLCPP_INFO(get_logger(), "Initializing P23 Status Node");

        initializeNode();
        setSubscribers();
        setServices();
        setPublishers();

        RCLCPP_INFO(get_logger(), "P23 Status Node Initialized");

        /*
            We need to get the Lifecycle Manager to an LV_ON state. This will ensure that every node is alive but unconfigured.
            The configuration will happen when the Mission Selected DV Change happens.
        */

        RCLCPP_INFO(get_logger(), "Sending LV_ON state change");
        // changeDVStatus(LV_ON);
    }

    void P23StatusNode::initializeNode()
    {
        /*
            Initialize System state variables.
        */

        conesCountAll = 0;
        conesActual = 0;
        insMode = 2;
        currentLap = -1;
        missionFinished = false;
        currentASStatus = AS_OFF;
        currentMission = MANUAL;
        missionLocked = false;
        currentDVStatus = STARTUP;

        /*
            Initialize speed/acceleration variables and such.
        */

        velocityX = 0.0;
        velocityY = 0.0;
        yawRate = 0.0;
        accelerationX = 0.0;
        accelerationY = 0.0;

        /*
            Initialize Timers. Slow VCU communication is set at 10Hz, Medium VCU communication is set at 20Hz. Controls communication
            should be done as soon as new control decisions are made (ASAP). Sensor Checkups are set at 1Hz (temporarily, we will see).
            Might add a CPU/GPU temperature checkup.
        */

        sensorCheckupTimer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&P23StatusNode::checkSensors, this));
        pcToVCU_slow_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&P23StatusNode::PCtoVCU_slow, this));
        pcToVCU_medium_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&P23StatusNode::PCtoVCU_medium, this));
    }

    void P23StatusNode::setSubscribers()
    {   
        using std::placeholders::_1;

        /*
            Add the mission_selection message handler on a mutex group. We only want to receive the first mission selected.
            The rest get ignored using a flag.
        */

        mission_selection_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        rclcpp::SubscriptionOptions options;
        options.callback_group = mission_selection_group_;

        canbus_mission_subscription_ = create_subscription<custom_msgs::msg::MissionSelection>(
            "canbus/mission_selection", 10, std::bind(&P23StatusNode::updateMission, this, _1), options);

        canbus_status_subscription_ = create_subscription<custom_msgs::msg::AutonomousStatus>(
            "canbus/as_status", 10, std::bind(&P23StatusNode::updateASStatus, this, _1));

        velocity_estimation_subscription_ = create_subscription<custom_msgs::msg::VelEstimation>(
            "velocity_estimation", 10, std::bind(&P23StatusNode::updateVelocityInformation, this, _1));
        
        // slam_subscription_ = create_subscription<custom_msgs::msg::Slam>(
        //     "", 10, std::bind(&P23StatusNode::updateSLAMInformation, this, _1));

        // controls_subscription_ = create_subscription<custom_msgs::msg::Controls>(
        //         "", 10, std::bind(&P23StatusNode::updateControlsInput, this, _1));
    }

    void P23StatusNode::setPublishers()
    {
        // Set publisher towards CANBUS writer topic
        canbus_system_state_publisher_ = create_publisher<custom_msgs::msg::CanSystemState>(
            std::string(get_name()) + std::string("/system_state"), 10);

        canbus_vehicle_variables_publisher_ = create_publisher<custom_msgs::msg::CanVehicleVariables>(
            std::string(get_name()) + std::string("/vehicle_variables"), 10);

        canbus_controls_publisher_ = create_publisher<custom_msgs::msg::CanControlCommand>(
            std::string(get_name()) + std::string("/control_command"), 10);
    }

    void P23StatusNode::setServices()
    {
        // Set a service to change DV Status and to receive IMU Mode
        p23_status_client_ = create_client<custom_msgs::srv::DriverlessStatus>("lifecycle_manager/change_driverless_status");
        ins_mode_client_ = create_client<custom_msgs::srv::InsMode>("vn_300/update_ins_mode");
    }

    void P23StatusNode::updateMission(const custom_msgs::msg::MissionSelection::SharedPtr msg)
    {   
        if (missionLocked)
            return;

        currentMission = static_cast<Mission>(msg->mission_selected);

        if (currentMission == MANUAL) {
            //Shutdown the System
        }

        // changeDVStatus might need to become blocking. Will see.
        // INS Status can (and should) remain non-blocking
        changeDVStatus(MISSION_SELECTED);
    }

    void P23StatusNode::updateASStatus(const custom_msgs::msg::AutonomousStatus::SharedPtr msg)
    {
        AS_Status statusReceived = static_cast<AS_Status>(msg->id);

        switch(statusReceived) {
            case(AS_OFF):
                // Error Handling point
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
                // Error Handling point
                RCLCPP_INFO(get_logger(), "H egw ekana malakia, h oi eletronix kapsane kouti :(");
                break;    
        }

        currentASStatus = statusReceived;
    }

    void P23StatusNode::requestINSStatus()
    {
        using namespace std::chrono_literals;

        auto request = std::make_shared<custom_msgs::srv::InsMode::Request>();
        
        while (!ins_mode_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {   
                // Error-Handling point
                return;
            }
            RCLCPP_INFO(get_logger(), "INS Service not available, waiting...");
        }

        // Send INS Mode request
        using ServiceResponseFuture = rclcpp::Client<custom_msgs::srv::InsMode>::SharedFuture;
        auto response_received_callback = [this](ServiceResponseFuture future) {
            auto result = future.get();
            insMode = result.get()->ins_mode;
            RCLCPP_INFO(get_logger(), "Received INS Mode from Vectornav %u", insMode);
        };

        auto future_result = ins_mode_client_->async_send_request(request, response_received_callback);
    }

    void P23StatusNode::changeDVStatus(DV_Status newStatus)
    {
        using namespace std::chrono_literals;

        // Create a service to communicate with the Lifecycle Manager to change the status to whatever is specified
        auto request = std::make_shared<custom_msgs::srv::DriverlessStatus::Request>();

        request->new_status.id = newStatus;
        request->new_status.label = driverlessStatusList[newStatus];

        if (newStatus == MISSION_SELECTED){
            request->mission.id = currentMission;
            request->mission.label = missionList[currentMission];
            RCLCPP_INFO(get_logger(), "Mission Selected is %s", missionList[currentMission].c_str());

        }

        if ((newStatus == DV_READY) || (newStatus == DV_DRIVING)) {
            RCLCPP_INFO(get_logger(), "Want to change to DV_READY or DV_DRIVING --> CHECK INS MODE");
            while(insMode != 2) {
                // Error-Handling point
                RCLCPP_INFO(get_logger(), "IMU Still not in mode 2. Current mode: %u",insMode);
            }   
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
                if (currentDVStatus == MISSION_SELECTED)
                    missionLocked = true;
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
}