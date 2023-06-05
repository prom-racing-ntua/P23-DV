#include <rmw/qos_profiles.h>
#include "p23_status_node.hpp"

/*
    TODO List:
    1. Write a callback to receive SLAM message and update current Laps e.t.c. This means that SLAM must be complete first

    General Notes:
    - What happens if lifecycle is in the ON_MISSION_LOCKED transition and before it ends we receive a MISSION_UNLOCKED (we receive it only once)?
    - How to do the stopping manuever when mission finished and then shut down the nodes?
*/

namespace p23_status_namespace
{
P23StatusNode::P23StatusNode() : Node("p23_status") {
    loadParameters();

    initializeNode();
    setSubscribers();
    setServices();
    setPublishers();

    RCLCPP_INFO(get_logger(), "P23 Status Node Initialized");

    /*
        We need to get the Lifecycle Manager to an LV_ON state. This will ensure that every node is alive but unconfigured.
        The configuration will happen when the Mission Selected DV Change happens.
    */
    RCLCPP_INFO(get_logger(), "Sending Startup signal to Lifecycle Manager");
    changeDVStatus(p23::ON_STARTUP);
}


void P23StatusNode::initializeNode() {
    // Initialize System state variables.
    conesCountAll = 0;
    conesActual = 0;

    // Variables to check for DV_READY
    insMode = 0;
    nodesReady = false;

    currentLap = -1;
    // Initialize to 255 because current lap received might be 0
    maxLaps = 255;
    pcError = false;

    missionLocked = false;
    missionFinished = false;
    currentAsStatus = p23::AS_OFF;
    currentMission = p23::MISSION_UNLOCKED;
    currentDvStatus = p23::STARTUP;

    for (std::string node : nodeList)
    {
        nodeStatusMap[node] = false;
    }

    // The period calculation has integer division problems, but chrono receives int anyway (for more accuracy change to nanoseconds)
    sensorCheckupTimer_ = create_wall_timer(std::chrono::milliseconds(1000 / sensorCheckupFrequency), std::bind(&P23StatusNode::checkVectornav, this));
    systemStateTimer_ = create_wall_timer(std::chrono::milliseconds(1000 / systemStateFrequency), std::bind(&P23StatusNode::sendSystemState, this));
}

void P23StatusNode::setSubscribers() {
    using std::placeholders::_1;

    mission_selection_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = mission_selection_group_;

    canbus_mission_subscription_ = create_subscription<custom_msgs::msg::MissionSelection>(
        "/canbus/mission_selection", 10, std::bind(&P23StatusNode::updateMission, this, _1), options);

    canbus_status_subscription_ = create_subscription<custom_msgs::msg::AutonomousStatus>(
        "/canbus/as_status", 10, std::bind(&P23StatusNode::updateASStatus, this, _1));

    slam_subscription_ = create_subscription<custom_msgs::msg::LocalMapMsg>(
        "local_map", 10, std::bind(&P23StatusNode::updateSLAMInformation, this, _1));

    lifecycle_node_status_subscription_ = create_subscription<custom_msgs::msg::LifecycleNodeStatus>(
        "lifecycle_manager/lifecycle_node_status", 10, std::bind(&P23StatusNode::receiveNodeStatus, this, _1));
}


void P23StatusNode::setPublishers() {
    // Set publisher towards CANBUS writer topic
    canbus_system_state_publisher_ = create_publisher<custom_msgs::msg::TxSystemState>("/system_state", 10);
}


void P23StatusNode::setServices() {
    // Set a service to change DV Status and to receive IMU Mode
    p23_status_client_ = create_client<custom_msgs::srv::DriverlessTransition>("lifecycle_manager/change_driverless_status");
    ins_mode_client_ = create_client<custom_msgs::srv::InsMode>("/vn_300/get_ins_mode");
    vectornav_heartbeat_client_ = create_client<custom_msgs::srv::InsMode>("/vn_200/get_ins_mode");
}


// Callback function when receiving new mission message
void P23StatusNode::updateMission(const custom_msgs::msg::MissionSelection::SharedPtr msg) {
    currentMission = static_cast<p23::Mission>(msg->mission_selected);

    // TODO: Check again lap counts
    switch (currentMission)
    {
    case(p23::MISSION_UNLOCKED):
        // Go back to LV-ON if we are in Mission Selected
        if ((currentDvStatus == p23::MISSION_SELECTED or currentDvStatus == p23::DV_READY) and (currentAsStatus == p23::AS_OFF))
        {
            changeDVStatus(p23::ON_MISSION_UNLOCKED);
        }
        else
        {
            // Display error
            RCLCPP_ERROR_STREAM(get_logger(), "Cannot unlock mission now!\nAS State: " << currentAsStatus << "\nDV State: " << currentDvStatus);
        }
        maxLaps = 255;
        return;
    case(p23::ACCELERATION):
        maxLaps = 1;
        break;
    case(p23::SKIDPAD):
        maxLaps = 5;
        break;
    case(p23::AUTOX):
        maxLaps = 1;
        break;
    case(p23::TRACKDRIVE):
        maxLaps = 10;
        break;
    case(p23::EBS_TEST):
        maxLaps = 255;
        break;
    case(p23::INSPECTION):
        maxLaps = 255;
        break;
    case(p23::MANUAL):
        // The PC will shutdown so no one cares what happens here...
        std::system("shutdown -h 1");
        return;
    default:
        RCLCPP_ERROR_STREAM(get_logger(), "Invalid mission received " << currentMission);
        currentMission = p23::MISSION_UNLOCKED;
        maxLaps = 255;
        return;
    }

    // We received new mission
    currentDvStatus = p23::MISSION_SELECTED;
    changeDVStatus(p23::ON_MISSION_LOCKED);
}


// Callback function when we receive new AS Status
void P23StatusNode::updateASStatus(const custom_msgs::msg::AutonomousStatus::SharedPtr msg) {
    p23::AS_Status statusReceived = static_cast<p23::AS_Status>(msg->id);

    // Maybe not for the stopping in Finished or Emergency...
    // We are constantly receiving the status...so this is to avoid spam
    if (statusReceived == currentAsStatus) { return; }

    switch (statusReceived)
    {
    case(p23::AS_Status::AS_OFF):
        // Don't do anything, wait for mission selection
        RCLCPP_WARN(get_logger(), "Current AS Status: AS_OFF. Waiting for mission lock");
        break;

    case(p23::AS_Status::AS_READY):
        // Activate all nodes except controls. We are waiting for go signal now.
        RCLCPP_WARN(get_logger(), "Current AS Status: AS_READY. Activating nodes");
        changeDVStatus(p23::ON_AS_READY);
        break;

    case(p23::AS_Status::AS_DRIVING):
        // This happens when we receive the go signal. Activate controls if we are not in DV_DRIVING.
        RCLCPP_WARN(get_logger(), "Current AS Status: AS_DRIVING. Activating controls");
        changeDVStatus(p23::ON_AS_DRIVING);
        break;

    case(p23::AS_Status::AS_FINISHED):
        // Mission is finished, probably shutdown nodes
        RCLCPP_WARN(get_logger(), "Current AS Status: AS_FINISHED. De-activating nodes");

        // TODO: If in standstill shutdown nodes
        if (standstill)
            changeDVStatus(p23::SHUTDOWN_NODES);
        break;

    case(p23::AS_Status::AS_EMERGENCY):
        /* Send a deactivation signal to the nodes.*/
        // Safely stop then shutdown nodes. Activate service brakes for redundancy.
        RCLCPP_WARN(get_logger(), "Current AS Status: AS_EMERGENCY. De-activating nodes");

        // TODO: Do emergency brake maneuver and when in standstill shutdown nodes
        // changeDVStatus(p23::SHUTDOWN_NODES);
        break;
    }
    currentAsStatus = statusReceived;
}


void P23StatusNode::changeDVStatus(p23::DV_Transitions requested_transition) {
    using namespace std::chrono_literals;
    p23::DV_Status transition_to;

    // Do checks to see if transition is valid
    switch (requested_transition)
    {
    case p23::DV_Transitions::ON_STARTUP:
        // This transition is only available when DV System is in STARTUP Status
        if (currentDvStatus != p23::STARTUP)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Cannot transition to LV_ON from " << p23::driverless_status_list.at(currentDvStatus));
            return;
        }
        transition_to = p23::DV_Status::LV_ON;
        break;

    case p23::DV_Transitions::ON_MISSION_LOCKED:
        // This transition can only happen if the mission isn't locked yet
        // TODO: Can this be called if we are in MISSION_SELECTED?
        if (currentAsStatus != p23::AS_OFF)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Cannot lock new mission when in " << p23::autonomous_status_list.at(currentAsStatus) << \
                ". Current mission is " << p23::mission_list.at(currentMission));
            return;
        }
        else if (currentDvStatus == p23::DV_READY or missionLocked)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Mission already locked, currently in DV_READY and " << p23::mission_list.at(currentMission) << ". Try to MISSION_UNLOCK");
            return;
        }
        missionLocked = true;
        transition_to = p23::DV_Status::DV_READY;
        break;

    case p23::DV_Transitions::ON_MISSION_UNLOCKED:
        // This transition is only available if we are in MISSION_LOCKED or DV_READY and still in AS_OFF
        if (currentAsStatus != p23::AS_OFF)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Cannot unlock mission when in " << p23::autonomous_status_list.at(currentAsStatus) << \
                ". Current mission is " << p23::mission_list.at(currentMission));
            return;
        }
        else if (currentDvStatus == p23::LV_ON or !missionLocked)
        {
            RCLCPP_ERROR(get_logger(), "Mission is already unlocked, currently in LV_ON");
            return;
        }
        missionLocked = false;
        transition_to = p23::DV_Status::LV_ON;
        break;

    case p23::DV_Transitions::ON_AS_READY:
        // This happens when we receive an AS_READY signal from the VCU
        if (currentAsStatus != p23::AS_READY)
        {
            RCLCPP_ERROR(get_logger(), "Current Autonomous System State is not AS_READY");
            return;
        }
        transition_to = p23::DV_Status::DV_READY;
        break;

    case p23::DV_Transitions::ON_AS_DRIVING:
        // This happens when we receive an AS_DRIVING signal from the VCU
        if (currentAsStatus != p23::AS_DRIVING)
        {
            RCLCPP_ERROR(get_logger(), "Current Autonomous System State is not AS_DRIVING");
            return;
        }
        transition_to = p23::DV_Status::DV_DRIVING;
        break;

    case p23::DV_Transitions::SHUTDOWN_NODES:
        // This transition is called when in MISSION_FINISHED or in AS_EMERGENCY, the car is stopped and the nodes are to be shutdown
        // TODO: Checks here
        break;

    default:
        RCLCPP_ERROR_STREAM(get_logger(), "Invalid transition requested: " << requested_transition);
        return;
    }


    // Create a service request to communicate with the Lifecycle Manager to change the status to whatever is specified
    auto request = std::make_shared<custom_msgs::srv::DriverlessTransition::Request>();
    request->transition.id = requested_transition;
    request->transition.label = p23::transition_list.at(requested_transition);
    request->mission.id = currentMission;
    request->mission.label = p23::mission_list.at(currentMission);


    RCLCPP_INFO_STREAM(get_logger(), "Sending a " << p23::transition_list.at(requested_transition) << " request to Lifecycle Manager");

    if (!p23_status_client_->wait_for_service(5s))
    {
        /*
            Just set pc_error bool to True and it will automatically send to the LV system.
        */
        RCLCPP_INFO(get_logger(), "Lifecycle Manager Service not responding");
        pcError = true;
        return;
    }

    // Send request to Lifecycle Manager
    using ServiceResponseFuture = rclcpp::Client<custom_msgs::srv::DriverlessTransition>::SharedFuture;
    auto response_received_callback = [this, requested_transition, transition_to](ServiceResponseFuture future) {
        auto result = future.get();
        bool success = result.get()->success;

        // If transitioning to LV_ON -> nodesReady = false
        // If transitioning to DV_READY -> nodesReady = true
        if (success)
        {
            if (transition_to == p23::DV_READY and !nodesReady)
            {
                nodesReady = true;
                RCLCPP_WARN(get_logger(), "Nodes successfully transitioned to DV_READY, waiting for INS mode 2");
            }
            else if (transition_to == p23::LV_ON)
            {
                RCLCPP_WARN(get_logger(), "Driverless Status successfully changed to: LV_ON");
                nodesReady = false;
            }
            else
            {
                RCLCPP_WARN_STREAM(get_logger(), "Driverless Status successfully changed to: " << p23::driverless_status_list.at(transition_to));
            }
            currentDvStatus = transition_to;
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "Could not change Driverless Status to: " << p23::driverless_status_list.at(transition_to));
            // Error-Handling point
        }
    };

    auto future_result = p23_status_client_->async_send_request(request, response_received_callback);
}


void P23StatusNode::loadParameters() {
    nodeList = declare_parameter<std::vector<std::string>>("managing_node_list",
        { "acquisition_left", "acquisition_right", "acquisition_center", "inference",
            "velocity_estimation", "slam", "saltas", "path_planning"
        });
    systemStateFrequency = declare_parameter<int>("system_state_publish_frequency", 10);
    sensorCheckupFrequency = declare_parameter<int>("sensor_checkup_frequency", 1);
}
} // p23_status_namespace