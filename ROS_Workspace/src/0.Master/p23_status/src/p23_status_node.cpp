#include <rmw/qos_profiles.h>
#include "p23_status_node.hpp"


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

    currentLap = 0;
    // Initialize to 255 because current lap received might be 0
    maxLaps = 255;
    standstill = false;
    pcError = false;
    pcToBeShutdown = false;

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
        "/canbus/autonomous_status", 10, std::bind(&P23StatusNode::updateASStatus, this, _1));

    slam_subscription_ = create_subscription<custom_msgs::msg::LocalMapMsg>(
        "local_map", 10, std::bind(&P23StatusNode::updateSLAMInformation, this, _1));

    lifecycle_node_status_subscription_ = create_subscription<custom_msgs::msg::LifecycleNodeStatus>(
        "lifecycle_manager/lifecycle_node_status", 10, std::bind(&P23StatusNode::receiveNodeStatus, this, _1));
}


void P23StatusNode::setPublishers() {
    // Set publisher towards CANBUS writer topic
    canbus_system_state_publisher_ = create_publisher<custom_msgs::msg::TxSystemState>("/system_state", 10);

    dv_transition_client_ = rclcpp_action::create_client<custom_msgs::action::DriverlessTransition>(this,
      "lifecycle_manager/change_driverless_status");
}

void P23StatusNode::setServices() {
    // Set a service to change DV Status and to receive IMU Mode
    ins_mode_client_ = create_client<custom_msgs::srv::InsMode>("/vn_300/get_ins_mode");
    vectornav_heartbeat_client_ = create_client<custom_msgs::srv::InsMode>("/vn_200/get_ins_mode");

    // Create server to receive total laps
    total_laps_server_ = create_service<custom_msgs::srv::SetTotalLaps>(
        std::string(get_name()) + "/set_total_laps", std::bind(&P23StatusNode::setTotalLaps, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, mission_selection_group_
    );
}

// Callback function when receiving new mission message
void P23StatusNode::updateMission(const custom_msgs::msg::MissionSelection::SharedPtr msg) {
    if (missionLocked and static_cast<p23::Mission>(msg->mission_selected) != p23::MISSION_UNLOCKED) //xx: giati vazoume 2 
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Mission already locked, currently in DV_READY and " << p23::mission_list.at(currentMission) << ". Try to MISSION_UNLOCK");
        return;
    }
    else if (currentAsStatus != p23::AS_OFF)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Cannot select new mission when in " << p23::autonomous_status_list.at(currentAsStatus) << \
            ". Current mission is " << p23::mission_list.at(currentMission));
        return;
    }

    /* If we are on manual and then we reselect, then stop the PC from shutting down */
    if (pcToBeShutdown) {
        pcToBeShutdown = false;
        std::system("shutdown -c");
    }

    currentMission = static_cast<p23::Mission>(msg->mission_selected);

    // TODO: Check again lap counts
    switch (static_cast<p23::Mission>(msg->mission_selected))
    {
    case(p23::MISSION_UNLOCKED):
        // Go back to LV-ON if we are in Mission Selected
        changeDVStatus(p23::ON_MISSION_UNLOCKED);
        maxLaps = 255;
        return;
    case(p23::ACCELERATION):
        maxLaps = 1;
        break;
    case(p23::SKIDPAD):
        maxLaps = 5;
        break;
    case(p23::AUTOX):
        maxLaps = 2;
        break;
    case(p23::TRACKDRIVE):
        maxLaps = 11;
        break;
    case(p23::EBS_TEST):
        maxLaps = 255;
        break;
    case(p23::INSPECTION):
        maxLaps = 255;
        break;
    case(p23::MANUAL):
        // The PC will shutdown so no one cares what happens here...
        std::system("shutdown -h 30 sec");
        pcToBeShutdown = true;
        // TODO: Command to cancel is "shutdown -c". Create a flag that sets shutdown and cancels it if we change mission"
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

    bool success;
    switch (statusReceived)
    {
    case(p23::AS_Status::AS_OFF):
        // Don't do anything, wait for mission selection
        RCLCPP_WARN(get_logger(), "Current AS Status: AS_OFF. Waiting for mission lock");
        success = true;
        break;

    case(p23::AS_Status::AS_READY):
        // Activate all nodes except controls. We are waiting for go signal now.
        RCLCPP_WARN(get_logger(), "Current AS Status: AS_READY. Activating nodes");
        success = changeDVStatus(p23::ON_AS_READY);
        break;
        
    case(p23::AS_Status::AS_DRIVING):
        // This happens when we receive the go signal. Activate controls if we are not in DV_DRIVING.
        RCLCPP_WARN(get_logger(), "Current AS Status: AS_DRIVING. Activating controls");
        success = changeDVStatus(p23::ON_AS_DRIVING);
        break;

    case(p23::AS_Status::AS_FINISHED):
        // Mission is finished, probably shutdown nodes
        RCLCPP_WARN(get_logger(), "Current AS Status: AS_FINISHED. De-activating nodes");
        currentAsStatus = statusReceived;
        
        success = changeDVStatus(p23::SHUTDOWN_NODES);
        break;

    case(p23::AS_Status::AS_EMERGENCY):
        if(currentDvStatus == p23::DV_Status::STARTUP || currentDvStatus == p23::DV_Status::LV_ON|| currentDvStatus == p23::DV_Status::MISSION_SELECTED)
        {
            RCLCPP_WARN(get_logger(), "Current AS Status: AS_EMERGENCY. Nodes already inactive");
            currentAsStatus = statusReceived;
            success = 1;
        }
        else
        {
            /* Send a deactivation signal to the nodes.*/
            // Safely stop then shutdown nodes. Activate service brakes for redundancy.
            RCLCPP_WARN(get_logger(), "Current AS Status: AS_EMERGENCY. De-activating nodes");
            currentAsStatus = statusReceived;

            // TODO: Do emergency brake maneuver and when in standstill shutdown nodes
            success = changeDVStatus(p23::SHUTDOWN_NODES);
        }
        break;
    }
    if (success) currentAsStatus = statusReceived;
}

bool P23StatusNode::changeDVStatus(p23::DV_Transitions requested_transition) {
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
            return 0;
        }
        transition_to = p23::DV_Status::LV_ON;
        break;

    case p23::DV_Transitions::ON_MISSION_LOCKED:
        // This transition can only happen if the mission isn't locked yet
        missionLocked = true;
        transition_to = p23::DV_Status::DV_READY;
        break;

    case p23::DV_Transitions::ON_MISSION_UNLOCKED:
        // This transition is only available if we are in MISSION_LOCKED or DV_READY and still in AS_OFF
        if (!missionLocked)
        {
            RCLCPP_ERROR(get_logger(), "Mission is already unlocked, currently in LV_ON");
            return 0;
        }
        missionLocked = false;
        transition_to = p23::DV_Status::LV_ON;
        break;

    case p23::DV_Transitions::ON_AS_READY:
        // This happens when we receive an AS_READY signal from the VCU
        if (currentAsStatus != p23::AS_OFF) {
            RCLCPP_ERROR(get_logger(), "Cannot trnsition to AS_READY. Current Autonomous System State is not AS_OFF");
            return 0;
        }
        // We need to be in DV_READY to transition to AS_READY
        if (currentDvStatus != p23::DV_READY) {
            RCLCPP_ERROR(get_logger(), "Cannot trnsition to AS_READY. Current Driverless System State is not DV_READY");
            return 0;
        }
        transition_to = p23::DV_Status::DV_READY;
        break;

    case p23::DV_Transitions::ON_AS_DRIVING:
        // This happens when we receive an AS_DRIVING signal from the VCU
        if (currentAsStatus != p23::AS_READY) {
            RCLCPP_ERROR(get_logger(), "Cannot trnsition to AS_DRIVING. Current Autonomous System State is not AS_READY");
            return 0;
        }
        // We need to be in DV_READY to transition to AS_DRIVING
        if (currentDvStatus != p23::DV_READY) {
            RCLCPP_ERROR(get_logger(), "Cannot transition to AS_DRIVING. Current Driverless System State is not DV_READY");
            return 0;
        }
        transition_to = p23::DV_Status::DV_DRIVING;
        break;

    case p23::DV_Transitions::SHUTDOWN_NODES:
        // This transition is called when in MISSION_FINISHED or in AS_EMERGENCY, the car is stopped and the nodes are to be shutdown
        // TODO: Checks here
        if (currentAsStatus == p23::AS_Status::AS_EMERGENCY) {
            RCLCPP_INFO(get_logger(), "Currently in AS_EMERGENCY, activate service brakes and when in standstill shutdown nodes.");
            transition_to = p23::DV_Status::NODE_PROBLEM;
        }
        else if (currentAsStatus == p23::AS_Status::AS_FINISHED) {
            RCLCPP_INFO(get_logger(), "AS_Finished, when in standstill, shutdown nodes");
            /* Should we spin around standstill flag and wait for it to become true? */
            transition_to = p23::DV_Status::MISSION_FINISHED;
        }
        break;
    default:
        RCLCPP_ERROR_STREAM(get_logger(), "Invalid transition requested: " << requested_transition);
        return 0;
    }

    if (!dv_transition_client_->wait_for_action_server(std::chrono::milliseconds(2000))) {
        RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
        return 0;
    }

    auto send_goal_options = rclcpp_action::Client<Transition>::SendGoalOptions();

    send_goal_options.goal_response_callback = std::bind(&P23StatusNode::TransitionResponse, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&P23StatusNode::TransitionFeedback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&P23StatusNode::TransitionResult, this, std::placeholders::_1, transition_to);
    
    auto goal = Transition::Goal();
    goal.transition.id = requested_transition;
    goal.transition.label = p23::transition_list.at(requested_transition);
    goal.mission.id = currentMission;
    goal.mission.label = p23::mission_list.at(currentMission);
    RCLCPP_INFO_STREAM(get_logger(), "Sending a " << p23::transition_list.at(requested_transition) << " request to Lifecycle Manager");

    // Send goal to Lifecycle Manager
    dv_transition_client_->async_send_goal(goal, send_goal_options);

    return 1;
}

void P23StatusNode::loadParameters() {
    nodeList = declare_parameter<std::vector<std::string>>("managing_node_list",
        { "acquisition_left", "acquisition_right", "acquisition_center", "inference",
            "velocity_estimation", "slam", "saltas", "path_planning"
        });
    systemStateFrequency = declare_parameter<int>("system_state_publish_frequency", 10);
    sensorCheckupFrequency = declare_parameter<int>("sensor_checkup_frequency", 1);
    wait_for_mode_2 = declare_parameter<bool>("wait_for_mode_2", true);
}
} // p23_status_namespace