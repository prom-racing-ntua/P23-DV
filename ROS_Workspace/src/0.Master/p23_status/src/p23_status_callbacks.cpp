#include <rmw/qos_profiles.h>
#include "p23_status_node.hpp"


namespace p23_status_namespace
{
void P23StatusNode::updateSLAMInformation(const custom_msgs::msg::LocalMapMsg::SharedPtr msg) {
    conesCountAll = msg->cones_count_all;
    conesActual = msg->cones_count_actual;
    currentLap = msg->lap_count;

    // Checking if the car as zero velocity for a set amount of time for standstill
    if (msg->pose.velocity_state.velocity_x < 0.005) // small value to see if vx = 0
    {
        if (now().seconds() - standstill_time > 2 and !standstill) 
        {
            standstill = true;
            RCLCPP_WARN(get_logger(), "Vehicle at standstill");
        }
    }
    else
    {
        if (standstill) RCLCPP_WARN(get_logger(), "Vehicle moving");
        standstill = false;
        standstill_time = now().seconds();
    }

    // Check if mission finished
    if ((currentLap == maxLaps) and standstill and (currentDvStatus!=p23::MISSION_FINISHED))
    {
        RCLCPP_WARN(get_logger(), "Laps Completed and vehicle at Standstill. MISSION FINISHED!");
        currentDvStatus = p23::MISSION_FINISHED;
    }
}

// void P23StatusNode::receiveMissionFinished(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
//     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
// {
//     RCLCPP_INFO(get_logger(), "Received mission finished trigger from control node");

//     /* Cancel Vectornav/Lifecycle Node status as to not go to PC Error by mistake */
//     sensorCheckupTimer_.reset();
//     lifecycle_node_status_subscription_.reset();

//     currentDvStatus = p23::DV_Status::MISSION_FINISHED;
//     /* With currentDvStatus set as Mission Finished, the VCU will automatically send an AS_FINISHED message that will later then change our
//         AS Status and shutdown the nodes. */
//     response->success = true;
// }

void P23StatusNode::checkVectornav() {
    using namespace std::chrono_literals;
    using VectornavServiceResponseFuture = rclcpp::Client<custom_msgs::srv::InsMode>::SharedFuture;
    auto vn300_request{ std::make_shared<custom_msgs::srv::InsMode::Request>() };
    auto vn200_request{ std::make_shared<custom_msgs::srv::InsMode::Request>() };

    auto vn300_heartbeat_callback = [this](VectornavServiceResponseFuture future) {
        auto result = future.get();
        nodeStatusMap["vn_300"] = !result->sensor_connected;
        if (result->sensor_connected)
        {
            // insMode = 2; //HARD CODED FOR LAB
            // If in mission inspection set ins mode to 2
            if (currentMission == p23::INSPECTION || !wait_for_mode_2) {
                insMode = 2;
            }
            // Check if we received the same mode as before
            else if (insMode != result->ins_mode)
            {
                insMode = result->ins_mode;
                // insMode = 2;
                // RCLCPP_INFO(get_logger(), "Received INS Mode from VN-300: %u", insMode);
            }

            // If in mode 2 and all nodes configured transition to DV Ready State
            if ((insMode == 2) and nodesReady and (currentDvStatus == p23::MISSION_SELECTED) and (currentAsStatus == p23::AS_OFF)) {
                RCLCPP_WARN(get_logger(), "INS in mode 2 and DV System ready. Transitioning to DV_READY");
                currentDvStatus = p23::DV_READY;
            }
        }
        else { insMode = 0; }
    };

    auto vn200_heartbeat_callback = [this](VectornavServiceResponseFuture future) {
        auto result = future.get();
        nodeStatusMap["vn_200"] = !result->sensor_connected;
    };

    // Send request to vn-300
    // Wait for service for 0.2 sec
    if (!ins_mode_client_->wait_for_service(std::chrono::milliseconds(200)))
    {
        nodeStatusMap["vn_300"] = true;
        RCLCPP_WARN(get_logger(), "VN-300 service is not available");
    }
    else { auto future_result = ins_mode_client_->async_send_request(vn300_request, vn300_heartbeat_callback); }

    // Send request to vn-200
    // Wait for service for 0.2 sec
    if (!vectornav_heartbeat_client_->wait_for_service(std::chrono::milliseconds(200)))
    {
        nodeStatusMap["vn_200"] = true;
        // nodeStatusMap["vn_200"] = false;
        RCLCPP_WARN(get_logger(), "VN-200 service is not available");
    }
    else { auto future_result = vectornav_heartbeat_client_->async_send_request(vn200_request, vn200_heartbeat_callback); }
}

void P23StatusNode::receiveNodeStatus(const custom_msgs::msg::LifecycleNodeStatus::SharedPtr msg) {
    nodeStatusMap["saltas"] = msg->clock_error;
    nodeStatusMap["acquisition_left"] = msg->camera_left_error;
    nodeStatusMap["acquisition_right"] = msg->camera_right_error;
    nodeStatusMap["pure_pursuit"] = msg->pi_pp_controls_error;
    nodeStatusMap["mpc"] = msg->mpc_controls_error;
    nodeStatusMap["slam"] = msg->slam_error;
    nodeStatusMap["velocity_estimation"] = msg->velocity_estimation_error;
    nodeStatusMap["inference"] = msg->inference_error;
    nodeStatusMap["path_planning"] = msg->path_planning_error;

    /* Iterate through map and check if any node is dead. If a critical node has failed, go to pc_error */
    for (auto const& it : nodeStatusMap)
    {
        /* If a node has a problem then check if this node is a critical one */
        if (it.second)
        {
            /* TODO: handleNodeProblem();*/
            currentDvStatus = p23::NODE_PROBLEM;
            nodesReady = false;
        }
    }
}

void P23StatusNode::sendSystemState() {
    custom_msgs::msg::TxSystemState systemStateMsg;

    /* Information that is sent by the P23 Status Node */
    systemStateMsg.lap_counter = currentLap;
    systemStateMsg.cones_count_actual = conesActual;
    systemStateMsg.cones_count_all = conesCountAll;
    systemStateMsg.dv_status.id = currentDvStatus;
    systemStateMsg.dv_status.label = p23::driverless_status_list.at(currentDvStatus);

    systemStateMsg.vn_200_error = nodeStatusMap["vn_200"];
    systemStateMsg.vn_300_error = nodeStatusMap["vn_300"];
    systemStateMsg.ins_mode = insMode;

    /* Information that is sent by the Lifecycle Manager Node*/
    systemStateMsg.camera_left_error = nodeStatusMap["acquisition_left"];
    systemStateMsg.camera_right_error = nodeStatusMap["acquisition_right"];

    systemStateMsg.clock_error = nodeStatusMap["saltas"];
    systemStateMsg.camera_inference_error = nodeStatusMap["inference"];
    systemStateMsg.velocity_estimation_error = nodeStatusMap["velocity_estimation"];
    systemStateMsg.slam_error = nodeStatusMap["slam"];
    systemStateMsg.mpc_controls_error = nodeStatusMap["mpc"];
    systemStateMsg.path_planning_error = nodeStatusMap["path_planning"];
    systemStateMsg.pi_pp_controls_error = nodeStatusMap["pure_pursuit"];

    canbus_system_state_publisher_->publish(systemStateMsg);
}

void P23StatusNode::setTotalLaps(const std::shared_ptr<custom_msgs::srv::SetTotalLaps::Request> request,
        std::shared_ptr<custom_msgs::srv::SetTotalLaps::Response> response) {
    // If we are in Inspection mission count this as a mission finished signal
    if (currentMission == p23::INSPECTION) {
        RCLCPP_WARN(get_logger(), "Inspection Mission Finished");
        currentDvStatus = p23::MISSION_FINISHED;
    }
    else {
        // Just set the total laps and return
        maxLaps = request->total_laps;
        RCLCPP_WARN_STREAM(get_logger(), "Received new total laps count: " << maxLaps);
    }
    response->success = true;
}
} // p23_status_namespace