#include <rmw/qos_profiles.h>
#include "p23_status_node.hpp"


namespace p23_status_namespace
{
void P23StatusNode::updateSLAMInformation(const custom_msgs::msg::PoseMsg::SharedPtr msg) {
    conesCountAll = msg->cones_count_all;
    // TODO: Fix in SLAM -> Message is uint_8 ... but takes value -1
    currentLap = msg->lap_count;
    // standstill = msg->standstill;
    

    if (currentLap >= maxLaps)
    {
        // TODO: Go to Mission Finished state and start braking
    }
}

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
            insMode = result->ins_mode;

            if ((insMode == 2) and nodesReady and (currentDvStatus != p23::DV_READY) and (currentAsStatus == p23::AS_OFF) and (currentDvStatus != p23::DV_DRIVING))
            {
                RCLCPP_WARN(get_logger(), "INS in mode 2 and DV System ready. Transitioning to DV_READY");
                currentDvStatus = p23::DV_READY;
            }
            else { RCLCPP_INFO(get_logger(), "Received INS Mode from VN-300: %u", insMode); }
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
    for (auto const& [key, error] : nodeStatusMap) {
        /* If a node has a problem then check if this node is a critical one */
        if (error) {
            currentDvStatus = p23::NODE_PROBLEM;
            if (std::find(nodeList.begin(), nodeList.end(), key) != nodeList.end()) {
                /* TODO: Talk about which node should send us to AS_EMERGENCY mode */
            }
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
} // p23_status_namespace