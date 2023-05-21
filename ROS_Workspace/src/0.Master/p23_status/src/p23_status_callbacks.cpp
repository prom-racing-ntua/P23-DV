#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include "p23_status_node.hpp"


namespace p23_status_namespace
{
    void P23StatusNode::updateSLAMInformation(const custom_msgs::msg::PoseMsg::SharedPtr msg)
    {
        conesCountAll = msg->cones_count_all;
        currentLap = msg->lap_count;

        if (currentLap >= maxLaps)
        {
            // Go to Mission Finished state and start braking
        }
    }

    void P23StatusNode::checkSensors()
    {
        requestINSStatus();
        vn200Heartbeat();
    }

    void P23StatusNode::receiveNodeStatus(const custom_msgs::msg::LifecycleNodeStatus::SharedPtr msg)
    {
        nodeStatusMap["saltas"] = msg->clock_ok;
        nodeStatusMap["acquisition_left"] = msg->camera_left_ok;
        nodeStatusMap["acquisition_right"] = msg->camera_right_ok;
        nodeStatusMap["pure_pursuit"] = msg->pi_pp_controls_ok;
        nodeStatusMap["mpc"] = msg->mpc_controls_ok;
        nodeStatusMap["slam"] = msg->slam_ok;
        nodeStatusMap["velocity_estimation"] = msg->velocity_estimation_ok;
        nodeStatusMap["inference"] = msg->inference_ok;
        nodeStatusMap["path_planning"] = msg->path_planning_ok;
    }

    void P23StatusNode::sendSystemState()
    {
        custom_msgs::msg::TxSystemState systemStateMsg;

        /* Information that is sent by the P23 Status Node */
        systemStateMsg.lap_counter = currentLap;
        systemStateMsg.cones_count_actual = conesActual;
        systemStateMsg.cones_count_all = conesCountAll;
        systemStateMsg.dv_status.id = currentDVStatus;

        systemStateMsg.vn_200_ok = nodeStatusMap["vn_200"];
        systemStateMsg.vn_300_ok = nodeStatusMap["vn_300"];

        /* Information that is sent by the Lifecycle Manager Node*/
        systemStateMsg.camera_left_ok = nodeStatusMap["acquisition_left"];
        systemStateMsg.camera_right_ok = nodeStatusMap["acquisition_right"];
        
        systemStateMsg.clock_ok = nodeStatusMap["saltas"];
        systemStateMsg.camera_inference_ok = nodeStatusMap["inference"];
        systemStateMsg.velocity_estimation_ok = nodeStatusMap["velocity_estimation"];
        systemStateMsg.slam_ok = nodeStatusMap["slam"];
        systemStateMsg.mpc_controls_ok = nodeStatusMap["mpc"];
        systemStateMsg.path_planning_ok = nodeStatusMap["path_planning"];
        systemStateMsg.pi_pp_controls_ok = nodeStatusMap["pure_pursuit"];

        canbus_system_state_publisher_->publish(systemStateMsg);
    }

    void P23StatusNode::requestINSStatus()
    {
        using namespace std::chrono_literals;
        auto vn300_request = std::make_shared<custom_msgs::srv::InsMode::Request>();
        
        while (!ins_mode_client_->wait_for_service(std::chrono::milliseconds(100)))
        {
            nodeStatusMap["vn_300"] = true;
            RCLCPP_INFO(get_logger(), "INS Service not available, waiting...");
        }

        using vn300ServiceResponseFuture = rclcpp::Client<custom_msgs::srv::InsMode>::SharedFuture;
        
        auto vn300_heartbeat_callback = [this](vn300ServiceResponseFuture future) {
            auto result = future.get();
            insMode = result.get()->ins_mode;
            nodeStatusMap["vn_300"] = false;
            RCLCPP_INFO(get_logger(), "Received INS Mode from VN-300: %u", insMode);
        };

        auto future_result = ins_mode_client_->async_send_request(vn300_request, vn300_heartbeat_callback);
    }

    void P23StatusNode::vn200Heartbeat()
    {
        using namespace std::chrono_literals;
        auto vn200_request = std::make_shared<custom_msgs::srv::VectornavHeartbeat::Request>();

        while (!vectornav_heartbeat_client_->wait_for_service(std::chrono::milliseconds(100)))
        {
            nodeStatusMap["vn_200"] = true;
            RCLCPP_INFO(get_logger(), "Vectornav Heartbeat service not available, waiting...");
        }

        using vn200ServiceResponseFuture = rclcpp::Client<custom_msgs::srv::VectornavHeartbeat>::SharedFuture;

        auto vn200_heartbeat_callback = [this](vn200ServiceResponseFuture future) {
            auto result = future.get();
            nodeStatusMap["vn_200"] = false;
        };

        auto future_result = vectornav_heartbeat_client_->async_send_request(vn200_request, vn200_heartbeat_callback);
    }
}