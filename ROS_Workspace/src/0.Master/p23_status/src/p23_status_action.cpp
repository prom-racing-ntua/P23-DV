#include "p23_status_node.hpp"

namespace p23_status_namespace
{
    // Receives the goal from the action server and sees if the goal was accepted or not
    void P23StatusNode::TransitionResponse(GoalHandle::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(get_logger(), "Goal accepted by Lifecycle Manager");
        }
    }

    // Continuously receives the action feedback and checks if any node failed
    void P23StatusNode::TransitionFeedback(GoalHandle::SharedPtr goalHandle, const std::shared_ptr<const Transition::Feedback> feedback)
    {
        uint remainingTransitions, failedTransitions;

        remainingTransitions = feedback->remaining_transitions;
        failedTransitions = feedback->failed_transitions;
        RCLCPP_INFO(get_logger(), "Got feedback from Manager. Remaining Transitions: %u. Failed Transitions: %u", remainingTransitions, failedTransitions);

        if (failedTransitions != 0) {
            RCLCPP_ERROR(get_logger(), "A Lifecycle Transitions has failed, Cancel DV Transition and do not update status");
            dv_transition_client_->async_cancel_goal(goalHandle);
        }

        return;
    }

    // Receives the final response when the transition goal is reached
    void P23StatusNode::TransitionResult(const GoalHandle::WrappedResult &result, const p23::DV_Status& transition_to)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(), "DV Transition was successful");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(get_logger(), "Unknown result code");
                return;
        }
        
        bool success = result.result->success;

        /* Add the transition to the result so we know which dv state to go to*/
        if (success)
        {
            if (transition_to == p23::DV_READY and !nodesReady)
            {
                nodesReady = true;
                RCLCPP_WARN(get_logger(), "Nodes successfully transitioned to DV_READY, waiting for INS mode 2");
                return;
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

            // Get max laps
            if (currentAsStatus == p23::AS_READY)
            {
                if (currentMission == p23::INSPECTION or currentMission == p23::EBS_TEST) {maxLaps = 255;}
                else
                {
                    auto enabled_nodes{ get_node_names() };
                    bool is_mpc_active{ std::find(enabled_nodes.begin(), enabled_nodes.end(), "/mpc") != enabled_nodes.end() };
                    bool is_pid_active{ std::find(enabled_nodes.begin(), enabled_nodes.end(), "/pid_pp_controller") != enabled_nodes.end() };

                    if (is_mpc_active and is_pid_active)
                    {
                        RCLCPP_ERROR(get_logger(), "Both MPC and PID are active in AS Ready");
                    }
                    else if (is_mpc_active)
                    {
                        auto param_cli{ std::make_shared<rclcpp::SyncParametersClient>(this, "/mpc") };
                        if (!param_cli->wait_for_service(std::chrono::seconds(1))) \
                            RCLCPP_ERROR(get_logger(), "Controls node not responding, using default value for max laps");
                        else \
                            maxLaps = param_cli->get_parameter("max_laps", maxLaps);
                    }
                    else if (is_pid_active)
                    {
                        auto param_cli{ std::make_shared<rclcpp::SyncParametersClient>(this, "/pid_pp_controller") };
                        if (!param_cli->wait_for_service(std::chrono::seconds(1))) \
                            RCLCPP_ERROR(get_logger(), "Controls node not responding, using default value for max laps");
                        else \
                            maxLaps = param_cli->get_parameter("max_laps", maxLaps);
                    }
                    else
                    {
                        RCLCPP_ERROR(get_logger(), "No controller nodes are active");
                    }
                    RCLCPP_WARN_STREAM(get_logger(), "Max laps set to: " << maxLaps);
                }
            }
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "Could not change Driverless Status to: " << p23::driverless_status_list.at(transition_to));
            // Error-Handling point
        }        
    }
}