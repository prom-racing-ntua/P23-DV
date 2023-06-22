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

            // Get max laps: DOES NOT WORK
            // if (currentAsStatus == p23::AS_READY)
            // {
            //     rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_cli;

            //     if (currentMission == p23::INSPECTION or currentMission == p23::EBS_TEST) {
            //         maxLaps = 255;
            //         RCLCPP_WARN(get_logger(), "Inspection or EBS mission, no max laps");
            //         return;
            //     }
            //     else if((currentMission == p23::SKIDPAD) or (currentMission == p23::TRACKDRIVE)) {
            //         RCLCPP_INFO(get_logger(), "Loading max laps parameter from MPC node");
            //         param_cli = create_client<rcl_interfaces::srv::GetParameters>("mpc/get_parameters");
            //     }
            //     else if ((currentMission==p23::ACCELERATION) or (currentMission==p23::AUTOX)) {
            //         RCLCPP_INFO(get_logger(), "Loading max laps parameter from Pure Pursuit node");
            //         param_cli = create_client<rcl_interfaces::srv::GetParameters>("pure_pursuit/get_parameters");
            //     }
            //     else {
            //         RCLCPP_ERROR_STREAM(get_logger(), "Unknown mission, using default value for max laps: " << maxLaps);
            //         return;
            //     }

            //     if (!param_cli->wait_for_service(std::chrono::seconds(1))) {
            //         RCLCPP_ERROR_STREAM(get_logger(), "Controls node not responding, using default value for max laps: " << maxLaps);
            //         return;
            //     }

            //     auto request{ std::make_shared<rcl_interfaces::srv::GetParameters::Request>() };
            //     request->names.push_back("total_laps");

            //     auto max_laps_callback = [this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
            //         RCLCPP_ERROR(get_logger(), "In callback");
            //         auto result = future.get();
            //         if (result->values.empty()) {
            //             RCLCPP_ERROR_STREAM(get_logger(), "Get parameter failed, using default value for max laps: " << maxLaps);
            //             return;
            //         }
            //         maxLaps = result->values[0].integer_value;
            //         RCLCPP_WARN_STREAM(get_logger(), "Max laps set to: " << maxLaps);
            //     };

            //     auto get_max_laps_result = param_cli->async_send_request(request, max_laps_callback);
            // }
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "Could not change Driverless Status to: " << p23::driverless_status_list.at(transition_to));
            // Error-Handling point
        }        
    }
}