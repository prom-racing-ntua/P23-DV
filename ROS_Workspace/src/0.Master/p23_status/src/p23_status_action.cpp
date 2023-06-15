#include "p23_status_node.hpp"

namespace p23_status_namespace
{
    
    void P23StatusNode::TransitionResponse(GoalHandle::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(get_logger(), "Goal accepted by Lifecycle Manager");
        }
    }

    void P23StatusNode::TransitionFeedback(GoalHandle::SharedPtr goalHandle, const std::shared_ptr<const Transition::Feedback> feedback)
    {
        uint remainingTransitions, failedTransitions;

        remainingTransitions = feedback->remaining_transitions;
        failedTransitions = feedback->failed_transitions;

        if (failedTransitions != 0) {
            RCLCPP_ERROR(get_logger(), "A Lifecycle Transitions has failed, Cancel DV Transition and do not update status");
            dv_transition_client_->async_cancel_goal(goalHandle);
        }

        RCLCPP_INFO(get_logger(), "Got feedback from Manager. Remaining Transitions: %u", remainingTransitions);
    }

    void P23StatusNode::TransitionResult(const GoalHandle::WrappedResult &result)
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

        /* Placeholder */
        p23::DV_Status transition_to = p23::DV_Status::LV_ON;
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
        }
        else
        {
            RCLCPP_WARN_STREAM(get_logger(), "Could not change Driverless Status to: " << p23::driverless_status_list.at(transition_to));
            // Error-Handling point
        }        
    }
}