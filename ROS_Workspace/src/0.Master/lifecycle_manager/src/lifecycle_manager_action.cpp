#include "lifecycle_manager_node.hpp"


namespace lifecycle_manager_namespace
{
    rclcpp_action::GoalResponse LifecycleManagerNode::handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DVTransition::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "New Goal From P23 Status");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse LifecycleManagerNode::handleCancellation(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel transition");
        ongoing_goal_handle = nullptr;
        goalTimer->cancel();
        /* Reset the transitions already made. */
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void LifecycleManagerNode::handleAccept(const std::shared_ptr<GoalHandle> goal_handle)
    {
        ongoing_goal_handle = goal_handle;
        const auto goal = goal_handle->get_goal();

        p23::DV_Transitions newDVStatus = static_cast<p23::DV_Transitions>(goal->transition.id);
        p23::Mission missionSent = static_cast<p23::Mission>(goal->mission.id);

        /* Reset the failedTransitionCounter and set new goalCounter depending on transition */
        failedTransitionCounter = 0;

        using namespace std::placeholders;
        switch(newDVStatus) {
            case(p23::DV_Transitions::ON_STARTUP):
                RCLCPP_INFO(get_logger(), "Received Startup signal, start a heartbeat check for each node managed");
                startup();
                goalCounter = 0;
                break;
            case(p23::DV_Transitions::SHUTDOWN_NODES):
                RCLCPP_INFO(get_logger(), "Either Mission Finished or AS Emergency, shutting down currently running nodes");
                goalCounter = nodeList.size();
                std::thread{std::bind(&LifecycleManagerNode::shutdownSelectedNodes, this, _1, _2), nodeList, Transition::TRANSITION_ACTIVE_SHUTDOWN}.detach();
                break;
            case(p23::DV_Transitions::ON_MISSION_LOCKED):
                RCLCPP_INFO(get_logger(), "Received mission, configure the nodes");
                goalCounter = nodeList.size();
                std::thread{std::bind(&LifecycleManagerNode::configureNodes, this, _1), missionSent}.detach();
                break;
            case(p23::DV_Transitions::ON_MISSION_UNLOCKED):
                RCLCPP_INFO(get_logger(), "Unlocking mission, waiting for new mission to arrive, cleanup nodes");
                goalCounter = nodeList.size();
                std::thread{std::bind(&LifecycleManagerNode::cleanupNodes, this)}.detach();
                break;
            case(p23::DV_Transitions::ON_AS_READY):
                RCLCPP_INFO(get_logger(), "Received AS Ready, shutdown the nodes that are not used and wait for AS driving");
                goalCounter = nodeList.size() + nodesToShutdown.size() - 1;
                std::thread{std::bind(&LifecycleManagerNode::activateSystem, this)}.detach();
                break;
            case(p23::DV_Transitions::ON_AS_DRIVING):
                goalCounter = 1;
                RCLCPP_INFO(get_logger(), "Received AS Driving, activate control node");
                std::thread{std::bind(&LifecycleManagerNode::activateControls, this)}.detach();
                break;
            default:
                RCLCPP_ERROR(get_logger(), "Unknown Transition Requested");
                return;
        }
        goalTimer->reset();
        return;
    }

    void LifecycleManagerNode::publishActionFeedback() 
    {
        auto result = std::make_shared<DVTransition::Result>();
        auto feedback = std::make_shared<DVTransition::Feedback>();

        if (ongoing_goal_handle == nullptr)
        {
            RCLCPP_ERROR(get_logger(), "Ongoing goal handle has not been set. Canceling timer");
            goalTimer->cancel();
            return;
        }

        if (ongoing_goal_handle->is_canceling()) 
        {
            result->success = false;
            ongoing_goal_handle->canceled(result);
            ongoing_goal_handle = nullptr;
            goalTimer->cancel();
            RCLCPP_INFO(get_logger(), "Driverless Transition Cancelled");
            return;
        }

        /* Async cancel goal does not work very well, I think that we should abort from the Lifecycle Manager. 
            Should not be a problem since both nodes know how many transitions failed.*/

        feedback->remaining_transitions = goalCounter;
        feedback->failed_transitions = failedTransitionCounter;
        ongoing_goal_handle->publish_feedback(feedback);

        if (goalCounter <= 0) {
            result->success = true;
            ongoing_goal_handle->succeed(result);
            ongoing_goal_handle = nullptr;
            goalTimer->cancel();
            RCLCPP_INFO(get_logger(), "Driverless Transition complete!");
        }
    }
}