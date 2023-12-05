#include "lifecycle_manager_node.hpp"


namespace lifecycle_manager_namespace
{
    rclcpp_action::GoalResponse LifecycleManagerNode::handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DVTransition::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "New Goal From P23 Status");
        if (ongoing_goal_handle == nullptr) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        
        p23::DV_Transitions requested_transition{ static_cast<p23::DV_Transitions>(goal->transition.id) };
        // Complete current transition successfully and then accept incoming
        if ((requested_transition == p23::ON_AS_READY) or (requested_transition == p23::ON_AS_DRIVING) or (requested_transition == p23::SHUTDOWN_NODES)) \
            incoming_transition = false;
        // Complete current transition returning failure and then accept incoming
        else \
            incoming_transition = true;
        
        // Wait until transition is over [Ugly but works]
        while (ongoing_goal_handle!=nullptr) continue;
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

        auto status_msg = custom_msgs::msg::LifecycleNodeTransitionState();
        status_array = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        /* Reset the failedTransitionCounter and set new goalCounter depending on transition */
        failedTransitionCounter = 0;

        using namespace std::placeholders;
        switch(newDVStatus) {
            case(p23::DV_Transitions::ON_STARTUP):
                status_msg.transition_requested = 0;
                RCLCPP_INFO(get_logger(), "Received Startup signal, start a heartbeat check for each node managed");
                startup();
                goalCounter = 0;
                break;
            case(p23::DV_Transitions::SHUTDOWN_NODES):
                status_msg.transition_requested = 6;
                RCLCPP_INFO(get_logger(), "Either Mission Finished or AS Emergency, shutting down currently running nodes");
                goalCounter = nodeList.size();
                std::thread{std::bind(&LifecycleManagerNode::shutdownSelectedNodes, this, _1, _2), nodeList, Transition::TRANSITION_ACTIVE_SHUTDOWN}.detach();
                break;
            case(p23::DV_Transitions::ON_MISSION_LOCKED):
                status_msg.transition_requested = 1;
                RCLCPP_INFO(get_logger(), "Received mission, configure the nodes");
                goalCounter = nodeList.size();
                std::thread{std::bind(&LifecycleManagerNode::configureNodes, this, _1), missionSent}.detach();
                break;
            case(p23::DV_Transitions::ON_MISSION_UNLOCKED):
                status_msg.transition_requested = 2;
                RCLCPP_INFO(get_logger(), "Unlocking mission, waiting for new mission to arrive, cleanup nodes");
                goalCounter = nodeList.size();
                std::thread{std::bind(&LifecycleManagerNode::cleanupNodes, this)}.detach();
                break;
            case(p23::DV_Transitions::ON_AS_READY):
                status_msg.transition_requested = 3;
                RCLCPP_INFO(get_logger(), "Received AS Ready, shutdown the nodes that are not used and wait for AS driving");
                goalCounter = nodeList.size() + nodesToShutdown.size() - 1;
                std::thread{std::bind(&LifecycleManagerNode::activateSystem, this)}.detach();
                break;
            case(p23::DV_Transitions::ON_AS_DRIVING):
                status_msg.transition_requested = 5;
                goalCounter = 1;
                RCLCPP_INFO(get_logger(), "Received AS Driving, activate control node");
                std::thread{std::bind(&LifecycleManagerNode::activateControls, this)}.detach();
                break;
            default:
                RCLCPP_ERROR(get_logger(), "Unknown Transition Requested");
                return;
        }
        telemetry_node_transition_publisher->publish(status_msg);
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

        // auto status_msg = custom_msgs::msg::LifecycleNodeTransitionState();
        // telemetry_node_transition_publisher->publish(status_msg);

        if (goalCounter <= 0) {
            if (incoming_transition) result->success = false;
            else result->success = true;

            auto status_msg = custom_msgs::msg::LifecycleNodeTransitionState();
            telemetry_node_transition_publisher->publish(status_msg);
            
            ongoing_goal_handle->succeed(result);
            incoming_transition = false;
            ongoing_goal_handle = nullptr;
            goalTimer->cancel();
            RCLCPP_INFO(get_logger(), "Driverless Transition complete!");
        }
    }
}