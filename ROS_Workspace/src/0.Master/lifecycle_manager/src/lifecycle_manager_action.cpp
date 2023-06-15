#include "lifecycle_manager_node.hpp"


namespace lifecycle_manager_namespace
{
    rclcpp_action::GoalResponse LifecycleManagerNode::handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DVTransition::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "Received Goal From P23 Status");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse LifecycleManagerNode::handleCancelation(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel transition");
        /* Reset the transitions already made */
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void LifecycleManagerNode::handleAccept(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Gamw thn panagia sas");
        const auto goal = goal_handle->get_goal();
        rclcpp::Rate loop_rate(1);

        p23::DV_Transitions newDVStatus = static_cast<p23::DV_Transitions>(goal->transition.id);
        p23::Mission missionSent = static_cast<p23::Mission>(goal->mission.id);

        auto feedback = std::make_shared<DVTransition::Feedback>();
        auto result = std::make_shared<DVTransition::Result>();

        using namespace std::placeholders;
        switch(newDVStatus) {
            case(p23::DV_Transitions::ON_STARTUP):
                RCLCPP_INFO(get_logger(), "Received Startup signal, start a heartbeat check for each node managed");
                startup();
                goalCounter = 0;
                failedTransitionCounter = 0;
                result->success = true;
                goal_handle->succeed(result);
                return;
            case(p23::DV_Transitions::SHUTDOWN_NODES):
                RCLCPP_INFO(get_logger(), "Either Mission Finished or AS Emergency, shutting down currently running nodes");
                goalCounter = nodeList.size();
                std::thread{std::bind(&LifecycleManagerNode::shutdownSelectedNodes, this, _1, _2), nodeList, Transition::TRANSITION_ACTIVE_SHUTDOWN}.detach();
                // shutdownSelectedNodes(nodeList, Transition::TRANSITION_ACTIVE_SHUTDOWN);
                break;
            case(p23::DV_Transitions::ON_MISSION_LOCKED):
                RCLCPP_INFO(get_logger(), "Received mission, configure the nodes");
                goalCounter = nodeList.size();

                // std::thread{std::bind(&LifecycleManagerNode::configureNodes, this, _1), missionSent}.detach();
                configureNodes(missionSent);
                break;
            case(p23::DV_Transitions::ON_MISSION_UNLOCKED):
                RCLCPP_INFO(get_logger(), "Unlocking mission, waiting for new mission to arrive, cleanup nodes");
                goalCounter = nodeList.size();
                std::thread{std::bind(&LifecycleManagerNode::cleanupNodes, this)}.detach();
                // cleanupNodes();
                currentMission = p23::MISSION_UNLOCKED;
                break;
            case(p23::DV_Transitions::ON_AS_READY):
                RCLCPP_INFO(get_logger(), "Received AS Ready, shutdown the nodes that are not used and wait for AS driving");
                goalCounter = nodeList.size() + nodesToShutdown.size();
                std::thread{std::bind(&LifecycleManagerNode::activateSystem, this)}.detach();
                activateSystem();
                break;
            case(p23::DV_Transitions::ON_AS_DRIVING):
                goalCounter = 1;
                RCLCPP_INFO(get_logger(), "Received AS Driving, activate control node");
                std::thread{std::bind(&LifecycleManagerNode::activateControls, this)}.detach();
                activateControls();
                break;
        }

        while ((goalCounter != 0)  && rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "Driverless Transition Cancelled");
                return;
            }
            
            feedback->remaining_transitions = goalCounter;
            feedback->failed_transitions = failedTransitionCounter;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");

            loop_rate.sleep();
        }

        if (rclcpp::ok()) {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Driverless Transition to %u complete!", newDVStatus);
        }
    }
}