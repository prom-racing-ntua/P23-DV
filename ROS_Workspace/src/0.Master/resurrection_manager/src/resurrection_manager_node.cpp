#include "resurrection_manager.hpp"

namespace resurrection_manager_namespace
{
    ResurrectionManagerNode::ResurrectionManagerNode() : Node("resurrection_manager")
    {
        loadParameters();
        initializeServices();
        RCLCPP_INFO(get_logger(), "Resurrection Manager launched, waiting for dead nodes");
    }

    void ResurrectionManagerNode::initializeServices()
    {
        resurrectionService = create_service<custom_msgs::srv::ResurrectNode>(
            std::string(get_name()) + std::string("resurrection_service"), 
            std::bind(&ResurrectionManagerNode::handleResurrection, this, std::placeholders::_1, std::placeholders::_2)
        );

        resurrectionOrderService = create_service<custom_msgs::srv::ResurrectOrder>(
            std::string(get_name()) + std::string("resurrection_node_control"),
            std::bind(&ResurrectionManagerNode::multicastOrder, this, std::placeholders::_1, std::placeholders::_2)
        );
    }

    void ResurrectionManagerNode::loadParameters()
    {
        nodeList = declare_parameter<std::vector<std::string>>("managing_node_list",
            { "acquisition_left", "acquisition_right", "inference",
                "velocity_estimation", "slam", "saltas", "path_planning", "mpc", "pure_pursuit"
            });
    }
}