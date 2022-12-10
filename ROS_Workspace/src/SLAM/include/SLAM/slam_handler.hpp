#ifndef SLAM_HANDLER_HPP
#define SLAM_HANDLER_HPP

#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>


class SlamHandler : public rclcpp::Node {
private:
    // Subscribe to Perception topic
    rclcpp::Subscription<perception_msgs::msg::Perception2Slam>::SharedPtr perception2slam_sub_;

    void perceptionMessageCallback(const perception_msgs::msg::Perception2Slam::SharedPtr msg);
}