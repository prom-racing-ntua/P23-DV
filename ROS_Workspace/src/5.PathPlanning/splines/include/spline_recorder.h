#ifndef SPLINE_RECORDER_H
#define SPLINE_RECORDER_H

#include <fstream>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "custom_msgs/msg/waypoints_msg.hpp"

#include "arc_length_spline.h"

namespace path_planning
{
// 
class SplineRecorder : public rclcpp::Node {
private:
    // ArcLengthSpline spline_;
    std::ofstream spline_file_;
    int count_;
    rclcpp::Subscription<custom_msgs::msg::WaypointsMsg>::SharedPtr sub_;

public:
    SplineRecorder();
    ~SplineRecorder();
    void splineCallback(const custom_msgs::msg::WaypointsMsg::SharedPtr msg);

};
} // namespace path_planning

#endif //SPLINE_RECORDER_H