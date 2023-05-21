#include "spline_recorder.h"
#include <string>
#include "ament_index_cpp/get_package_share_directory.hpp"


namespace path_planning
{
SplineRecorder::SplineRecorder() : Node("spline_recorder") {
    count_ = 0;
    std::string share_dir{ ament_index_cpp::get_package_share_directory("splines") };
    RCLCPP_INFO(get_logger(), share_dir);
    spline_file_.open(share_dir + "/../../../../spline_log.txt");

    sub_ = create_subscription<custom_msgs::msg::WaypointsMsg>("waypoints", 10, std::bind(&SplineRecorder::splineCallback, this, std::placeholders::_1));
}

SplineRecorder::~SplineRecorder() {
    spline_file_.close();
}

void SplineRecorder::splineCallback(const custom_msgs::msg::WaypointsMsg::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Entered Callback");
    spline_file_ << ++count_ << '\n';


    PointsArray midpoints{ msg->count, 2 };
    for (int i{ 0 }; i < msg->count; i++)
    {
        midpoints(i, 0) = msg->waypoints[i].x;
        midpoints(i, 1) = msg->waypoints[i].y;
    }

    if (msg->count <= 2)
    {
        spline_file_ << midpoints << '\n';
    }
    else
    {
        ArcLengthSpline spline{ midpoints, BoundaryCondition::Anchored };
        PointsArray spline_points{ spline.getSplineCurve(100) };
        spline_file_ << spline_points << '\n';
    }
}
} // namespace path_planning


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<path_planning::SplineRecorder>());
    rclcpp::shutdown();
    return 0;
}