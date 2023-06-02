#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "arc_length_spline.h"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/waypoints_msg.hpp"
#include "custom_msgs/msg/point2_struct.hpp"


namespace ns_telemetry
{
class Telemetry : public rclcpp::Node {
private:
    rclcpp::Subscription<custom_msgs::msg::PoseMsg>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<custom_msgs::msg::LocalMapMsg>::SharedPtr map_subscriber_;
    rclcpp::Subscription<custom_msgs::msg::WaypointsMsg>::SharedPtr waypoints_subscriber_;
    rclcpp::Subscription<custom_msgs::msg::Point2Struct>::SharedPtr look_ahead_subscriber_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr car_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr midpoints_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr look_ahead_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr spline_publisher_;

    void pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr msg);
    void map_callback(const custom_msgs::msg::LocalMapMsg::SharedPtr msg);
    void waypoints_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr msg);
    void look_ahead_callback(const custom_msgs::msg::Point2Struct::SharedPtr msg);

public:
    Telemetry();
    ~Telemetry();
};


Telemetry::Telemetry() : Node("telemetry") {
    using std::placeholders::_1;
    // Set publishers
    landmark_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("landmark_marker_array", 10);
    car_publisher_ = create_publisher<visualization_msgs::msg::Marker>("car_pose_marker", 10);
    midpoints_publisher_ = create_publisher<visualization_msgs::msg::Marker>("midpoint_markers", 10);
    look_ahead_publisher_ = create_publisher<visualization_msgs::msg::Marker>("look_ahead_marker", 10);
    spline_publisher_ = create_publisher<nav_msgs::msg::Path>("spline_path", 10);


    rclcpp::CallbackGroup::SharedPtr callback_group{ create_callback_group(rclcpp::CallbackGroupType::Reentrant) };
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group;

    // Set subscribers
    pose_subscriber_ = create_subscription<custom_msgs::msg::PoseMsg>("pose", 10, std::bind(&Telemetry::pose_callback, this, _1), options);
    map_subscriber_ = create_subscription<custom_msgs::msg::LocalMapMsg>("local_map", 10, std::bind(&Telemetry::map_callback, this, _1), options);
    waypoints_subscriber_ = create_subscription<custom_msgs::msg::WaypointsMsg>("waypoints", 10, std::bind(&Telemetry::waypoints_callback, this, _1), options);
    look_ahead_subscriber_ = create_subscription<custom_msgs::msg::Point2Struct>("pp_target_point", 10, std::bind(&Telemetry::look_ahead_callback, this, _1), options);
}

Telemetry::~Telemetry() {}

void Telemetry::pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr msg) {
    visualization_msgs::msg::Marker car_marker{};
    car_marker.header.frame_id = "map";
    car_marker.header.stamp = now();
    car_marker.ns = "my_ns";
    car_marker.id = 0;

    car_marker.type = visualization_msgs::msg::Marker::CUBE;
    car_marker.action = visualization_msgs::msg::Marker::ADD;

    // Minus y variable is for left handed system
    car_marker.pose.position.x = msg->position.y;
    car_marker.pose.position.y = msg->position.x;
    car_marker.pose.position.z = 0.7;

    tf2::Quaternion car_orientation;
    car_orientation.setRPY(0.0, 0.0, M_PI / 2 - msg->theta);
    car_orientation.normalize();

    car_marker.pose.orientation = tf2::toMsg(car_orientation);

    car_marker.scale.x = 2.0;
    car_marker.scale.y = 1.22;
    car_marker.scale.z = 0.9;

    car_marker.color.r = 0.839;
    car_marker.color.g = 0.224;
    car_marker.color.b = 0.082;
    car_marker.color.a = 0.7;

    car_publisher_->publish(car_marker);
}

void Telemetry::look_ahead_callback(const custom_msgs::msg::Point2Struct::SharedPtr msg) {
    visualization_msgs::msg::Marker look_ahead{};
    look_ahead.header.frame_id = "map";
    look_ahead.header.stamp = now();
    look_ahead.ns = "my_ns";
    look_ahead.id = 1;

    look_ahead.type = visualization_msgs::msg::Marker::SPHERE;
    look_ahead.action = visualization_msgs::msg::Marker::ADD;

    look_ahead.pose.orientation.w = 1.0;
    look_ahead.scale.x = 0.6;
    look_ahead.scale.y = 0.6;
    look_ahead.scale.z = 0.6;

    look_ahead.color.r = 204.0 / 255.0;
    look_ahead.color.g = 219.0 / 255.0;
    look_ahead.color.b = 136.0 / 255.0;
    look_ahead.color.a = 0.5;

    look_ahead.pose.position.x = msg->y;
    look_ahead.pose.position.y = msg->x;

    look_ahead_publisher_->publish(look_ahead);
}

void Telemetry::map_callback(const custom_msgs::msg::LocalMapMsg::SharedPtr msg) {
    int id{ 5 };
    visualization_msgs::msg::MarkerArray cones_array{};
    visualization_msgs::msg::Marker cone_marker{};

    cone_marker.header.frame_id = "map";
    cone_marker.header.stamp = now();
    cone_marker.ns = "my_ns";
    cone_marker.action = cone_marker.ADD;
    cone_marker.type = cone_marker.CYLINDER;

    cone_marker.color.a = 1.0;
    cone_marker.scale.x = 0.3;
    cone_marker.scale.y = 0.3;
    cone_marker.scale.z = 0.4;
    cone_marker.pose.position.z = 0.2;
    cone_marker.pose.orientation.x = 0.0;
    cone_marker.pose.orientation.y = 0.0;
    cone_marker.pose.orientation.z = 0.0;
    cone_marker.pose.orientation.w = 1.0;

    for (auto cone : msg->local_map)
    {
        cone_marker.id = id;
        cone_marker.pose.position.x = cone.coords.y;
        cone_marker.pose.position.y = cone.coords.x;
        switch (cone.color)
        {
        case 0:
            cone_marker.color.r = 1.0;
            cone_marker.color.g = 1.0;
            cone_marker.color.b = 0.0;
            break;
        case 1:
            cone_marker.color.r = 0.0;
            cone_marker.color.g = 0.0;
            cone_marker.color.b = 0.8;
            break;
        case 2:
            cone_marker.color.r = 247.0 / 250.0;
            cone_marker.color.g = 140.0 / 250.0;
            cone_marker.color.b = 25.0 / 250.0;
            break;
        case 3:
            cone_marker.color.r = 117.0 / 250.0;
            cone_marker.color.g = 59.0 / 250.0;
            cone_marker.color.b = 29.0 / 250.0;
            break;
        default:
            RCLCPP_WARN(get_logger(), "SlamFromFile() -> Invalid cone color encountered when reading map");
            break;
        }
        cones_array.markers.push_back(cone_marker);
        id++;
    }
    landmark_publisher_->publish(cones_array);
}

void Telemetry::waypoints_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr msg) {
    visualization_msgs::msg::Marker points{};
    nav_msgs::msg::Path path{};

    path.header.frame_id = "map";
    path.header.stamp = now();

    points.header.frame_id = "map";
    points.header.stamp = now();
    points.ns = "my_ns";
    points.id = 2;

    points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points.action = visualization_msgs::msg::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;

    points.color.r = 76.0 / 255.0;
    points.color.g = 66.0 / 255.0;
    points.color.b = 133.0 / 255.0;
    points.color.a = 1.0;

    constexpr int spline_size{ 40 };
    path_planning::PointsArray targets{ msg->waypoints.size(), 2 };

    for (size_t i{ 0 }; i < msg->waypoints.size(); i++)
    {
        geometry_msgs::msg::Point p;
        p.x = msg->waypoints[i].y;
        p.y = msg->waypoints[i].x;
        p.z = 0.0;

        points.points.push_back(p);
        targets(i, 0) = p.x;
        targets(i, 1) = p.y;
    }

    path_planning::ArcLengthSpline spline{targets, path_planning::BoundaryCondition::Anchored};
    path_planning::PointsData spline_data{ spline.getSplineData(spline_size) };
    for (long int i{ 0 }; i < spline_data.rows(); i++)
    {
        geometry_msgs::msg::PoseStamped pose{};
        pose.pose.position.x = spline_data(i, 0);
        pose.pose.position.y = spline_data(i, 1);
        pose.pose.position.z = 0.0;

        tf2::Quaternion pose_orientation;
        pose_orientation.setRPY(0.0, 0.0, M_PI / 2 - spline_data(i, 2));
        pose_orientation.normalize();
        pose.pose.orientation = tf2::toMsg(pose_orientation);

        path.poses.push_back(pose);
    }

    midpoints_publisher_->publish(points);
    spline_publisher_->publish(path);
}
} // ns_telemetry

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto options{ rclcpp::ExecutorOptions() };
    rclcpp::executors::MultiThreadedExecutor executor{ options, 3 };

    auto telemetry_node = std::make_shared<ns_telemetry::Telemetry>();
    executor.add_node(telemetry_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
