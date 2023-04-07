#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "custom_msgs/msg/local_map_msg.hpp"
#include "custom_msgs/msg/pose_msg.hpp"
#include "custom_msgs/msg/waypoints_msg.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace ns_telemetry
{
class Telemetry: public rclcpp::Node {
private:
    rclcpp::Subscription<custom_msgs::msg::PoseMsg>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<custom_msgs::msg::LocalMapMsg>::SharedPtr map_subscriber_;
    rclcpp::Subscription<custom_msgs::msg::WaypointsMsg>::SharedPtr waypoints_subscriber_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    void pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr msg);
    void map_callback(const custom_msgs::msg::LocalMapMsg::SharedPtr msg);
    void waypoints_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr msg);

public:
    Telemetry();
    ~Telemetry();
};


Telemetry::Telemetry(): Node("telemetry") {
    using std::placeholders::_1;
    // Set publishers
    landmark_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("landmark_marker_array", 10);
    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>("car_pose_marker", 10);

    rclcpp::CallbackGroup::SharedPtr callback_group{ create_callback_group(rclcpp::CallbackGroupType::Reentrant) };
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group;

    // Set subscribers
    pose_subscriber_ = create_subscription<custom_msgs::msg::PoseMsg>("pose", 10, std::bind(&Telemetry::pose_callback, this, _1), options);
    map_subscriber_ = create_subscription<custom_msgs::msg::LocalMapMsg>("local_map", 10, std::bind(&Telemetry::map_callback, this, _1), options);
    waypoints_subscriber_ = create_subscription<custom_msgs::msg::WaypointsMsg>("waypoints", 10, std::bind(&Telemetry::waypoints_callback, this, _1), options);
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
    car_marker.color.a = 1.0;

    marker_publisher_->publish(car_marker);
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
    visualization_msgs::msg::Marker delete_path{};
    visualization_msgs::msg::Marker delete_points{};

    delete_path.header.frame_id = "map";
    delete_path.header.stamp = now();
    delete_path.ns = "my_ns";
    delete_path.id = 1;
    delete_path.action = visualization_msgs::msg::Marker::DELETE;

    delete_points.header.frame_id = "map";
    delete_points.header.stamp = now();
    delete_points.ns = "my_ns";
    delete_points.id = 2;
    delete_points.action = visualization_msgs::msg::Marker::DELETE;

    visualization_msgs::msg::Marker path_marker{};
    visualization_msgs::msg::Marker points{};

    path_marker.header.frame_id = "map";
    path_marker.header.stamp = now();
    path_marker.ns = "my_ns";
    path_marker.id = 1;

    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.3;

    path_marker.color.r = 0.8980;
    path_marker.color.g = 0.0078;
    path_marker.color.b = 0.9922;
    path_marker.color.a = 1.0;

    points.header.frame_id = "map";
    points.header.stamp = now();
    points.ns = "my_ns";
    points.id = 2;

    points.type = visualization_msgs::msg::Marker::POINTS;
    points.action = visualization_msgs::msg::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.4;
    points.scale.y = 0.4;

    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;
    points.color.a = 1.0;

    for (auto point : msg->waypoints)
    {
        geometry_msgs::msg::Point p;
        p.x = point.y;
        p.y = point.x;
        p.z = 0.0;

        path_marker.points.push_back(p);
        points.points.push_back(p);
    }
    marker_publisher_->publish(points);
    marker_publisher_->publish(path_marker);
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
