#include <chrono>
#include <functional>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class Visualization : public rclcpp::Node {
private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string share_dir_;
    std::fstream track_file_;
    std::fstream car_path_file_;

    double current_x;
    double current_y;
    double next_x;
    double next_y;

    void loadTrack() {
        int color;
        double cone_x;
        double cone_y;
        int id{ 0 };

        visualization_msgs::msg::MarkerArray cones{};
        visualization_msgs::msg::Marker cone{};

        cone.header.frame_id = "map";
        cone.header.stamp = now();
        cone.ns = "my_ns";
        cone.action = cone.ADD;
        cone.type = cone.CYLINDER;

        cone.color.a = 1.0;
        cone.scale.x = 0.2;
        cone.scale.y = 0.2;
        cone.scale.z = 0.5;
        cone.pose.position.z = 0.15;
        cone.pose.orientation.x = 0.0;
        cone.pose.orientation.y = 0.0;
        cone.pose.orientation.z = 0.0;
        cone.pose.orientation.w = 1.0;

        while (!track_file_.eof())
        {
            std::string line;
            std::getline(track_file_, line);
            std::stringstream stream{ line };

            stream >> cone_y >> cone_x >> color;

            cone.id = id;
            cone.pose.position.x = cone_x;
            cone.pose.position.y = cone_y;
            if (color == 1)
            {
                cone.color.r = 0.0;
                cone.color.g = 0.0;
                cone.color.b = 1.0;
            }
            else
            {
                cone.color.r = 1.0;
                cone.color.g = 1.0;
                cone.color.b = 0.0;
            }
            cones.markers.push_back(cone);
            id++;
        }

        track_pub_->publish(cones);
    }

    void TimerCallback() {
        std::string line;
        std::getline(car_path_file_, line);
        std::stringstream stream(line);
        stream >> next_x >> next_y;

        visualization_msgs::msg::Marker car_marker{};
        car_marker.header.frame_id = "map";
        car_marker.header.stamp = now();
        car_marker.ns = "my_ns";
        car_marker.id = 0;

        car_marker.type = visualization_msgs::msg::Marker::CUBE;
        car_marker.action = visualization_msgs::msg::Marker::ADD;

        car_marker.pose.position.x = current_x;
        car_marker.pose.position.y = current_y;
        car_marker.pose.position.z = 0.7;

        double yaw{ std::atan2(next_y - current_y, next_x - current_x) };
        tf2::Quaternion car_orientation;
        car_orientation.setRPY(0.0, 0.0, yaw);
        car_orientation.normalize();

        car_marker.pose.orientation = tf2::toMsg(car_orientation);

        car_marker.scale.x = 2.4;
        car_marker.scale.y = 1.5;
        car_marker.scale.z = 1.4;

        car_marker.color.r = 0.839;
        car_marker.color.g = 0.224;
        car_marker.color.b = 0.082;
        car_marker.color.a = 1.0;

        RCLCPP_INFO_STREAM(get_logger(), "Car Pose: " << current_x << ", " << current_y << ", " << yaw);

        pub_->publish(car_marker);

        current_x = next_x;
        current_y = next_y;
        if (car_path_file_.eof())
        {
            rclcpp::shutdown();
        }
    }
public:
    explicit Visualization() : Node("slam_visual") {
        double fps{ 10 };

        share_dir_ = ament_index_cpp::get_package_share_directory("slam_visual");

        track_file_.open(share_dir_ + "/test_tracks/trackdrive.txt");
        car_path_file_.open(share_dir_ + "/test_tracks/trackdrive_midpoints.txt");

        std::string line;
        std::getline(car_path_file_, line);
        RCLCPP_INFO(get_logger(), line.c_str());

        std::stringstream stream(line);

        stream >> current_x;
        stream >> current_y;

        pub_ = create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        track_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);

        loadTrack();
        timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / fps)), std::bind(&Visualization::TimerCallback, this));
    }

    ~Visualization() {
        track_file_.close();
        car_path_file_.close();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    Visualization node{};
    rclcpp::spin(node.get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}