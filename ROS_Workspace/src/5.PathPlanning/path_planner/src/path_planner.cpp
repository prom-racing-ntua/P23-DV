#include "path_planner.hpp"

Path_Planner_Node::Path_Planner_Node():Node("path_planning"), waymaker() {
    //waymaker = Triangulation();
    parameter_load();

    waymaker.init
    (
        get_parameter("maximum_angle").as_int(),
        get_parameter("maximum_edge_angle").as_int(),
        get_parameter("maximum_distance").as_int(),
        0, 0,
        get_parameter("target_depth").as_int(),
        get_parameter("same_edge_penalty").as_int(),
        get_parameter("length_penalty").as_double(),
        get_parameter("angle_penalty").as_double(),
        get_parameter("total_length_reward").as_double()
    );
    sub_mapper = this->create_subscription<custom_msgs::msg::LocalMapMsg>("local_map", 10, std::bind(&Path_Planner_Node::mapping_callback, this, _1));
    //sub_odometry = this->create_subscription<custom_msgs::msg::PoseMsg>("Pose",10,std::bind(&Path_Planner_Node::pose_callback, this, _1));
    pub_waypoints = this->create_publisher<custom_msgs::msg::WaypointsMsg>("waypoints", 10);
}

void Path_Planner_Node::parameter_load() {
    selection_radius_small = declare_parameter<int>("selection_radius_small", 5);
    selection_radius_big = declare_parameter<int>("selection_radius_big", 10);
    selection_angle = declare_parameter<int>("selection_angle", 90);
    declare_parameter<int>("maximum_angle", 90);
    declare_parameter<int>("maximum_edge_angle", 90);
    declare_parameter<int>("maximum_distance", 5);
    declare_parameter<int>("target_depth", 10);

    declare_parameter<int>("same_edge_penalty", 10);
    declare_parameter<float>("length_penalty", 0.1);
    declare_parameter<float>("angle_penalty", 0.1);
    declare_parameter<float>("total_length_reward", 0.075);
}

std::vector<Cone> select_cones_by_dist_and_angle(const std::vector<Cone>& full_map, const Point& position, const Point& direction, int radius_small, int radius_big, int angle) {
    std::vector<Cone> selected;
    selected.reserve(full_map.size());
    for (Cone cone : full_map)
    {
        if (cone.color != 0 && cone.color != 1)
        {
            continue;
        }
        if (CGAL::squared_distance(cone.coords, position) <= radius_small * radius_small)
        {
            selected.push_back(cone);
        }

        else if (std::abs(angle_point_2(direction, position, cone.coords)) < angle && CGAL::squared_distance(cone.coords, position) <= radius_big * radius_big)
        {
            selected.push_back(cone);
        }
    }
    selected.shrink_to_fit();
    return selected;
}

void Path_Planner_Node::mapping_callback(const custom_msgs::msg::LocalMapMsg::SharedPtr msg) {
    int cone_count = msg->cone_count;
    if (cone_count < 3) return;

    std::vector<Cone> full_map, local_map;
    full_map.reserve(cone_count);
    for (custom_msgs::msg::ConeStruct cone : msg->local_map)
    {
        full_map.push_back(Cone(Point(cone.coords.x, cone.coords.y), cone.color));
    }
    Point current_position(msg->pose.position.x, msg->pose.position.y);
    float theta = msg->pose.theta;
    Point current_direction(current_position.x() + std::cos(theta), current_position.y() + std::sin(theta));
    local_map = select_cones_by_dist_and_angle(full_map, current_position, current_direction, selection_radius_small, selection_radius_big, selection_angle);
    std::cout << local_map.size() << std::endl;
    std::pair<std::vector<Point>, int> batch_output = waymaker.new_batch(local_map, current_position, Direction_2(Segment_2(current_position, current_direction)));
    std::vector<Point> waypoints(batch_output.first);
    std::cout << "score: " << batch_output.second << '\n';

    custom_msgs::msg::WaypointsMsg for_pub;
    for_pub.count = waypoints.size();
    std::vector<custom_msgs::msg::Point2Struct> waypoints_ros;
    waypoints_ros.reserve(waypoints.size());
    custom_msgs::msg::Point2Struct sample;
    for (Point point : waypoints)
    {
        sample.x = point.x();
        sample.y = point.y();
        waypoints_ros.push_back(sample);
    }

    for_pub.waypoints = waypoints_ros;
    pub_waypoints->publish(for_pub);
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Path_Planner_Node>());
    rclcpp::shutdown();
    return 0;
}