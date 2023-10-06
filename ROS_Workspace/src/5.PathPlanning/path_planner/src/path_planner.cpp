#include "path_planner.hpp"

Path_Planner_Node::Path_Planner_Node():Node("path_planning"), waymaker(), total_execution_time(0), last_length(0), average_angle(0), prev_lap(0) {
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
        get_parameter("total_length_reward").as_double(),
        get_parameter("filtering_threshold").as_int()
    );
    
    sub_mapper = this->create_subscription<custom_msgs::msg::LocalMapMsg>("local_map", 10, std::bind(&Path_Planner_Node::mapping_callback, this, _1));
    
    pub_waypoints = this->create_publisher<custom_msgs::msg::WaypointsMsg>("waypoints", 10);

    save_total_path.open("src/6.Controls/pid_pp_controller/data/midpoints_from_pp.txt");
}

void Path_Planner_Node::parameter_load() {
    selection_radius_small = declare_parameter<int>("selection_radius_small", 5);
    selection_radius_big = declare_parameter<int>("selection_radius_big", 10);
    selection_angle = declare_parameter<int>("selection_angle", 90);
    declare_parameter<int>("maximum_angle", 90);
    declare_parameter<int>("maximum_edge_angle", 90);
    declare_parameter<int>("maximum_distance", 5);
    declare_parameter<int>("target_depth", 10);
    declare_parameter<int>("filtering_threshold", 100);

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

std::pair<int, int> count_cones_by_color(const std::vector<Cone> &local_map)
{
    int counter[] = {0, 0, 0, 0};

    for (Cone cone : local_map)
    {
        counter[cone.color]++;
    }
    return std::make_pair(counter[0], counter[1]);
}

void Path_Planner_Node::mapping_callback(const custom_msgs::msg::LocalMapMsg::SharedPtr msg) {
    rclcpp::Time starting_time = this->now();
    int cone_count = msg->local_map.size();
    std::vector<Cone> full_map, local_map;
    double x0,y0;
    full_map.reserve(cone_count);
    for (custom_msgs::msg::ConeStruct cone : msg->local_map)
    {
        x0 = cone.coords.x;
        y0 = cone.coords.y;
        
        full_map.push_back(Cone(Point(x0, y0), cone.color));
    }
    //full_map.push_back(Cone(Point(0, +1.5), 1)); //adjusting for big orange cones at start line
    //full_map.push_back(Cone(Point(0, -1.5), 0));
    Point current_position(msg->pose.position.x, msg->pose.position.y);
    float theta = msg->pose.theta; //adjustment for reversed y-axis
    Point current_direction(current_position.x() + std::cos(theta), current_position.y() + std::sin(theta));
    local_map = select_cones_by_dist_and_angle(full_map, current_position, current_direction, selection_radius_small, selection_radius_big, selection_angle);

    if (local_map.size() < 3)
        return;
    std::pair<int, int> b_y_count = count_cones_by_color(local_map);
    if(b_y_count.first < 1 || b_y_count.second < 1)
        return;

    //std::cout<<"("<<current_direction.x()<<","<<current_direction.y()<<")"<<std::endl;
    std::pair<std::vector<Point>, int> batch_output = waymaker.new_batch(local_map, current_position, Direction_2(Segment_2(current_position, current_direction)));

    bool lap_change = prev_lap!=msg->lap_count;
    prev_lap = msg->lap_count;

    if(this->has_completed_total_path)
    {
        if(lap_change)
        {
            finalized.initial_v_x = msg->pose.velocity_state.global_index==0?-1: msg->pose.velocity_state.velocity_x;
            finalized.lap_count = msg->lap_count;

            pub_waypoints->publish(finalized);
      
            RCLCPP_INFO_STREAM(get_logger(), "Lap changed. Published full path.");
        }
        return;
    }


    std::vector<Point> waypoints(batch_output.first);
    if (waypoints.size() == 0)
    {
        return;
    }
    average_angle = std::max(average_angle, this->get_angle_avg(waypoints));
    if(batch_output.second>=50 and 0)
    {
        std::cout<<">>> MALAKIA <<<"<<std::endl;
        for(Cone cone:local_map)
        {
            if(cone.color==0)std::cout<<"("<<cone.coords.x()<<","<<cone.coords.y()<<"),";
        }
        std::cout<<std::endl;
        for(Cone cone:local_map)
        {
            if(cone.color==1)std::cout<<"("<<cone.coords.x()<<","<<cone.coords.y()<<"),";
        }
        std::cout<<std::endl;
        for(Point point:batch_output.first)
        {
            std::cout<<"("<<point.x()<<","<<point.y()<<"),";
        }
        std::cout<<std::endl<<"Position: ("<<current_position.x()<<","<<current_position.y()<<")"<<std::endl;
        std::cout<<"Direction: ("<<current_direction.x()<<","<<current_direction.y()<<")"<<std::endl;
        std::cout<<"-------------"<<std::endl;
        //exit(1);
    }
    

    custom_msgs::msg::WaypointsMsg for_pub;
    for_pub.count = waypoints.size();
    std::vector<custom_msgs::msg::Point2Struct> waypoints_ros;
    waypoints_ros.reserve(waypoints.size());
    custom_msgs::msg::Point2Struct sample;
    for (Point point : waypoints)
    {
        //std::cout<<"("<<point.x()<<","<<point.y()<<"),";
        sample.x = point.x();
        sample.y = point.y();
        waypoints_ros.push_back(sample);
    }
    //std::cout<<std::endl;
    for_pub.waypoints = waypoints_ros;
    if(last_length==0)
    {
        last_path = for_pub;
        last_length = this->get_length(waypoints);
        last_position = current_position;
    }
    else
    {
        float l = this->get_length(waypoints);
        if(l + std::sqrt(CGAL::squared_distance(current_position, last_position))<0.90*last_length)
        {
            std::cout<<waymaker.get_batch_number()<<" Kept last: Last = "<<last_length<<" Dist = "<<std::sqrt(CGAL::squared_distance(current_position, last_position))<<" Current = "<<l<<std::endl;
            rclcpp::Duration total_time = this->now() - starting_time;
            total_execution_time += total_time.nanoseconds() / 1000000.0;
            std::cout << "Time of Execution: " << total_time.nanoseconds() / 1000000.0 << " ms." << std::endl;
            return ;
        }
    }
    if(batch_output.second > 90)
    {
        if(last_length - std::sqrt(CGAL::squared_distance(current_position, Point(last_path.waypoints[0].x, last_path.waypoints[0].y))) > 4)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Path is faulty but last path is viable...");
            return ;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Best path is faulty. Aborting...");
            exit(1); //Initiates ABS. If there is a safer method, it should be prefered.
        }
    }
    else
    {
        last_path = for_pub;
        last_length = this->get_length(waypoints);
        last_position = current_position;
    }

    if(msg->lap_count != 0)
    {
        if(added_waypoints.size() == 0)
        {
            added_waypoints.push_back(waypoints[1]); //waypoints[1] == midpoints[0]
        }
        else
        {
            if(CGAL::squared_distance(waypoints[1], added_waypoints[added_waypoints.size() - 1]) > 2 * 2)
            {
                added_waypoints.push_back(waypoints[1]);
            }
            if(added_waypoints.size() != 1)
            {
                // Point vect1 = Point(added_waypoints[0].x() - waypoints[waypoints.size()-2].x(), added_waypoints[0].y() - waypoints[waypoints.size()-2].y());
                // Point vect2 = Point(-added_waypoints[0].x() + waypoints[waypoints.size()-1].x(), -added_waypoints[0].y() + waypoints[waypoints.size()-1].y());

                // float norm1 = std::sqrt(squared_distance(vect1, Point(0, 0)));
                // float norm2 = std::sqrt(squared_distance(vect2, Point(0, 0)));
                // float dist = std::sqrt(squared_distance(waypoints[waypoints.size()-1], waypoints[waypoints.size()-2]));

                // float cosine = (vect1.x()*vect2.x() + vect1.y()*vect2.y()) / (norm1 * norm2);
                float mn = DBL_MAX, loc;
                int mn_idx = 0;

                for(int i=1; i<waypoints.size(); i++)
                {   
                    loc = std::sqrt(CGAL::squared_distance(waypoints[i], added_waypoints[0]));
                    mn = std::min(mn, loc);
                    mn_idx = i;
                }

                std::cout<<"Min dist to first is: "<<mn<<std::endl;

                if(mn < 2  && added_waypoints.size()>10)
                {
                    std::vector<custom_msgs::msg::Point2Struct> waypoints_ros_final;

                    for(int i=1; i<mn_idx; i++)
                    {
                        if(CGAL::squared_distance(waypoints[i], added_waypoints[added_waypoints.size() - 1]) > 2 * 2)
                            added_waypoints.push_back(waypoints[i]);
                    }
                    this->has_completed_total_path = 1;

                    for_pub.count = added_waypoints.size();
                    waypoints_ros_final.reserve(added_waypoints.size() + 5);

                    save_total_path<<added_waypoints.size();

                    for(int i = 0; i<added_waypoints.size(); i++) //create finalized message
                    {
                        save_total_path<<added_waypoints[i].x()<<" "<<added_waypoints[i].y()<<std::endl;
                        // std::cout<<"("<<added_waypoints[i].x()<<", "<<added_waypoints[i].y()<<"),";
                        sample.x = added_waypoints[i].x();
                        sample.y = added_waypoints[i].y();
                        waypoints_ros_final.push_back(sample);
                    }
                    for(int i=0; i < 5; i++)
                    {
                        waypoints_ros_final.push_back(waypoints_ros_final[i]);
                        save_total_path<<added_waypoints[i].x()<<" "<<added_waypoints[i].y()<<std::endl;
                    }
                    // std::cout<<std::endl;
                    finalized.waypoints = waypoints_ros_final;
                    finalized.is_out_of_map = 0;
                    finalized.count = waypoints_ros_final.size();

                    waypoints_ros.erase(waypoints_ros.begin() + mn_idx, waypoints_ros.end());

                    for(int i = 0; i<added_waypoints.size(); i++) // create curr message, that puts the added wayps at the end of the existing path
                    {
                        if(CGAL::squared_distance(added_waypoints[i], waypoints[0])<1)break;
                        waypoints_ros.push_back(waypoints_ros_final[i]);
                    }
                    for_pub.waypoints = waypoints_ros;
                    // for(int i=0; i<waypoints_ros.size(); i++)
                    // {
                    //     std::cout<<"("<<waypoints_ros[i].x<<","<<waypoints_ros[i].y<<"),";
                    // }
                    // std::cout<<std::endl;

                    RCLCPP_WARN(get_logger(), "Path completed. Will stop making new paths.");
                }
            }
        }
    }

    for_pub.is_out_of_map = waymaker.out_of_convex;
    for_pub.initial_v_x = msg->pose.velocity_state.global_index==0?-1: msg->pose.velocity_state.velocity_x;
    for_pub.lap_count = msg->lap_count;
//    for_pub.chain_index = (waymaker.get_batch_number());
    pub_waypoints->publish(for_pub);


    rclcpp::Time ending_time = this->now();
    std::cout << waymaker.get_batch_number() << " score: " << batch_output.second << " no of midpoints: " << waypoints.size() << std::endl;
    rclcpp::Duration total_time = this->now() - starting_time;
//    fprintf(timestamp_log, "%f\t%d\n", ending_time.seconds(), waymaker.get_batch_number());

    total_execution_time += total_time.nanoseconds() / 1000000.0;
    std::cout << "Time of Execution: " << total_time.nanoseconds() / 1000000.0 << " ms." << std::endl;
}

Path_Planner_Node::~Path_Planner_Node() {
    std::cout << "Average execution time: " << total_execution_time / waymaker.get_batch_number() << std::endl;
    std::cout << "Max angle: "<<average_angle <<std::endl;
}

float Path_Planner_Node::get_length(std::vector<Point> path)const
{
    float l = 0;
    for(int i=1; i<path.size(); i++)
    {
        l += std::sqrt(CGAL::squared_distance(path[i],path[i-1]));
    }
    return l;
}

float Path_Planner_Node::get_angle_avg(std::vector<Point> path)const
{
    //if(path.size()-2==0)return 0;
    float l = 0;
    float mx=0;
    for(int i=1; i<path.size()-1; i++)
    {
        l += std::abs(180-angle_point_2(path[i-1], path[i], path[i+1]));
        mx = std::max(mx, float(std::abs(180-angle_point_2(path[i-1], path[i], path[i+1]))));
        //std::cout<<std::abs(180-angle_point_2(path[i-1], path[i], path[i+1]))<<", ";
    }
    //std::cout<<"Average angle = "<<l/(path.size()-2)<<std::endl;
    return mx;
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Path_Planner_Node>());
    rclcpp::shutdown();
    return 0;
}
