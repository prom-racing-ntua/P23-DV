#include "rclcpp/rclcpp.hpp"
#include "rmw/qos_profiles.h"
#include <rclcpp/qos.hpp>

#include "lifecycle_pathplanning_node.hpp"

namespace path_planner
{
    LifecyclePathPlanner::LifecyclePathPlanner() : LifecycleNode("path_planner"), waymaker(), total_execution_time(0), mission(MISSION::UNLOCK), prev_lap(-1), has_completed_total_path(0), last_length(0)
    {
        loadParameters();
        RCLCPP_WARN(get_logger(), "\n-- Path Planner Node Created");
    }

    void LifecyclePathPlanner::loadParameters()
    {
        declare_parameter<int>("selection_radius_small", 5);
        declare_parameter<int>("selection_radius_big", 10);
        declare_parameter<int>("selection_angle", 90);
        declare_parameter<int>("maximum_angle", 90);
        declare_parameter<int>("maximum_edge_angle", 90);
        declare_parameter<int>("maximum_distance", 5);
        declare_parameter<int>("target_depth", 10);
        declare_parameter<int>("filtering_threshold", 100);

        declare_parameter<int>("same_edge_penalty", 10);
        declare_parameter<float>("length_penalty", 0.1);
        declare_parameter<float>("angle_penalty", 0.1);
        declare_parameter<float>("total_length_reward", 0.075);

        declare_parameter<int>("mission", 4);
    }

    std::vector<Cone> select_cones_by_dist_and_angle(const std::vector<Cone> &full_map, const Point &position, const Point &direction, int radius_small, int radius_big, int angle, bool skip_orange)
    {
        std::vector<Cone> selected;
        selected.reserve(full_map.size());
        for (Cone cone : full_map)
        {
            if (cone.color != 0 && cone.color != 1 && skip_orange)
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
        // fs.close();
        selected.shrink_to_fit();
        return selected;
    }

    std::tuple<int, int, int> count_cones_by_color(const std::vector<Cone> &local_map)
    {
        int counter[] = {0, 0, 0, 0};

        for (Cone cone : local_map)
        {
            counter[cone.color]++;
        }
        return std::make_tuple(counter[0], counter[1], counter[2]);
    }

    std::vector<custom_msgs::msg::Point2Struct> LifecyclePathPlanner::skidpad_midpoints(const std::vector<custom_msgs::msg::ConeStruct> &local_map, const Point &current_position,  int local_map_size, int lap)
    {
        Cone cones[local_map_size];
        double x0, y0;
        for(int i=0; i<local_map_size; i++)
        {
            x0 = local_map[i].coords.x;
            y0 = local_map[i].coords.y;
            // std::cout<<i<<"= ("<<x0<<", "<<y0<<")\n";
            cones[i] = Cone(Point(x0, y0), local_map[i].color);
        }
        // std::cout<<std::endl;

        std::vector<custom_msgs::msg::Point2Struct> waypoints_ros;
        Point waypoint;
        int midpoints_size, initial_waypoints_size;
        int *outer_idxs, *inner_idxs;
        int closest_index = -1;
        float closest_distance = DBL_MAX;
        int wa, wb=1;
        switch (lap)
        {
        case 0:
            {
            midpoints_size = 4 + 8;
            initial_waypoints_size = 4;
            wa = 2;
            outer_idxs = new int[midpoints_size]{59, 61, 28, 44,
                                                 30, 57, 56, 55, 54, 53, 52, 51
            };
            inner_idxs = new int[midpoints_size]{58, 60, 45,  9, 
                                                  7,  6,  5,  4,  3,  2,  1,  0
            };
            }break;
        case 1:
            {
            midpoints_size = 15 + 8;
            initial_waypoints_size = 15;
            wa = 3;
            outer_idxs = new int[midpoints_size]{30, 57, 56, 55, 54, 53, 52, 51,
                                                 50, 49, 48, 47, 46, 45, 44, 
                                                 30, 57, 56, 55, 54, 53, 52, 51
            };
            inner_idxs = new int[midpoints_size]{ 7,  6,  5,  4,  3,  2,  1,  0,
                                                 15, 14, 13, 12, 11, 10,  9,
                                                  7,  6,  5,  4,  3,  2,  1,  0
            };
            }break;
        case 2:
            {
            midpoints_size = 15 + 8;
            initial_waypoints_size = 15;
            wa = 3;
            outer_idxs = new int[midpoints_size]{30, 57, 56, 55, 54, 53, 52, 51,
                                                 50, 49, 48, 47, 46, 45, 44, 
                                                  7, 16, 17, 18, 19, 20, 21, 22
            };
            inner_idxs = new int[midpoints_size]{ 7,  6,  5,  4,  3,  2,  1,  0,
                                                 15, 14, 13, 12, 11, 10,  9,
                                                 30, 31, 32, 33, 34, 35, 36, 37
            };
            }break;
        case 3:
            {
            midpoints_size = 15 + 8;
            initial_waypoints_size = 15;
            wa = 3;
            outer_idxs = new int[midpoints_size]{ 7, 16, 17, 18, 19, 20, 21, 22,
                                                 23, 24, 25, 26, 27, 28,  9,
                                                  7, 16, 17, 18, 19, 20, 21, 22
            };
            inner_idxs = new int[midpoints_size]{30, 31, 32, 33, 34, 35, 36, 37,
                                                 38, 39, 40, 41, 42, 43, 44, 
                                                 30, 31, 32, 33, 34, 35, 36, 37
            };
            }break;
        case 4:
            {
            midpoints_size = 15 + 6;
            initial_waypoints_size = 15;
            wa = 3;
            outer_idxs = new int[midpoints_size]{ 7, 16, 17, 18, 19, 20, 21, 22,
                                                 23, 24, 25, 26, 27, 28,  9,
                                                  7, 57, 62, 64, 67, 68
            };
            inner_idxs = new int[midpoints_size]{30, 31, 32, 33, 34, 35, 36, 37,
                                                 38, 39, 40, 41, 42, 43, 44,
                                                 30, 16, 63, 65, 66, 71
            };
            }break;
        case 5:
            {
            midpoints_size = 6;
            initial_waypoints_size = 6;
            wa = 1;
            outer_idxs = new int[midpoints_size]{ 7, 57, 62, 64, 67, 68};
            inner_idxs = new int[midpoints_size]{30, 16, 63, 65, 66, 71};
            }break;
        default:
            break;
        }
        custom_msgs::msg::Point2Struct sample;
        
        float distance;
        for(int i=last_visited_index; i<(last_visited_index==0 ? initial_waypoints_size/2 : initial_waypoints_size); i++)
        {
            waypoint = my_edge(cones[outer_idxs[i]], cones[inner_idxs[i]]).midpoint();
            distance = std::sqrt(std::pow(current_position.x() - waypoint.x(), 2) + std::pow(current_position.y() - waypoint.y(), 2));
            if(distance < closest_distance)
            {
                closest_index = i;
                closest_distance = distance;
            }
        }

        if(closest_index == -1)
        {
            closest_index = last_visited_index;
        }
        else
        {
            last_visited_index = closest_index;
        }

        // std::cout<<closest_index<<" "<<closest_distance<<" "<<lap<<" "<<std::endl;

        waypoints_ros.reserve(midpoints_size - closest_index + 1);
        sample.x = current_position.x();
        sample.y = current_position.y();
        waypoints_ros.push_back(sample);

        for(int i=closest_index; i<midpoints_size; i++)
        {
            waypoint = my_edge(cones[outer_idxs[i]], cones[inner_idxs[i]]).weighted_midpoint(wa, wb);

            sample.x = waypoint.x();
            sample.y = waypoint.y();
            waypoints_ros.push_back(sample);
        } 
        return waypoints_ros;
    }

    void LifecyclePathPlanner::mapping_callback(const custom_msgs::msg::LocalMapMsg::SharedPtr msg)
    {
        rclcpp::Time starting_time = this->now();
        
        switch (mission)
        {
        case MISSION::SKIDPAD:
            {
            int lap = static_cast<int>(msg->lap_count);
            if(lap != prev_lap)
            {
                prev_lap = lap;
                last_visited_index = 0;
            }
            std::vector<custom_msgs::msg::Point2Struct> waypoints_ros;
            waypoints_ros = skidpad_midpoints(msg->local_map, Point(msg->pose.position.x, msg->pose.position.y), msg->local_map.size(), lap);

            custom_msgs::msg::WaypointsMsg for_pub;

            for_pub.count = waypoints_ros.size();
            for_pub.waypoints = waypoints_ros;

            for_pub.initial_v_x = msg->pose.velocity_state.global_index == 0 ? -1 : msg->pose.velocity_state.velocity_x;
            for_pub.lap_count = msg->lap_count;

            for_pub.is_out_of_map = false;
            for_pub.should_exit   = false;

            for_pub.global_index =  msg->pose.velocity_state.global_index;

            pub_time_1 = this->now().nanoseconds()/1e6;
            pub_waypoints->publish(for_pub);
            pub_time_2 = this->now().nanoseconds()/1e6;

            // Timestamp Logging
            timestamp_log.log(starting_time.nanoseconds()/1e6, 0, msg->pose.velocity_state.global_index);
            timestamp_log.log((pub_time_2 + pub_time_1)/2, 1, msg->pose.velocity_state.global_index);

            return;
            }break;
        case MISSION::UNLOCK:
            RCLCPP_WARN(get_logger(), "Error in getting mission!");
            return;
            break;
        default:
            break;
        }
        int cone_count = msg->local_map.size();
        std::vector<Cone> full_map, local_map;
        double x0, y0;
        full_map.reserve(cone_count);
        for (custom_msgs::msg::ConeStruct cone : msg->local_map)
        {
            x0 = cone.coords.x;
            y0 = cone.coords.y;

            full_map.push_back(Cone(Point(x0, y0), cone.color));
        }
        
        // Current Positiom
        Point current_position(msg->pose.position.x, msg->pose.position.y);
        float theta = msg->pose.theta; // adjustment for reversed y-axis
        Point current_direction(current_position.x() + std::cos(theta), current_position.y() + std::sin(theta));

        // Local Map Selection
        bool skip_orange = (mission == MISSION::AUTOX || mission == MISSION::TRACKDRIVE);

        local_map = select_cones_by_dist_and_angle(full_map, current_position, current_direction, selection_radius_small, selection_radius_big, selection_angle, skip_orange);
        
        if (local_map.size() < 3)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Too few cones...");
            return;
        }

        std::tuple<int, int, int> color_count = count_cones_by_color(local_map);
        if((std::get<0>(color_count) < 1 || std::get<1>(color_count) < 1) && (std::get<2>(color_count) < 3))
        {
            RCLCPP_INFO_STREAM(get_logger(), "Too few cones...");
            return;
        }

        // std::cout<<"("<<current_direction.x()<<","<<current_direction.y()<<")"<<std::endl;
        std::pair<std::vector<Point>, int> batch_output = waymaker.new_batch(local_map, current_position, Direction_2(Segment_2(current_position, current_direction)));

        bool lap_change = prev_lap != msg->lap_count;
        prev_lap = msg->lap_count;

        if (this->has_completed_total_path)
        {
            if (lap_change)
            {
                finalized.initial_v_x = msg->pose.velocity_state.global_index == 0 ? -1 : msg->pose.velocity_state.velocity_x;
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
        
        
        custom_msgs::msg::WaypointsMsg for_pub;
        
        std::vector<custom_msgs::msg::Point2Struct> waypoints_ros;
        waypoints_ros.reserve(waypoints.size());
        bool should_exit = false;
        custom_msgs::msg::Point2Struct sample;
        for(int i=0; i<waypoints.size(); i++)
        {
            Point point = waypoints[i];
            sample.x = point.x();
            sample.y = point.y();
            waypoints_ros.push_back(sample);
        }
        waypoints_ros.shrink_to_fit();
        // std::cout<<std::endl;
        for_pub.waypoints = waypoints_ros;
        for_pub.count = waypoints_ros.size();
        if (last_length == 0)
        {
            last_path = for_pub;
            last_length = this->get_length(waypoints);
            last_position = current_position;
        }
        else
        {
            float l = this->get_length(waypoints);
            if (l + std::sqrt(CGAL::squared_distance(current_position, last_position)) < 0.90 * last_length)
            {
                std::cout << waymaker.get_batch_number() << " Kept last: Last = " << last_length << " Dist = " << std::sqrt(CGAL::squared_distance(current_position, last_position)) << " Current = " << l << std::endl;
                rclcpp::Duration total_time = this->now() - starting_time;
                total_execution_time += total_time.nanoseconds() / 1000000.0;
                std::cout << "Time of Execution: " << total_time.nanoseconds() / 1000000.0 << " ms." << std::endl;
                return;
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
                should_exit = true;
                // exit(1); //Initiates ABS. If there is a safer method, it should be prefered.
            }
        }
        else
        {
            last_path = for_pub;
            last_length = this->get_length(waypoints);
            last_position = current_position;
        }

        if (msg->lap_count != 0)
        {
            if (added_waypoints.size() == 0)
            {
                added_waypoints.push_back(waypoints[1]); // waypoints[1] == midpoints[0]
            }
            else
            {
                if (CGAL::squared_distance(waypoints[1], added_waypoints[added_waypoints.size() - 1]) > 2 * 2)
                {
                    added_waypoints.push_back(waypoints[1]);
                }
                if (added_waypoints.size() != 1)
                {
                    // Point vect1 = Point(added_waypoints[0].x() - waypoints[waypoints.size()-2].x(), added_waypoints[0].y() - waypoints[waypoints.size()-2].y());
                    // Point vect2 = Point(-added_waypoints[0].x() + waypoints[waypoints.size()-1].x(), -added_waypoints[0].y() + waypoints[waypoints.size()-1].y());

                    // float norm1 = std::sqrt(squared_distance(vect1, Point(0, 0)));
                    // float norm2 = std::sqrt(squared_distance(vect2, Point(0, 0)));
                    // float dist = std::sqrt(squared_distance(waypoints[waypoints.size()-1], waypoints[waypoints.size()-2]));

                    // float cosine = (vect1.x()*vect2.x() + vect1.y()*vect2.y()) / (norm1 * norm2);
                    float mn = DBL_MAX, loc;
                    int mn_idx = 0;

                    for (int i = 1; i < waypoints.size(); i++)
                    {
                        loc = std::sqrt(CGAL::squared_distance(waypoints[i], added_waypoints[0]));
                        mn = std::min(mn, loc);
                        mn_idx = i;
                    }

                    std::cout << "Min dist to first is: " << mn << std::endl;

                    if (mn < 2 && added_waypoints.size() > 10)
                    {
                        std::vector<custom_msgs::msg::Point2Struct> waypoints_ros_final;

                        for (int i = 1; i < mn_idx; i++)
                        {
                            if (CGAL::squared_distance(waypoints[i], added_waypoints[added_waypoints.size() - 1]) > 2 * 2)
                                added_waypoints.push_back(waypoints[i]);
                        }
                        this->has_completed_total_path = 1;

                        for_pub.count = added_waypoints.size();
                        waypoints_ros_final.reserve(added_waypoints.size() + 5);

                        save_total_path << added_waypoints.size();

                        for (int i = 0; i < added_waypoints.size(); i++) // create finalized message
                        {
                            save_total_path << added_waypoints[i].x() << " " << added_waypoints[i].y() << std::endl;
                            // std::cout<<"("<<added_waypoints[i].x()<<", "<<added_waypoints[i].y()<<"),";
                            sample.x = added_waypoints[i].x();
                            sample.y = added_waypoints[i].y();
                            waypoints_ros_final.push_back(sample);
                        }
                        for (int i = 0; i < 5; i++)
                        {
                            waypoints_ros_final.push_back(waypoints_ros_final[i]);
                            save_total_path << added_waypoints[i].x() << " " << added_waypoints[i].y() << std::endl;
                        }
                        // std::cout<<std::endl;
                        finalized.waypoints = waypoints_ros_final;
                        finalized.is_out_of_map = 0;
                        finalized.should_exit = 0;
                        finalized.count = waypoints_ros_final.size();

                        waypoints_ros.erase(waypoints_ros.begin() + mn_idx, waypoints_ros.end());

                        for (int i = 0; i < added_waypoints.size(); i++) // create curr message, that puts the added wayps at the end of the existing path
                        {
                            if (CGAL::squared_distance(added_waypoints[i], waypoints[0]) < 1)
                                break;
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
        for_pub.initial_v_x = msg->pose.velocity_state.global_index == 0 ? -1 : msg->pose.velocity_state.velocity_x;
        for_pub.lap_count = msg->lap_count;
        for_pub.global_index = msg->pose.velocity_state.global_index;
        for_pub.should_exit = should_exit;
        pub_time_1 = this->now().nanoseconds()/1e6;
        pub_waypoints->publish(for_pub);
        pub_time_2 = this->now().nanoseconds()/1e6;

        // Timestamp Logging
        timestamp_log.log(starting_time.nanoseconds()/1e6, 0, msg->pose.velocity_state.global_index);
        timestamp_log.log((pub_time_2 + pub_time_1)/2, 1, msg->pose.velocity_state.global_index);

        std::cout << waymaker.get_batch_number() << " score: " << batch_output.second << " no of midpoints: " << waypoints.size() << std::endl;
        rclcpp::Duration total_time = this->now() - starting_time;
        total_execution_time += total_time.nanoseconds() / 1000000.0;

        if(should_exit)
        {
            exit(1);
        }
        RCLCPP_INFO_STREAM(get_logger(), "Time of Execution: " << total_time.nanoseconds() / 1000000.0 << " ms.");
    }

    LifecyclePathPlanner::~LifecyclePathPlanner()
    {
        std::cout << "Average execution time: " << total_execution_time / waymaker.get_batch_number() << std::endl;
    }

    float LifecyclePathPlanner::get_length(std::vector<Point> path) const
    {
        float l = 0;
        for (int i = 1; i < path.size(); i++)
        {
            l += std::sqrt(CGAL::squared_distance(path[i], path[i - 1]));
        }
        return l;
    }

    float LifecyclePathPlanner::get_angle_avg(std::vector<Point> path) const
    {
        // if(path.size()-2==0)return 0;
        float l = 0;
        float mx = 0;
        for (int i = 1; i < path.size() - 1; i++)
        {
            l += std::abs(180 - angle_point_2(path[i - 1], path[i], path[i + 1]));
            mx = std::max(mx, float(std::abs(180 - angle_point_2(path[i - 1], path[i], path[i + 1]))));
            // std::cout<<std::abs(180-angle_point_2(path[i-1], path[i], path[i+1]))<<", ";
        }
        // std::cout<<"Average angle = "<<l/(path.size()-2)<<std::endl;
        return mx;
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    path_planner::LifecyclePathPlanner pathPlannerNode{};
    rclcpp::spin(pathPlannerNode.get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}