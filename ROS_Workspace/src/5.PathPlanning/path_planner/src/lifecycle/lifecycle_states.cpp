#include "rclcpp/rclcpp.hpp"
#include "rmw/qos_profiles.h"
#include <rclcpp/qos.hpp>

#include "lifecycle_pathplanning_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace path_planner
{
    Logger::Logger()
    {
        this->name = "";
        this->file = nullptr;
        this->run_idx = -1;
    }
    void Logger::init(std::string name)
    {
        this->name = name;
        auto dirIter = std::filesystem::directory_iterator("timestamp_logs");

        // this->run_idx = std::count_if(
        //         begin(dirIter),
        //         end(dirIter),
        //         [](auto& entry) { return is_regular_file(entry.path()); }
        // );
        this->run_idx = -1;

        for(auto& entry: dirIter) ++run_idx;
        
        char f1[30 + name.length()];
        snprintf(f1, sizeof(f1), "timestamp_logs/run_%d/%s_log.txt", this->run_idx, name.c_str());
        this->file = fopen(f1, "w");
    }
    Logger::~Logger()
    {
        if(file!=nullptr)
        {
            fclose(file);
        }
    }
    std::string Logger::check()const
    {
        if(file==nullptr)
        {
            return "Couldn't open logger " + name;
        }
        else
        {
            return "File " + name + " opened successfully";
        }
    }
    void Logger::log(double timestamp, int type, int index)
    {
        if(file == nullptr)return;
        fprintf(file, "%f\t%d\t%d\n", timestamp, type, index);
    }

    path_planner::CallbackReturn 
        LifecyclePathPlanner::on_configure(const rclcpp_lifecycle::State & state)
    {
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

        using std::placeholders::_1;
        sub_mapper = this->create_subscription<custom_msgs::msg::LocalMapMsg>("local_map", 10, std::bind(&LifecyclePathPlanner::mapping_callback, this, _1));
        pub_waypoints = this->create_publisher<custom_msgs::msg::WaypointsMsg>("waypoints", 10);

        // Timestamp logging
        this->timestamp_log.init("path_planning");
        RCLCPP_INFO_STREAM(get_logger(), timestamp_log.check());

        RCLCPP_WARN(get_logger(), "\n-- Path Planner Configured!");

        return path_planner::CallbackReturn::SUCCESS;
    }

    path_planner::CallbackReturn 
        LifecyclePathPlanner::on_activate(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_WARN(get_logger(), "\n-- Path Planner Activated!");
        pub_waypoints->on_activate();
        return path_planner::CallbackReturn::SUCCESS;
    }

    path_planner::CallbackReturn 
        LifecyclePathPlanner::on_deactivate(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_WARN(get_logger(), "\n-- Path Planner De-Activated!");
        pub_waypoints->on_deactivate();
        return path_planner::CallbackReturn::SUCCESS;
    }

    path_planner::CallbackReturn 
        LifecyclePathPlanner::on_cleanup(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_WARN(get_logger(), "\n-- Path Planner Un-Configured!");
        pub_waypoints.reset();
        sub_mapper.reset();
        return path_planner::CallbackReturn::SUCCESS;
    }

    path_planner::CallbackReturn 
        LifecyclePathPlanner::on_shutdown(const rclcpp_lifecycle::State & state)
    {   
        /* If current state is UNCONFIGURED, then just return */
        using NodeState = lifecycle_msgs::msg::State;

        uint8_t currentState = state.id();
        
        if (currentState == NodeState::PRIMARY_STATE_UNCONFIGURED) {
            RCLCPP_INFO(get_logger(), "\n-- Path Planning Shutdown!");
            return path_planner::CallbackReturn::SUCCESS;
        }

        pub_waypoints.reset();
        sub_mapper.reset();

        RCLCPP_INFO(get_logger(), "\n-- Path Planning Shutdown!");
        return path_planner::CallbackReturn::SUCCESS;
    }

    path_planner::CallbackReturn 
        LifecyclePathPlanner::on_error(const rclcpp_lifecycle::State & state)
    {
        return path_planner::CallbackReturn::SUCCESS;
    }
}