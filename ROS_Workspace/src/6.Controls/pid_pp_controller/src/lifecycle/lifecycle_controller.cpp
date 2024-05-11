#include "lifecycle_controller.hpp"
#include <iomanip>

LifecyclePID_PP_Node::LifecyclePID_PP_Node() : LifecycleNode("pure_pursuit"), profile(nullptr), model(), pp_controller(), spline(nullptr), pid_controller(), has_run_waypoints(false), count_wp(0), prev_lap(1), switch_br(false), should_exit(false), last_torque(0), mission(MISSION::UNLOCK)
{
    parameter_load();
    RCLCPP_WARN(get_logger(), "\n-- Pure Pursuit Node Created");
}

LifecyclePID_PP_Node::~LifecyclePID_PP_Node()
{
    delete profile;
    delete spline;
    std::cout << "Object destroyed" << std::endl;
}

void LifecyclePID_PP_Node::known_map_substitute(int lap, int total_laps)
{
    switch (mission)
    {
    case MISSION::TRACKDRIVE:
    {
        mids.open("src/6.Controls/pid_pp_controller/data/" + midpoints);
        int count;
        mids >> count;
        path_planning::PointsArray midpoints(count, 2);
        double x, y;
        for (int i = 0; i < count; i++)
        {
            mids >> x >> y;
            midpoints(i, 0) = x;
            midpoints(i, 1) = y;
        }
        path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
        bool is_end = lap == total_laps;
        double ms = max_speed;
        double v_init = lap == 0 ? 0 : this->v_x;
        VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, 0, safety_factor, braking_distance);
        path_planning::ArcLengthSpline *spline_to_delete = this->spline;
        VelocityProfile *profile_to_delete = this->profile;
        this->has_run_waypoints = true;
        this->is_end = is_end;
        this->profile = profile;
        this->spline = spline;
        /* VARIABLE UNLOCK */
        // pthread_spin_unlock(&global_lock_);

        delete spline_to_delete;
        delete profile_to_delete;
        mids.close();
    }break;
    case MISSION::ACCELERATION:
    {
        if (lap == 0)
        {
            path_planning::PointsArray midpoints(30, 2);
            for (int i = 0; i < 30; i++)
            {
                midpoints(i, 0) = i * 5;
                midpoints(i, 1) = 0;
            }
            path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
            bool is_end = 0;
            double ms = max_speed;
            double v_init = 3;
            VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, 0, safety_factor, braking_distance);
            path_planning::ArcLengthSpline *spline_to_delete = this->spline;
            VelocityProfile *profile_to_delete = this->profile;
            this->has_run_waypoints = true;
            this->is_end = is_end;
            this->profile = profile;
            this->spline = spline;
            /* VARIABLE UNLOCK */
            // pthread_spin_unlock(&global_lock_);

            delete spline_to_delete;
            delete profile_to_delete;
        }
        if (lap == 1)
        {
            path_planning::PointsArray midpoints2(16, 2);
            for (int i = 14; i < 30; i++)
            {
                midpoints2(i - 14, 0) = i * 5;
                midpoints2(i - 14, 1) = 0;
            }
            path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints2, path_planning::BoundaryCondition::Anchored);
            bool is_end = 1;
            double ms = max_speed;
            double v_init = this->v_x;
            VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, 0, safety_factor, braking_distance);
            path_planning::ArcLengthSpline *spline_to_delete = this->spline;
            VelocityProfile *profile_to_delete = this->profile;
            this->has_run_waypoints = true;
            this->is_end = is_end;
            this->profile = profile;
            this->spline = spline;
            /* VARIABLE UNLOCK */
            // pthread_spin_unlock(&global_lock_);

            delete spline_to_delete;
            delete profile_to_delete;
        }
    }break;
    case MISSION::SKIDPAD:
    {
        if (lap == 0)
        {
            // first straight
            path_planning::PointsArray midpoints(21, 2);
            for (int i = -5; i <= 0; i++)
            {
                midpoints(i + 5, 0) = i * 3;
                midpoints(i + 5, 1) = 0;
            }
            for (int i = 1; i <= 15; i++)
            {
                midpoints(i + 5, 0) = (7.675 + 2) * (std::sin(2 * PI * ((i * 1.0) / 30)));
                midpoints(i + 5, 1) = (7.675 + 2) * (1 - std::cos(2 * PI * ((i * 1.0) / 30)));
            }
            path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
            bool is_end = 0;
            double ms = max_speed;
            double v_init = 3;
            VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, 0, safety_factor, braking_distance);
            path_planning::ArcLengthSpline *spline_to_delete = this->spline;
            VelocityProfile *profile_to_delete = this->profile;
            this->has_run_waypoints = true;
            this->is_end = is_end;
            this->profile = profile;
            this->spline = spline;
            /* VARIABLE UNLOCK */
            // pthread_spin_unlock(&global_lock_);

            delete spline_to_delete;
            delete profile_to_delete;
        }
        else if (lap == 1)
        {
            // first right hand turn
            path_planning::PointsArray midpoints(60, 2);

            for (int i = 0; i < 60; i++)
            {
                midpoints(i, 0) = (7.675 + 2) * (std::sin(2 * PI * ((i * 1.0) / 40)));
                midpoints(i, 1) = (7.675 + 2) * (1 - std::cos(2 * PI * ((i * 1.0) / 40)));
            }
            path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
            bool is_end = 0;
            double ms = max_speed;
            double v_init = this->v_x;
            VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, 0, safety_factor, braking_distance);
            path_planning::ArcLengthSpline *spline_to_delete = this->spline;
            VelocityProfile *profile_to_delete = this->profile;
            this->has_run_waypoints = true;
            this->is_end = is_end;
            this->profile = profile;
            this->spline = spline;
            /* VARIABLE UNLOCK */
            // pthread_spin_unlock(&global_lock_);

            delete spline_to_delete;
            delete profile_to_delete;
        }
        else if (lap == 2)
        {
            // second right hand turn
            path_planning::PointsArray midpoints(60, 2);

            for (int i = 0; i < 40; i++)
            {
                midpoints(i, 0) = (7.675 + 2) * (std::sin(2 * PI * ((i * 1.0) / 40)));
                midpoints(i, 1) = (7.675 + 2) * (1 - std::cos(2 * PI * ((i * 1.0) / 40)));
            }
            for (int i = 0; i < 20; i++)
            {
                midpoints(i + 40, 0) = (7.675 + 2) * (std::sin(2 * PI * ((i * 1.0) / 40)));
                midpoints(i + 40, 1) = (7.675 + 2) * (-1 + std::cos(2 * PI * ((i * 1.0) / 40)));
            }
            path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
            bool is_end = 0;
            double ms = max_speed;
            double v_init = this->v_x;
            VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, 0, safety_factor, braking_distance);
            path_planning::ArcLengthSpline *spline_to_delete = this->spline;
            VelocityProfile *profile_to_delete = this->profile;
            this->has_run_waypoints = true;
            this->is_end = is_end;
            this->profile = profile;
            this->spline = spline;
            /* VARIABLE UNLOCK */
            // pthread_spin_unlock(&global_lock_);

            delete spline_to_delete;
            delete profile_to_delete;
        }
        else if (lap == 3)
        {
            // first left hand turn
            path_planning::PointsArray midpoints(60, 2);

            for (int i = 0; i < 60; i++)
            {
                midpoints(i, 0) = (7.675 + 2) * (std::sin(2 * PI * ((i * 1.0) / 40)));
                midpoints(i, 1) = (7.675 + 2) * (-1 + std::cos(2 * PI * ((i * 1.0) / 40)));
            }
            path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
            bool is_end = 0;
            double ms = max_speed;
            double v_init = this->v_x;
            VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, 0, safety_factor, braking_distance);
            path_planning::ArcLengthSpline *spline_to_delete = this->spline;
            VelocityProfile *profile_to_delete = this->profile;
            this->has_run_waypoints = true;
            this->is_end = is_end;
            this->profile = profile;
            this->spline = spline;
            /* VARIABLE UNLOCK */
            // pthread_spin_unlock(&global_lock_);

            delete spline_to_delete;
            delete profile_to_delete;
        }
        else if (lap == 4)
        {
            // second left hand turn
            path_planning::PointsArray midpoints(45, 2);

            for (int i = 0; i < 40; i++)
            {
                midpoints(i, 0) = (7.675 + 2) * (std::sin(2 * PI * ((i * 1.0) / 40)));
                midpoints(i, 1) = (7.675 + 2) * (-1 + std::cos(2 * PI * ((i * 1.0) / 40)));
            }
            for (int i = 0; i < 5; i++)
            {
                midpoints(i + 40, 0) = i * 5;
                midpoints(i + 40, 1) = 0;
            }

            path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
            bool is_end = 0;
            double ms = max_speed;
            double v_init = this->v_x;
            VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, 0, safety_factor, braking_distance);
            path_planning::ArcLengthSpline *spline_to_delete = this->spline;
            VelocityProfile *profile_to_delete = this->profile;
            this->has_run_waypoints = true;
            this->is_end = is_end;
            this->profile = profile;
            this->spline = spline;
            /* VARIABLE UNLOCK */
            // pthread_spin_unlock(&global_lock_);

            delete spline_to_delete;
            delete profile_to_delete;
        }
        else if (lap == 5)
        {
            // ending straight
            path_planning::PointsArray midpoints(5, 2);

            for (int i = 0; i < 5; i++)
            {
                midpoints(i, 0) = i * 5;
                midpoints(i, 1) = 0;
            }

            path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
            bool is_end = 1;
            double ms = max_speed;
            double v_init = this->v_x;
            VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, 0, safety_factor, braking_distance);
            path_planning::ArcLengthSpline *spline_to_delete = this->spline;
            VelocityProfile *profile_to_delete = this->profile;
            this->has_run_waypoints = true;
            this->is_end = is_end;
            this->profile = profile;
            this->spline = spline;
            /* VARIABLE UNLOCK */
            // pthread_spin_unlock(&global_lock_);

            delete spline_to_delete;
            delete profile_to_delete;
        }
        else if (lap == 6)
        {
            // ending straight with braking
            path_planning::PointsArray midpoints(4, 2);

            for (int i = 0; i < 4; i++)
            {
                midpoints(i, 0) = (i + 2) * 5;
                midpoints(i, 1) = 0;
            }

            path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
            bool is_end = 1;
            double ms = 0;
            double v_init = this->v_x;
            VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, 0, safety_factor, braking_distance);
            path_planning::ArcLengthSpline *spline_to_delete = this->spline;
            VelocityProfile *profile_to_delete = this->profile;
            this->has_run_waypoints = true;
            this->is_end = is_end;
            this->profile = profile;
            this->spline = spline;
            /* VARIABLE UNLOCK */
            // pthread_spin_unlock(&global_lock_);

            delete spline_to_delete;
            delete profile_to_delete;
        }
    }break;
    case MISSION::EBS_TEST:
    {
        if (lap == 0)
        {
            path_planning::PointsArray midpoints(20, 2);
            for (int i = 0; i < 20; i++)
            {
                midpoints(i, 0) = i * 5;
                midpoints(i, 1) = 0;
            }
            path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
            bool is_end = 1;
            double ms = max_speed;
            double v_init = 1;
            VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, 0, safety_factor, braking_distance);
            path_planning::ArcLengthSpline *spline_to_delete = this->spline;
            VelocityProfile *profile_to_delete = this->profile;
            this->has_run_waypoints = true;
            this->is_end = is_end;
            this->profile = profile;
            this->spline = spline;
            /* VARIABLE UNLOCK */
            // pthread_spin_unlock(&global_lock_);

            delete spline_to_delete;
            delete profile_to_delete;
        }
    }break;
    default:
    {
        RCLCPP_ERROR(get_logger(), "Invalid Discipline: %s", discipline);
    }break;
    }
}

void LifecyclePID_PP_Node::waypoints_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr msg)
{
    if((mission == MISSION::ACCELERATION || mission == MISSION::EBS_TEST) && (accel_x_axis))
        return;

    std::cout << ++count_wp << " Entered Waypoints callback" << std::endl;
    rclcpp::Time starting_time = this->now();

    path_planning::ArcLengthSpline *spline;

    switch (mission)
    {
    case MISSION::AUTOX:
    case MISSION::SKIDPAD:
    case MISSION::TRACKDRIVE:
    {
        path_planning::PointsArray midpoints(std::max(3, int(msg->count)-1), 2);
        
        if (msg->count >= 4)
        {
            for (int i = 1; i < msg->count; i++)
            {
                midpoints(i-1, 0) = msg->waypoints[i].x;
                midpoints(i-1, 1) = msg->waypoints[i].y;
            }
        }
        else if (msg->count == 3)
        {
            midpoints(0, 0) = msg->waypoints[1].x;
            midpoints(0, 1) = msg->waypoints[1].y;
            midpoints(1, 0) = 0.5 * msg->waypoints[1].x + 0.5 * msg->waypoints[2].x;
            midpoints(1, 1) = 0.5 * msg->waypoints[1].y + 0.5 * msg->waypoints[2].y;
            midpoints(2, 0) = msg->waypoints[2].x;
            midpoints(2, 1) = msg->waypoints[2].y;
        }
        else
        {
            return;
        };
        for (int i = 0; i < std::max(3, int(msg->count)-1); i++)
        {
            std::cout<<"("<<midpoints(i, 0)<<','<<midpoints(i, 1)<<')'<<", ";
        }
        std::cout<<std::endl;
        for (int i = 0; i < int(msg->count); i++)
        {
            std::cout<<"("<<msg->waypoints[i].x<<','<<msg->waypoints[i].y<<')'<<", ";
        }

        spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
    }break;
    case MISSION::ACCELERATION:
    case MISSION::EBS_TEST:
    {
        /*
            The idea for the new Accel controller is to use the path planning result to determine the centerline of the accel track.
            The path planning midpoints will most likely consist of 3-4 midpoints (ranged at 10m max). Using the last 2 points, we will add a final midpoint located at a distance from the vehicle, on the line defined by the last 2 points. With that, we will have an extended path which will enable better velocities and smoother accelerations.        
        */
        path_planning::PointsArray midpoints(std::max(3, int(msg->count)), 2);

        if (msg->count >= 3)
        {
            for (int i = 1; i < msg->count; i++)
            {
                midpoints(i-1, 0) = msg->waypoints[i].x;
                midpoints(i-1, 1) = msg->waypoints[i].y;
            }
        }
        else
        {
            return;
        };

        // last midpoint
        float l1_x = midpoints(std::max(3, int(msg->count))-2, 0);
        float l1_y = midpoints(std::max(3, int(msg->count))-2, 1);

        // second to last midpoint
        float l2_x = midpoints(std::max(3, int(msg->count))-3, 0);
        float l2_y = midpoints(std::max(3, int(msg->count))-3, 1);

        // direction vector
        float d_x  = l1_x - l2_x;
        float d_y  = l1_y - l2_y;

        // normalization
        d_x = d_x / (std::sqrt(std::pow(d_x, 2)+std::pow(d_y, 2)));
        d_y = d_y / (std::sqrt(std::pow(d_x, 2)+std::pow(d_y, 2)));

        // form additional point
        float add_x = l1_x + d_x * 75;
        float add_y = l1_y + d_y * 75;
        // std::cout<<".--------"<<add_x<<" "<<add_y<<std::endl;

        midpoints(std::max(3, int(msg->count))-1, 0) = add_x;
        midpoints(std::max(3, int(msg->count))-1, 1) = add_y;

        // for (int i = 0; i < std::max(3, int(msg->count)+1); i++)
        // {
        //     std::cout<<"("<<midpoints(i, 0)<<','<<midpoints(i, 1)<<')'<<", ";
        // }
        // std::cout<<std::endl<<"------"<<std::endl;

        spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
    }break;
    default:
    {
        return;
    }break;
    }


    // RCLCPP_INFO_STREAM(get_logger(), "Lap = %d / %d\n", msg->lap_count, laps_to_do);
    if (msg->lap_count == laps_to_do && !is_end)
    {
        RCLCPP_WARN(get_logger(), "Max laps detected. Starting Finishing Sequence...");
        is_end = true;
        last_position = Point(msg->waypoints[0].x, msg->waypoints[0].y);
    }
    else if (is_end)
    {
        braking_distance -= Point::distance(last_position, Point(msg->waypoints[0].x, msg->waypoints[0].y));
        last_position = Point(msg->waypoints[0].x, msg->waypoints[0].y);
    }

    //bool is_end = msg->lap_count == laps_to_do;
    double ms;
    switch (mission)
    {
    case MISSION::AUTOX:
    {
        ms = msg->is_out_of_map ? max_speed / 4 : max_speed;
    }break;
    case MISSION::SKIDPAD:
    {
        if(msg->lap_count==0 || msg->lap_count==1 || msg->lap_count==3 || msg->lap_count==5)
            ms = max_speed / 4;
        else
            ms = max_speed;
    }break;
    default:
    {
        ms = max_speed;
    }break;
    }


    this->is_out_of_map = msg->is_out_of_map;

    this->should_exit = msg->should_exit;

    double v_init = msg->initial_v_x == -1 ? this->v_x : msg->initial_v_x;

    VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, (msg->lap_count < 2)&&((mission==MISSION::AUTOX)) , safety_factor, braking_distance, mission==MISSION::ACCELERATION); // last available speed is used. Alternatively should be in waypoints msg
    /*
        To minimize time spent with locked object variables, we make it so that the bare minimum of operations is done. We store the modifiable objects(spline, profile) as pointers. Thus we achieve 2 things
        (a) The new objects can be constructed locally and the modification required is only the copying of the pointer address
        (b) The old objects can be destroyed locally as we store their old address in local variables and delete them independently
        As a result during lockdown we do only 4 address copies and a bool copy
    */
    // if(is_end)exit(0);
    /* VARIABLE LOCK */
    // pthread_spin_lock(&global_lock_);
    path_planning::ArcLengthSpline *spline_to_delete = this->spline;
    VelocityProfile *profile_to_delete = this->profile;
    this->has_run_waypoints = true;
    this->is_end = is_end;
    this->profile = profile;
    this->spline = spline;
    /* VARIABLE UNLOCK */
    // pthread_spin_unlock(&global_lock_);

    delete spline_to_delete;
    delete profile_to_delete;
    rclcpp::Duration total_time = this->now() - starting_time;
    total_execution_time += total_time.nanoseconds() / 1000000.0;

    waypoints_timestamp_log.log(starting_time.nanoseconds()/1e6, 0, msg->global_index);

    // std::cout << "Time of Waypoints Execution: " << total_time.nanoseconds() / 1000000.0 << " ms." << std::endl;
}

void LifecyclePID_PP_Node::pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr msg)
{
    std::cout << "Entered Pose Callback" << std::endl;

    rclcpp::Time starting_time = this->now();

    v_x = msg->velocity_state.velocity_x;
    v_y = msg->velocity_state.velocity_y;
    r = msg->velocity_state.yaw_rate;
    a_x = msg->velocity_state.acceleration_x;
    a_y = msg->velocity_state.acceleration_y;

    int exit_flag = 0;

    if (!has_run_waypoints)
    {
        if (!((mission == MISSION::ACCELERATION || mission == MISSION::EBS_TEST) && (accel_x_axis)))
            return;
        known_map_substitute(1, laps_to_do);
        has_run_waypoints = 1;
    }

    if (prev_lap != msg->lap_count && ((mission == MISSION::ACCELERATION || mission == MISSION::EBS_TEST) && (accel_x_axis)))
    {
        prev_lap = msg->lap_count;
        known_map_substitute(prev_lap, laps_to_do);
    }
    if(should_exit && motor_control == Motor_control_mode::TORQUE)
    {
        RCLCPP_WARN(get_logger(), "Path Planning has died due to bad path. Sending full breaks until Lifecycle Manager detects failure and activates EBS");
        custom_msgs::msg::TxControlCommand for_publish;
        for_publish.brake_pressure_target = 20.0;
        for_publish.motor_torque_target = model.max_negative_torque;
        for_publish.steering_angle_target = 0;
        for_publish.global_index = msg->velocity_state.global_index;
        for_publish.speed_actual = v_x * 3.6;
        for_publish.speed_target = 0;
        for_publish.motor_control = 1;
        for_publish.exit_flag = 5;
        
        pub_time_1 = this->now().nanoseconds()/1e6;
        pub_actuators->publish(for_publish);
        pub_time_2 = this->now().nanoseconds()/1e6;

        pose_timestamp_log.log(starting_time.nanoseconds()/1e6, 0, msg->velocity_state.global_index);
        pose_timestamp_log.log((pub_time_2 + pub_time_1)/2, 1, msg->velocity_state.global_index);
        return;
    }
    else if(should_exit && motor_control == Motor_control_mode::VELOCITY)
    {
        RCLCPP_WARN(get_logger(), "Path Planning has died due to bad path. Sending full breaks until Lifecycle Manager detects failure and activates EBS");
        custom_msgs::msg::TxControlCommand for_publish;
        for_publish.brake_pressure_target = 20.0;
        for_publish.motor_torque_target = 0; //motor_torque_target := velocity target
        for_publish.steering_angle_target = 0;
        for_publish.global_index = msg->velocity_state.global_index;
        for_publish.speed_actual = v_x * 3.6;
        for_publish.speed_target = 0;
        for_publish.motor_control = 0;
        for_publish.exit_flag = 4;
        
        pub_time_1 = this->now().nanoseconds()/1e6;
        pub_actuators->publish(for_publish);
        pub_time_2 = this->now().nanoseconds()/1e6;

        pose_timestamp_log.log(starting_time.nanoseconds()/1e6, 0, msg->velocity_state.global_index);
        pose_timestamp_log.log((pub_time_2 + pub_time_1)/2, 1, msg->velocity_state.global_index);
        return;
    }

    Point position(msg->position.x, msg->position.y);
    double theta = msg->theta;

    Projection projection = this->profile->operator()(position, theta);
    /*
    projection.velocity -> target velocity
    projection.radius -> local radius (R)
    prokection.cross_track_error -> distance from projected point
    */

    if(dynamic_vp == Velocity_profile_mode::CONSTANT)
    {
        projection.velocity = projection.velocity > 0 ? max_speed : 0;
    }

    std::cout << "target : " << projection.velocity << ". speed : " << this->v_x << std::endl;
    log << projection.velocity << " " << this->v_x << std::endl;

    custom_msgs::msg::TxControlCommand for_publish;
    for_publish.speed_actual = this->v_x;
    for_publish.speed_target = projection.velocity ;

    double fx_next = pid_controller(projection.velocity, this->v_x);


    double min_radius;
    double fz = model.Fz_calc("full", 1, 0, v_x);
    // Checking Force
    double force = fx_next + 0.5 * v_x * v_x * model.cd_A + 0.05 * fz - v_y * r * model.m;
    double fx; // old fx, new fx
    fx = std::abs(fx_next) > std::abs(model.m * a_x) ? fx_next : model.m * a_x;

    fz *= safety_factor; // C_SF1
    double rem = fz * fz - std::pow(fx / model.mx_max(fz), 2);

    double frz = model.Fz_calc("rear", 1, 1, v_x, a_x);
    frz *= safety_factor; // C_SF2
    /*TBC*/
    if (rem < 0)
    {
        // exei ginei malakia
        if (std::abs(model.m * a_x) > std::abs(fx_next))
        {
            // spiniaroume right now. HANDLING TBD
            force = 0; // Let go
        }
        else
        {
            // tha spiniaroume otan efarmostei
            force = 0.5 * v_x * v_x * model.cd_A + 0.03 * fz - v_y * r * model.m + safety_factor * std::min((model.mx_max(fz) * fz) * (force > 0 ? 1 : -1), force > 0 ? model.max_positive_force : model.max_negative_force);
            fx_next = (force - 0.5 * v_x * v_x * model.cd_A - 0.03 * fz + v_y * r * model.m) * (force > 0 ? 1 : -1);
            fx = std::abs(fx_next) > std::abs(model.m * a_x) ? fx_next : model.m * a_x;
            rem = fz * fz - std::pow(fx / model.mx_max(fz), 2);
        }
    }

    double trq = model.Torque(force);
    trq = std::min(model.max_positive_torque, std::max(model.max_negative_torque, trq));

    last_torque = trq = std::min(last_torque + max_torque_difference, std::max(last_torque - max_torque_difference, trq));
    for_publish.motor_control = static_cast<bool>(motor_control);

    if(motor_control == Motor_control_mode::TORQUE)
    {
        for_publish.motor_torque_target = trq;
    }
    else
    {
        for_publish.motor_torque_target = projection.velocity * (30 / PI) * model.gr / model.R_wheel;
    }

    // CALCULATING MIN RADIUS
    /*
        (fx/mx)**2 + (fy/my)**2 <= fz**2
        (fy/my)**2 <= fz**2 - (fx/mx)**2
        fy = mu**2/R    fx**2 - (fx/mx)**2 = rem
        mu**2/R <= rem
        R >= mu**2 / rem
    */

    // min_radius = model.m * v_x * v_x / rem;
    double mx_head = PI * max_steering / 180;
    double mn_radius_wheel = model.wb / std::tan(mx_head);
    double mu = model.my_max(fz);
    mu *= safety_factor; // C_SF3
    min_radius = std::min(v_x * v_x / (mu * model.g), mn_radius_wheel);

    Point tp;
    double ld, input;
    switch (lookahead_input)
    {
        case TARGET_VELOCITY:
            input = projection.velocity;
            break;
        case ACTUAL_VELOCITY:
            input = v_x;
            break;
        case LOCAL_RADIUS:
            input = projection.radius;
            break;
        case MAX_SPEED:
            input = sqrt(model.g * projection.radius);
            break;
        default:
            break;
    }
    // std::cout<<"9.. ";
    if (projection.cross_track_error < emergency_threshold)
    {
        
        ld = pp_controller.lookahead(input, false);
    }
    else if(!is_end)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Emergency Manouevre. Error = "<<projection.cross_track_error<<"m");
        std::cout << "emergency manouevre" << std::endl;
        for_publish.motor_torque_target *= for_publish.motor_torque_target > 0 ? 0.25 : 1;
        ld = pp_controller.lookahead(input, true);
        exit_flag = 2;
    }
    
    

    tp = this->profile->get_target_point(ld, position, min_radius, theta);
    double heading_angle;
    if (!tp.error)
        heading_angle = pp_controller(tp, theta, min_radius);
    else
    {
        heading_angle = 0;
        RCLCPP_INFO_STREAM(get_logger(), "Target point error");
        if(exit_flag==0)
            exit_flag = 3;
        else
            exit_flag = 4;
    }

    heading_angle = std::min(mx_head, std::max(-mx_head, heading_angle));

    if (last_steering != 5) // means that last_steering has not received an actual value
        heading_angle = std::min(last_steering + 0.25, std::max(last_steering - 0.25, heading_angle));

    last_steering = heading_angle;

    for_publish.steering_angle_target = heading_angle;
    for_publish.global_index = msg->velocity_state.global_index;

    if(!switch_br && (force < 0 && v_x < safe_speed_to_break && is_end))RCLCPP_INFO_STREAM(get_logger(), "Initiated Braking Sequence");

    switch_br = force < 0 && v_x < safe_speed_to_break && (is_end || projection.velocity < 0.1);
    if(tp.error && !is_end)switch_br=1;

    for_publish.brake_pressure_target = switch_br ? 20.0 : 0;
    // if (switch_br)
    //     for_publish.motor_torque_target = 0;
    // // std::cout<<"13.. ";

    double lr = model.wb * model.wd;
    Point drear = Point(-lr * std::cos(theta), -lr * std::sin(theta));
    Point act_target(position.x() - drear.x() + tp.x(), position.y() - drear.y() + tp.y());

    custom_msgs::msg::Point2Struct tg;
    tg.x = act_target.x();
    tg.y = act_target.y();
    pub_target->publish(tg);

    /* EXTRA SAFETY CHECKS */
    if(this->v_x > std::max(1.5 * max_speed, max_speed + 1)) // if max_speed is very low (i.e 1m/s, this would get triggered at 1.5m/s)
    {
        for_publish.motor_torque_target = model.max_negative_torque;
        for_publish.brake_pressure_target = 20;
        RCLCPP_INFO_STREAM(get_logger(), "Speed 50% bigger than target. Braking...");
        exit_flag = 6;
    }
    if(this->v_x > std::max(2 * max_speed, max_speed + 2))
    {
        for_publish.steering_angle_target = 0;
        for_publish.motor_torque_target = model.max_negative_torque;
        for_publish.brake_pressure_target = 20;
        for_publish.exit_flag = 7;
        pub_actuators->publish(for_publish);
        RCLCPP_INFO_STREAM(get_logger(), "Speed 100% bigger than target. Exiting to initiate EBS...");
        exit(1); //Initiates the ABS. If there is a safer method than exit, it should be preferred

    }
    if(exit_flag==0)
        exit_flag = 1;
    if(this->is_out_of_map)
        for_publish.exit_flag = 1;
    pub_time_1 = this->now().nanoseconds()/1e6;
    pub_actuators->publish(for_publish);
    pub_time_2 = this->now().nanoseconds()/1e6;
    std::cout << "Command: Torque = " << std::fixed << std::setprecision(4) << for_publish.motor_torque_target << " Nm, Heading = " << std::fixed << std::setprecision(4) << heading_angle << " rad, BP = " << switch_br ? 1 : 0;
    std::cout << " , ld = " << std::fixed << std::setprecision(4) << ld << std::endl;

    rclcpp::Duration total_time = this->now() - starting_time;
    total_execution_time += total_time.nanoseconds() / 1000000.0;

    std::cout << "Time of Pose Execution: " << total_time.nanoseconds() / 1000000.0 << " ms." << std::endl;

    pose_timestamp_log.log(starting_time.nanoseconds()/1e6, 0, msg->velocity_state.global_index);
    pose_timestamp_log.log((pub_time_2 + pub_time_1)/2, 1, msg->velocity_state.global_index);
}
/*  Exit Flag meaning:
    0:  invalid
    1:  all ok
    2:  emergency manouevre
    3:  Target Point error
    4:  emergency manouevre & target point error
    5:  path planning initiated stop
    6:  50% more. braking
    7:  100% more. ending
*/

void LifecyclePID_PP_Node::parameter_load()
{
    declare_parameter<float>("mass", 193.5);
    declare_parameter<float>("gravitational_acceleration", 9.81);
    declare_parameter<float>("wheelbase", 1.59);
    declare_parameter<float>("weight_distribution", 0.467);
    declare_parameter<float>("h_cog", 0.275);
    declare_parameter<float>("air_density", 1.225);
    declare_parameter<float>("Cd*A", 2);
    declare_parameter<float>("Cl*A", 5);
    declare_parameter<float>("gear_ratio", 3.9);
    declare_parameter<float>("wheel_radius", 0.2054);
    declare_parameter<float>("efficiency", 0.85);
    declare_parameter<float>("Fz_0", 1112.0554070627252);
    declare_parameter<float>("ld_min", 3);
    declare_parameter<float>("ld_max", 7);
    declare_parameter<float>("max_positive_torque",168);
    declare_parameter<float>("max_negative_torque", -150);
    declare_parameter<float>("minimum_weight_distribution_rear", 1);
    declare_parameter<float>("c_tire", 0.66);
    declare_parameter<float>("v_min", 3.0);
    declare_parameter<float>("v_max", 15.0);
    declare_parameter<float>("emergency_factor", 0.75);
    declare_parameter<float>("emergency_threshold", 5);
    declare_parameter<float>("safety_factor", 1.0);
    declare_parameter<float>("max_speed", 0.75);
    declare_parameter<float>("dt", 0.025);
    declare_parameter<float>("safe_speed_to_break", 1.0);
    declare_parameter<float>("braking_distance", 15.0);
    declare_parameter<float>("max_steering", 24.0);
    declare_parameter<float>("max_torque_difference", 15.0);

    declare_parameter<int>("kp", 1500);
    declare_parameter<int>("ki", 100);
    declare_parameter<int>("kd", 0);
    declare_parameter<int>("PID_max_output", 3000);
    declare_parameter<int>("Integral_max_output", 1500);
    declare_parameter<int>("spline_resolution_per_meter", 10);
    declare_parameter<int>("total_laps", 5);

    declare_parameter<bool>("dynamic_vp", true);
    declare_parameter<bool>("motor_control", true);
    declare_parameter<int>("lookahead_mode", 0);

    declare_parameter<bool>("accel_x_axis", false);

    declare_parameter<string>("discipline", "Autocross");
    declare_parameter<string>("midpoints", "midpoints_from_pp.txt");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::ExecutorOptions();

    auto PIDNode = std::make_shared<LifecyclePID_PP_Node>();
    rclcpp::executors::MultiThreadedExecutor executor{options, 2};
    executor.add_node(PIDNode->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}