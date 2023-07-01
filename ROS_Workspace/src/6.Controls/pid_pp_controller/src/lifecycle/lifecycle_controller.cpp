#include "lifecycle_controller.hpp"
#include <iomanip>

LifecyclePID_PP_Node::LifecyclePID_PP_Node() : LifecycleNode("pure_pursuit"), profile(nullptr), model(), pp_controller(), spline(nullptr), pid_controller(), has_run_waypoints(false), count_wp(0), prev_lap(1)
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
    if (discipline == "Trackdrive")
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
        double ms = is_end ? 0 : max_speed;
        double v_init = lap == 0 ? 0 : this->v_x;
        VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, 0, safety_factor);
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
    }
    else if (discipline == "Acceleration")
    {
        if (lap == 1 or lap == 0)
        {
            path_planning::PointsArray midpoints(30, 2);
            for (int i = 0; i < 30; i++)
            {
                midpoints(i, 0) = i * 5;
                midpoints(1, 1) = 0;
            }
            path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
            bool is_end = 0;
            double ms = max_speed;
            double v_init = 3;
            VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, 0, safety_factor);
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
        if (lap == 2)
        {
            path_planning::PointsArray midpoints(15, 2);
            for (int i = 15; i < 30; i++)
            {
                midpoints(i, 0) = i * 5;
                midpoints(1, 1) = 0;
            }
            path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
            bool is_end = 1;
            double ms = 0;
            double v_init = this->v_x;
            VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, 0, safety_factor);
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
    }
    else if (discipline == "Skidpad")
    {
    }
}

void LifecyclePID_PP_Node::waypoints_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr msg)
{
    std::cout << ++count_wp << " Entered Waypoints callback" << std::endl;
    rclcpp::Time starting_time = this->now();
    // Object variables should only be updated in the end
    path_planning::PointsArray midpoints(std::max(3, int(msg->count)), 2);
    if (msg->count >= 3)
    {
        // std::cout<<"3+ "<<std::endl;
        for (int i = 0; i < msg->count; i++)
        {
            midpoints(i, 0) = msg->waypoints[i].x;
            midpoints(i, 1) = msg->waypoints[i].y;
        }
    }
    else if (msg->count == 2)
    {
        // std::cout<<"2 "<<std::endl;
        midpoints(0, 0) = msg->waypoints[0].x;
        midpoints(0, 1) = msg->waypoints[0].y;
        midpoints(1, 0) = 0.5 * msg->waypoints[0].x + 0.5 * msg->waypoints[1].x;
        midpoints(1, 1) = 0.5 * msg->waypoints[0].y + 0.5 * msg->waypoints[1].y;
        midpoints(2, 0) = msg->waypoints[1].x;
        midpoints(2, 1) = msg->waypoints[1].y;
    }
    else
    {
        // std::cout<<"1? "<<std::endl;
        return;
    };
    path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
    bool is_end = msg->lap_count == laps_to_do;
    double ms = is_end ? 0 : max_speed;
    double v_init = msg->initial_v_x == -1 ? this->v_x : msg->initial_v_x;
    VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, true, safety_factor); // last available speed is used. Alternatively should be in waypoints msg

    /*
        To minimize time spent with locked object variables, we make it so that the bare minimum of operations is done. We store the modifiable objects(spline, profile) as pointers. Thus we achieve 2 things
        (a) The new objects can be constructed locally and the modification required is only the copying of the pointer address
        (b) The old objects can be destroyed locally as we store their old address in local variables and delete them independently
        As a result during lockdown we do only 4 address copies and a bool copy
    */

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
    // std::cout << "Time of Waypoints Execution: " << total_time.nanoseconds() / 1000000.0 << " ms." << std::endl;
}

void LifecyclePID_PP_Node::pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr msg)
{
    std::cout << "Entered Pose Callback" << std::endl;
    // std::cout << "Pos: " << msg->position.x << ", " << msg->position.y << ". theta: " << msg->theta << ". v_o : " << msg->velocity_state.velocity_x << std::endl;
    //  std::cout<<"1.. ";

    rclcpp::Time starting_time = this->now();

    // std::cout<<"2.. ";
    v_x = msg->velocity_state.velocity_x;
    v_y = msg->velocity_state.velocity_y;
    r = msg->velocity_state.yaw_rate;
    a_x = msg->velocity_state.acceleration_x;
    a_y = msg->velocity_state.acceleration_y;
    // std::cout<<"3.. ";

    if (!has_run_waypoints)
    {
        if (discipline == "Autocross")
            return;
        known_map_substitute(0, laps_to_do);
        has_run_waypoints = 1;
    }

    if (prev_lap != msg->lap_count && discipline != "Autocross")
    {
        prev_lap = msg->lap_count;
        known_map_substitute(prev_lap, laps_to_do);
    }

    Point position(msg->position.x, msg->position.y);
    // Point direction(std::cos(theta), std::sin(theta));
    double theta = msg->theta;
    // std::cout<<"4.. ";
    // std::pair<double, double> projection = (*this->profile)(position, theta);
    std::pair<double, double> projection = this->profile->operator()(position, theta);
    // std::cout<<"5.. ";
    std::cout << "target : " << projection.first << ". speed : " << this->v_x << std::endl;
    log << projection.first << " " << this->v_x << std::endl;
    custom_msgs::msg::TxControlCommand for_publish;
    for_publish.speed_actual = this->v_x * 3.6;
    for_publish.speed_target = projection.first * 3.6;
    // std::cout<<"6.. ";
    double min_radius;
    double force = pid_controller(projection.first - this->v_x);
    // Checking Force
    double fx_next = force - 0.5 * v_x * v_x * model.cd_A + v_y * r * model.m;
    double fx; // old fx, new fx
    fx = std::abs(fx_next) > std::abs(model.m * a_x) ? fx_next : model.m * a_x;
    double fz = model.Fz_calc("full", 1, 0, v_x);
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
            force = 0.5 * v_x * v_x - v_y * r * model.m + safety_factor * std::min((model.mx_max(fz) * fz) * (force > 0 ? 1 : -1), force > 0 ? model.max_positive_force : model.max_negative_force);
            fx_next = (force - 0.5 * v_x * v_x * model.cd_A + v_y * r * model.m) * (force > 0 ? 1 : -1);
            fx = std::abs(fx_next) > std::abs(model.m * a_x) ? fx_next : model.m * a_x;
            rem = fz * fz - std::pow(fx / model.mx_max(fz), 2);
        }
    }
    // std::cout<<"8.. ";

    double trq = model.Torque(force);
    trq = std::min(model.max_positive_torque, std::max(model.max_negative_torque, trq));

    for_publish.motor_torque_target = trq;

    // CALCULATING MIN RADIUS
    /*
        (fx/mx)**2 + (fy/my)**2 <= fz**2
        (fy/my)**2 <= fz**2 - (fx/mx)**2
        fy = mu**2/R    fx**2 - (fx/mx)**2 = rem
        mu**2/R <= rem
        R >= mu**2 / rem
    */

    // min_radius = model.m * v_x * v_x / rem;
    double mx_head = 3.14159 * 31.2 / 180;
    double mn_radius_wheel = model.wb / std::tan(mx_head);
    double mu = model.my_max(fz);
    mu *= safety_factor; // C_SF3
    min_radius = std::min(v_x * v_x / (mu * model.g), mn_radius_wheel);

    Point tp;
    double ld;
    // std::cout<<"9.. ";
    if (projection.second < emergency_threshold)
    {
        ld = pp_controller.lookahead(v_x, false);
    }
    else
    {
        std::cout<<"emergency manouevre"<<std::endl;
        for_publish.motor_torque_target *= 0.25;
        ld = pp_controller.lookahead(v_x, true);
    }
    // std::cout<<"10.. ";
    // std::cout<<"Ld = "<<ld<<" with v_x = "<<v_x<<std::endl;

    tp = this->profile->get_target_point(ld, position, min_radius, theta);
    double heading_angle;
    if (!tp.error)
        heading_angle = pp_controller(tp, theta, min_radius);
    else
        heading_angle = 0;
    // std::cout<<"12.. ";

    heading_angle = std::min(mx_head, std::max(-mx_head, heading_angle));

    for_publish.steering_angle_target = heading_angle;
    bool switch_br = force < 0 && v_x < safe_speed_to_break;

    for_publish.brake_pressure_target = switch_br;
    if (switch_br)
        for_publish.motor_torque_target = 0;
    // std::cout<<"13.. ";

    double lr = model.wb * model.wd;
    Point drear = Point(-lr * std::cos(theta), -lr * std::sin(theta));
    Point act_target(position.x() - drear.x() + tp.x(), position.y() - drear.y() + tp.y());

    custom_msgs::msg::Point2Struct tg;
    tg.x = act_target.x();
    tg.y = act_target.y();
    pub_target->publish(tg);

    pub_actuators->publish(for_publish);
    std::cout << "Command: Torque = " << std::fixed << std::setprecision(4) << for_publish.motor_torque_target << " Nm, Heading = " << std::fixed << std::setprecision(4) << heading_angle << " rad, BP = " << (is_end && v_x < safe_speed_to_break) ? 1 : 0;
    std::cout << " , ld = " << std::fixed << std::setprecision(4) << ld << std::endl;

    rclcpp::Duration total_time = this->now() - starting_time;
    total_execution_time += total_time.nanoseconds() / 1000000.0;

    std::cout << "Time of Pose Execution: " << total_time.nanoseconds() / 1000000.0 << " ms." << std::endl;
}

void LifecyclePID_PP_Node::parameter_load()
{
    declare_parameter<float>("mass", 193.5);
    declare_parameter<float>("gravitational_acceleration", 9.81);
    declare_parameter<float>("wheelbase", 1.59);
    declare_parameter<float>("weight_distribution", 0.467);
    declare_parameter<float>("h_cog", 0.275);
    declare_parameter<float>("air_density", 1.225);
    declare_parameter<float>("Cd*A", 2);
    declare_parameter<float>("Cl*A", 7);
    declare_parameter<float>("gear_ratio", 3.9);
    declare_parameter<float>("wheel_radius", 0.2054);
    declare_parameter<float>("efficiency", 0.85);
    declare_parameter<float>("Fz_0", 1112.0554070627252);
    declare_parameter<float>("ld_min", 2);
    declare_parameter<float>("ld_max", 7);
    declare_parameter<float>("max_positive_torque", 186.2);
    declare_parameter<float>("max_negative_torque", -186.2);
    declare_parameter<float>("minimum_weight_distribution_rear", 1);
    declare_parameter<float>("c_tire", 0.66);
    declare_parameter<float>("v_min", 0);
    declare_parameter<float>("v_max", 10);
    declare_parameter<float>("emergency_factor", 0.8);
    declare_parameter<float>("emergency_threshold", 0.7);
    declare_parameter<float>("safety_factor", 0.75);
    declare_parameter<float>("max_speed", 15);
    declare_parameter<float>("dt", 0.025);
    declare_parameter<float>("safe_speed_to_break", 1);

    declare_parameter<int>("kp", 1000);
    declare_parameter<int>("ki", 1000);
    declare_parameter<int>("kd", 0);
    declare_parameter<int>("PID_max_output", 3000);
    declare_parameter<int>("Integral_max_output", 1500);
    declare_parameter<int>("spline_resolution_per_meter", 10);
    declare_parameter<int>("total_laps", 1);

    declare_parameter<string>("discipline", "Autocross");
    declare_parameter<string>("midpoints", "");
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