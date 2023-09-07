#include "controller.hpp"
#include <pthread.h>
#include <iomanip>

void PID_PP_Node::parameter_load()
{
    declare_parameter<float>("mass", 193.5);
    declare_parameter<float>("gravitational_acceleration", 9.81);
    declare_parameter<float>("wheelbase", 1.759);
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
    declare_parameter<float>("braking_distance", 5);

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

PID_PP_Node::PID_PP_Node() : Node("PID_PP_controller"), profile(nullptr), model(), pp_controller(), spline(nullptr), pid_controller(), has_run_waypoints(false), count_wp(0), prev_lap(1), last_steering(5), last_torque(0), buffer_ticks(0)
{
    // PARAMETER LOADING
    parameter_load();

    // BASIC OBJECT INITIALIZATION
    model = Model(
        get_parameter("mass").as_double(),
        get_parameter("gravitational_acceleration").as_double(),
        get_parameter("wheelbase").as_double(),
        get_parameter("weight_distribution").as_double(),
        get_parameter("h_cog").as_double(),
        get_parameter("air_density").as_double(),
        get_parameter("Cd*A").as_double(),
        get_parameter("Cl*A").as_double(),
        get_parameter("gear_ratio").as_double(),
        get_parameter("wheel_radius").as_double(),
        get_parameter("efficiency").as_double(),
        get_parameter("Fz_0").as_double(),
        get_parameter("c_tire").as_double(),
        get_parameter("max_positive_torque").as_double(),
        get_parameter("max_negative_torque").as_double(),
        get_parameter("minimum_weight_distribution_rear").as_double());
    pid_controller.init(
        get_parameter("kp").as_int(),
        get_parameter("ki").as_int(),
        get_parameter("kd").as_int(),
        get_parameter("dt").as_double(),
        get_parameter("PID_max_output").as_int(),
        get_parameter("Integral_max_output").as_int());
    pp_controller.init(
        get_parameter("ld_min").as_double(),
        get_parameter("ld_max").as_double(),
        get_parameter("v_min").as_double(),
        get_parameter("v_max").as_double(),
        model.wb,
        get_parameter("emergency_factor").as_double());

    braking_distance = get_parameter("braking_distance").as_double();
    emergency_threshold = get_parameter("emergency_threshold").as_double();
    safety_factor = get_parameter("safety_factor").as_double();
    max_speed = get_parameter("max_speed").as_double();
    safe_speed_to_break = get_parameter("safe_speed_to_break").as_double();
    spline_res_per_meter = get_parameter("spline_resolution_per_meter").as_int();
    laps_to_do = get_parameter("total_laps").as_int();
    discipline = get_parameter("discipline").as_string();
    midpoints = get_parameter("midpoints").as_string();

    is_end = false;

    if (pthread_spin_init(&global_lock_, PTHREAD_PROCESS_SHARED) != 0)
    {
        RCLCPP_ERROR(get_logger(), "Global lock initialization failed: exit program");
        exit(1);
    }

    // CALLBACK GROUPS
    callback_group_waypoints = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // callback_group_pose = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_waypoints;
    /// auto sub_opt_2 = rclcpp::SubscriptionOptions();
    // sub_opt_2.callback_group = callback_group_pose;

    // SUB/PUB CREATION
    sub_waypoints = this->create_subscription<custom_msgs::msg::WaypointsMsg>("waypoints", 10, std::bind(&PID_PP_Node::waypoints_callback, this, _1), sub_opt);
    sub_pose = this->create_subscription<custom_msgs::msg::PoseMsg>("pose", 10, std::bind(&PID_PP_Node::pose_callback, this, _1), sub_opt);
    pub_actuators = this->create_publisher<custom_msgs::msg::TxControlCommand>("control_command", 10);
    pub_target = this->create_publisher<custom_msgs::msg::Point2Struct>("pp_target_point", 10);
    std::cout << "Object created" << std::endl;

    log.open("src/6.Controls/simple_sim/data/target_log.txt");
    // mids.open("src/6.Controls/pid_pp_controller/data/" + midpoints);

    std::cout << "Total laps: " << laps_to_do << std::endl;
}

PID_PP_Node::~PID_PP_Node()
{
    delete profile;
    delete spline;
    std::cout << "Object destroyed" << std::endl;
}
void PID_PP_Node::known_map_substitute(int lap, int total_laps)
{
    buffer_ticks = 0;
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
    }
    else if (discipline == "Acceleration")
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
    }
    else if (discipline == "Skidpad")
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
                midpoints(i + 5, 0) = (7.675 + 1.75) * (std::sin(2 * 3.14159 * ((i * 1.0) / 30)));
                midpoints(i + 5, 1) = (7.675 + 1.75) * (1 - std::cos(2 * 3.14159 * ((i * 1.0) / 30)));
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
                midpoints(i, 0) = (7.675 + 1.75) * (std::sin(2 * 3.14159 * ((i * 1.0) / 40)));
                midpoints(i, 1) = (7.675 + 1.75) * (1 - std::cos(2 * 3.14159 * ((i * 1.0) / 40)));
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
                midpoints(i, 0) = (7.675 + 1.75) * (std::sin(2 * 3.14159 * ((i * 1.0) / 40)));
                midpoints(i, 1) = (7.675 + 1.75) * (1 - std::cos(2 * 3.14159 * ((i * 1.0) / 40)));
            }
            for (int i = 0; i < 20; i++)
            {
                midpoints(i + 40, 0) = (7.675 + 1.75) * (std::sin(2 * 3.14159 * ((i * 1.0) / 40)));
                midpoints(i + 40, 1) = (7.675 + 1.75) * (-1 + std::cos(2 * 3.14159 * ((i * 1.0) / 40)));
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
                midpoints(i, 0) = (7.675 + 1.75) * (std::sin(2 * 3.14159 * ((i * 1.0) / 40)));
                midpoints(i, 1) = (7.675 + 1.75) * (-1 + std::cos(2 * 3.14159 * ((i * 1.0) / 40)));
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
                midpoints(i, 0) = (7.675 + 1.75) * (std::sin(2 * 3.14159 * ((i * 1.0) / 40)));
                midpoints(i, 1) = (7.675 + 1.75) * (-1 + std::cos(2 * 3.14159 * ((i * 1.0) / 40)));
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
    }
}

void PID_PP_Node::waypoints_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr msg)
{
    std::cout << ++count_wp << " Entered Waypoints callback" << std::endl;
    if (is_end)
        return;
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
    double ms = msg->is_out_of_map ? 2 : max_speed;
    this->is_out_of_map = msg->is_out_of_map;
    double v_init = msg->initial_v_x == -1 ? this->v_x : msg->initial_v_x;
    VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, v_init, is_end, true, safety_factor, braking_distance); // last available speed is used. Alternatively should be in waypoints msg
    this->buffer_ticks = is_out_of_map ? -70 : 0;
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

void PID_PP_Node::pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr msg)
{
    // std::cout<<">>> >>> "<<has_run_waypoints<<" "<<!has_run_waypoints<<std::endl;
    std::cout << "Entered Pose Callback" << std::endl;
    // std::cout << "Pos: " << msg->position.x << ", " << msg->position.y << ". theta: " << msg->theta << ". v_o : " << msg->velocity_state.velocity_x << std::endl;
    //  std::cout<<"1.. ";
    buffer_ticks++;
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
        known_map_substitute(1, laps_to_do);
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
    double fz = model.Fz_calc("full", 1, 0, v_x);
    double force = pid_controller(projection.first - this->v_x);
    // Checking Force
    double fx_next = force - 0.5 * v_x * v_x * model.cd_A - 0.03 * fz + v_y * r * model.m;
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
    min_radius = std::max(v_x * v_x / (mu * model.g), mn_radius_wheel);

    Point tp;
    double ld;
    // std::cout<<"9.. ";
    std::cout << ">>> " << projection.second << " " << emergency_threshold << std::endl;
    if (projection.second < emergency_threshold)
    {
        ld = pp_controller.lookahead(v_x, false);
    }
    else if (!is_end)
    {
        std::cout << "emergency manouevre" << std::endl;
        for_publish.motor_torque_target *= 0.25;
        ld = pp_controller.lookahead(v_x, true);
    }
    // std::cout<<"10.. ";
    // std::cout<<"Ld = "<<ld<<" with v_x = "<<v_x<<std::endl;

    tp = this->profile->get_target_point(ld, position, min_radius, theta);
    double heading_angle;
    if (!tp.error)
        heading_angle = pp_controller(tp, theta, min_radius);
    else if(!is_end)
    {
        heading_angle = 0;
        std::cout << "tp error" << std::endl;
    }
    else heading_angle = 0;
    // std::cout<<"12.. ";

    heading_angle = std::min(mx_head, std::max(-mx_head, heading_angle));

    if (last_steering != 5)
        heading_angle = std::min(last_steering + 0.25, std::max(last_steering - 0.25, heading_angle));

    last_steering = heading_angle;

    for_publish.steering_angle_target = heading_angle;
    bool switch_br = force < 0 && v_x < safe_speed_to_break && is_end;

    for_publish.brake_pressure_target = switch_br ? 10.0 : 0.0;
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

    // /* EXTRA SAFETY CHECKS */
    // if (this->v_x > 1.5 * projection.first && this->v_x > projection.first + 1 && !is_end && buffer_ticks > 10)
    // {
    //     for_publish.motor_torque_target = 0;
    //     for_publish.brake_pressure_target = 1;
    //     std::cout << "50% more." << std::endl;
    // }
    // if (this->v_x > 2 * projection.first && this->v_x > projection.first + 1 && !is_end && buffer_ticks > 10)
    // {
    //     for_publish.steering_angle_target = 0;
    //     for_publish.motor_torque_target = 0;
    //     for_publish.brake_pressure_target = 1;
    //     pub_actuators->publish(for_publish);
    //     std::cout << "Controls aborting" << std::endl;
    //     exit(1); // Initiates the ABS. If there is a safer method than exit, it should be preffered
    // }

    pub_actuators->publish(for_publish);
    std::cout << "Command: Torque = " << std::fixed << std::setprecision(4) << for_publish.motor_torque_target << " Nm, Heading = " << std::fixed << std::setprecision(4) << heading_angle << " rad, BP = " << switch_br ? 1 : 0;
    std::cout << " , ld = " << std::fixed << std::setprecision(4) << ld << std::endl;

    rclcpp::Duration total_time = this->now() - starting_time;
    total_execution_time += total_time.nanoseconds() / 1000000.0;

    std::cout << "Time of Pose Execution: " << total_time.nanoseconds() / 1000000.0 << " ms." << std::endl;
}

/* REMAINING TASKS */
/*
    -> cmake
    -> package
    -> config
    -> launch
*/

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<PID_PP_Node>());
    auto options = rclcpp::ExecutorOptions();

    rclcpp::Node::SharedPtr node = std::make_shared<PID_PP_Node>();
    rclcpp::executors::MultiThreadedExecutor executor{options, 2};
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}