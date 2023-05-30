#include "controller.hpp"
#include <pthread.h>

void PID_PP_Node::parameter_load()
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
}

PID_PP_Node::PID_PP_Node() : Node("PID_PP_controller"), profile(nullptr), model(), pp_controller(), spline(nullptr), pid_controller(), has_run_waypoints(false)
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
        get_parameter("minimum_weight_distribution_rear").as_double()
    );
    pid_controller.init(
        get_parameter("kp").as_int(),
        get_parameter("ki").as_int(),
        get_parameter("kd").as_int(),
        get_parameter("dt").as_double(),
        get_parameter("PID_max_output").as_int(),
        get_parameter("Integral_max_output").as_int()
    );
    pp_controller.init(
        get_parameter("ld_min").as_double(),
        get_parameter("ld_max").as_double(),
        get_parameter("v_min").as_double(),
        get_parameter("v_max").as_double(),
        model.wb,
        get_parameter("emergency_factor").as_double()
    );

    emergency_threshold = get_parameter("emergency_threshold").as_double();
    safety_factor = get_parameter("safety_factor").as_double();
    max_speed = get_parameter("max_speed").as_double();
    safe_speed_to_break = get_parameter("safe_speed_to_break").as_double();
    spline_res_per_meter = get_parameter("spline_resolution_per_meter").as_int();
    laps_to_do = get_parameter("total_laps").as_int();

    is_end = false;

    // Initialize global lock
    if (pthread_spin_init(&global_lock_, PTHREAD_PROCESS_SHARED) != 0)
    {
        RCLCPP_ERROR(get_logger(), "Global lock initialization failed: exit program");
        exit(1);
    }
    // SUB/PUB CREATION
    sub_waypoints = this->create_subscription<custom_msgs::msg::WaypointsMsg>("waypoints", 10, std::bind(&PID_PP_Node::waypoints_callback, this, _1));
    sub_pose = this->create_subscription<custom_msgs::msg::PoseMsg>("pose", 10, std::bind(&PID_PP_Node::pose_callback, this, _1));
    pub_actuators = this->create_publisher<custom_msgs::msg::TxControlCommand>("control_command", 10);
}

PID_PP_Node::~PID_PP_Node()
{
    delete profile;
    delete spline;
}

void PID_PP_Node::waypoints_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr msg)
{

    // Object variables should only be updated in the end
    path_planning::PointsArray midpoints(msg->count, 2);
    for (int i = 0; i < msg->count; i++)
    {
        midpoints(i, 0) = msg->waypoints[i].x;
        midpoints(i, 1) = msg->waypoints[i].y;
    }
    path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
    bool is_end = msg->lap_count == laps_to_do;
    double ms = is_end ? 0 : max_speed;
    VelocityProfile *profile = new VelocityProfile(*spline, ms, spline_res_per_meter, model, msg->initial_v_x, is_end); // last available speed is used. Alternatively should be in waypoints msg

    /*
        To minimize time spent with locked object variables, we make it so that the bare minimum of operations is done. We store the modifiable objects(spline, profile) as pointers. Thus we achieve 2 things
        (a) The new objects can be constructed locally and the modification required is only the copying of the pointer address
        (b) The old objects can be destroyed locally as we store their old address in local variables and delete them independently
        As a result during lockdown we do only 4 address copies and a bool copy
    */

    /* VARIABLE LOCK */
    pthread_spin_lock(&global_lock_);
    path_planning::ArcLengthSpline *spline_to_delete = this->spline;
    VelocityProfile *profile_to_delete = this->profile;
    this->has_run_waypoints = true;
    this->is_end = is_end;
    this->profile = profile;
    this->spline = spline;
    /* VARIABLE UNLOCK */
    pthread_spin_unlock(&global_lock_);

    delete spline_to_delete;
    delete profile_to_delete;
}

void PID_PP_Node::pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr msg)
{
    if (!has_run_waypoints)
        return;

    v_x = msg->velocity_state.velocity_x;
    v_y = msg->velocity_state.velocity_y;
    r = msg->velocity_state.yaw_rate;
    a_x = msg->velocity_state.acceleration_x;
    a_y = msg->velocity_state.acceleration_y;

    Point position(msg->position.x, msg->position.y);
    // Point direction(std::cos(theta), std::sin(theta));
    double theta = msg->theta;

    std::pair<double, double> projection = (*this->profile)(position, theta);

    custom_msgs::msg::TxControlCommand for_publish;
    for_publish.speed_actual = this->v_x;
    for_publish.speed_target = projection.first;

    double min_radius;
    double force = pid_controller(projection.first - this->v_x);
    // Checking Force
    double fx_next = force - 0.5 * v_x * v_x * model.cd_A + v_y * r * model.m;
    double fx = std::max(model.m * a_x, fx_next); // old fx, new fx
    // COG TBD
    double fz = model.Fz_calc("full", 1,0, v_x);
    double rem = fz * fz - std::pow(fx / model.mx_max(fz), 2);
    if (rem < 0)
    {
        // exei ginei malakia
        if (model.m * a_x > fx_next)
        {
            // spiniaroume right now. HANDLING TBD
        }
        else
        {
            // tha spiniaroume otan efarmostei
            force = 0.5 * v_x * v_x - v_y * r * model.m + safety_factor * (model.mx_max(fz) * fz);
            fx_next = force - 0.5 * v_x * v_x * model.cd_A + v_y * r * model.m;
            fx = std::max(model.m * a_x, fx_next);
            rem = fz * fz - std::pow(fx / model.mx_max(fz), 2);
        }
    }
    for_publish.motor_torque_target = model.Torque(force);

    // CALCULATING MIN RADIUS
    /*
        (fx/mx)**2 + (fy/my)**2 <= fz**2
        (fy/my)**2 <= fz**2 - (fx/mx)**2
        fy = mu**2/R    fx**2 - (fx/mx)**2 = rem
        mu**2/R <= rem
        R >= mu**2 / rem
    */
    min_radius = model.m * v_x * v_x / rem;
    Point tp;
    double ld;
    if (projection.second < emergency_threshold)
        ld = pp_controller.lookahead(v_x, false);
    else
        ld = pp_controller.lookahead(v_x, true);

    tp = this->profile->get_target_point(ld, position, min_radius, theta);
    //double R1 = ld * ld / (2 * (-tp.x() * std::sin(theta) + tp.y() * std::cos(theta)));
    //double R = R1 > 0 ? std::max(min_radius, R1) : std::min(-min_radius, R1);

    double heading_angle = pp_controller(tp, theta, min_radius);

    for_publish.steering_angle_target = heading_angle;
    for_publish.brake_pressure_target = is_end && v_x < safe_speed_to_break;

    pub_actuators->publish(for_publish);
}

/* REMAINING TASKS */
/*
    -> cmake
    -> package
    -> config
    -> launch
*/

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PID_PP_Node>());
    rclcpp::shutdown();
    return 0;
}