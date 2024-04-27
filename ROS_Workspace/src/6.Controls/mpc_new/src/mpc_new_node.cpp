#include "mpc_new_node.h"

/*CHECK THE LIFECYCLE NODE, THIS ONE WAS USED FOR TESTS IN THE SIMULATION ONLY*/

void Handler::parameterload(){
    declare_parameter<double>("max_velocity", 3.0);
    declare_parameter<double>("ds", 0.15);
    declare_parameter<double>("dt", 0.025);
    declare_parameter<int>("horizon", 30);
    declare_parameter<double>("F_init", 300.0);
    declare_parameter<double>("F_max", 1000.0);
    declare_parameter<double>("F_min", -1000.0);
    declare_parameter<std::string>("mission", "skidpad");
    declare_parameter<double>("time_delay", 0.1);
    declare_parameter<double>("T_max", 25.0);
    declare_parameter<double>("T_min", -25.0);
    declare_parameter<double>("angle_max", 0.436332157);
    declare_parameter<double>("angle_min", -0.436332157);
    declare_parameter<double>("wb", 1590.0);
    declare_parameter<double>("wd_front", 0.467);
    declare_parameter<double>("CdA", 2.0);
    declare_parameter<double>("ClA", 7.0);
    declare_parameter<double>("p_air", 1.225);
    declare_parameter<double>("h_cog", 0.27);
    declare_parameter<double>("gr", 3.9);
    declare_parameter<double>("Rw", 0.2);
    declare_parameter<double>("m", 190.0);
    declare_parameter<double>("g", 9.81);
    declare_parameter<double>("Iz", 110.0);
    declare_parameter<int>("N_rear", 2);
    declare_parameter<double>("d_piston", 0.025);
    declare_parameter<double>("R_disk_f", 0.079);
    declare_parameter<double>("R_disk_r", 0.0735);
    declare_parameter<double>("mi_disk", 0.6);
    declare_parameter<bool>("dynamic_ds", false);
}

Handler::Handler() : Node("NewMPC_Controller"){
    //Loading the parameters
    parameterload();

    double F_init = get_parameter("F_init").as_double();
    int horizonLength = get_parameter("horizon").as_int();
    double ds = get_parameter("ds").as_double();
    double dt = get_parameter("dt").as_double();
    double vel_max = get_parameter("max_velocity").as_double();
    double maxF = get_parameter("F_max").as_double();
    double minF = get_parameter("F_min").as_double();
    std::string mission = get_parameter("mission").as_string();
    double time_delay = get_parameter("time_delay").as_double();
    double T_max = get_parameter("T_max").as_double();
    double T_min = get_parameter("T_min").as_double();
    double angle_max = get_parameter("angle_max").as_double();
    double angle_min = get_parameter("angle_min").as_double();
    double wb = get_parameter("wb").as_double();
    double wd_front = get_parameter("wd_front").as_double();
    double CdA = get_parameter("CdA").as_double();
    double ClA = get_parameter("ClA").as_double();
    double p_air = get_parameter("p_air").as_double();
    double h_cog = get_parameter("h_cog").as_double();
    double gr = get_parameter("gr").as_double();
    double Rw = get_parameter("Rw").as_double();
    double m = get_parameter("m").as_double();
    double g = get_parameter("g").as_double();
    double Iz = get_parameter("Iz").as_double();
    int N_rear = get_parameter("N_rear").as_int();
    double d_piston = get_parameter("d_piston").as_double();
    double R_disk_f = get_parameter("R_disk_f").as_double();
    double R_disk_r = get_parameter("R_disk_r").as_double();
    double mi_disk = get_parameter("mi_disk").as_double();
    bool dynamic_ds = get_parameter("dynamic_ds").as_bool();
    //alvania
    mpc_solver = new_MpcSolver(F_init, horizonLength, ds, dt, vel_max, maxF, minF, mission, time_delay, T_max, T_min, angle_max, angle_min,
        wb, wd_front, CdA, ClA, p_air, h_cog, gr, Rw, m, g, Iz, N_rear, d_piston, R_disk_f, R_disk_r, mi_disk, dynamic_ds);

    //Next build the publishers and the subscibers
    auto sub_opt = rclcpp::SubscriptionOptions();

    //publishers
    dc_publisher = this->create_publisher<custom_msgs::msg::TxControlCommand>("control_commands", 10);
    //subscribers
    pose_subscriber = this->create_subscription<custom_msgs::msg::PoseMsg>("pose", 10, std::bind(&Handler::pose_callback, this, _1), sub_opt);
    path_subscriber = this->create_subscription<custom_msgs::msg::WaypointsMsg>("waypoints", 10, std::bind(&Handler::waypoints_callback, this, _1), sub_opt);

    spline = nullptr;
}

Handler::~Handler(){}

//This callback takes the message from the path planning and here we can actually do some 
//checks that this message includes and create the spline
void Handler::waypoints_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr path_msg){
    //Clearing from the previous spline
    if(!this->spline){
        free(this->spline);
        this->spline = nullptr;
    }
    mpc_solver.total_laps = path_msg->lap_count; 
    
    //Building the spline, checking the number of points the path planning gave
    path_planning::PointsArray midpoints(std::max(3, int(path_msg->count)), 2);
    if(path_msg->count >= 3){
        for(int i = 0; i < path_msg->count; i++){
            midpoints(i, 0) = path_msg->waypoints[i].x;
            midpoints(i, 1) = path_msg->waypoints[i].y;
        }
    }
    //In case only 2 points where returned, just populate with one more point to make them 3
    else if (path_msg->count == 2){
        //First point
        midpoints(0, 0) = path_msg->waypoints[0].x;
        midpoints(0, 1) = path_msg->waypoints[0].y;
        //Second point (the median of the 2)
        midpoints(1, 0) = 0.5*(path_msg->waypoints[0].x+path_msg->waypoints[1].x);
        midpoints(1, 1) = 0.5*(path_msg->waypoints[0].y+path_msg->waypoints[1].y);
        //Third point
        midpoints(2, 0) = path_msg->waypoints[1].x;
        midpoints(2, 1) = path_msg->waypoints[1].y;
    }
    //Well, if we reach here there is a problem
    else{
        return;
    }
    path_planning::ArcLengthSpline *spline = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::NaturalSpline);
    this->spline = spline;
    mpc_solver.spline = spline;
}

void Handler::pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr pose_msg){
    //Take data
    double X[8];
    X[0] = (float)pose_msg->position.x;
    X[1] = (float)pose_msg->position.y;
    X[2] = (float)pose_msg->theta;
    X[3] = (float)pose_msg->velocity_state.velocity_x;
    X[4] = (float)pose_msg->velocity_state.velocity_y;
    X[5] = (float)pose_msg->velocity_state.yaw_rate;
    X[6] = 0;
    X[7] = 0;

    mpc_solver.total_laps = pose_msg->lap_count; 

    if(spline != nullptr || mpc_solver.mission != "autocross" || mpc_solver.mission != "trackdrive"){
        //Build track first
        if(mpc_solver.mission == "autocross" || mpc_solver.mission == "trackdrive"){
            int spline_resolution = int (spline->getApproximateLength()/mpc_solver.s_interval_);
            mpc_solver.whole_track = this->spline->getSplineData(spline_resolution);
        }
        //Solve
        mpc_solver.build(X);

        //Return data output_struct
        auto mpc_msg = custom_msgs::msg::TxControlCommand(); 
        mpc_msg.speed_target = mpc_solver.output_struct.speed_target;
        mpc_msg.speed_actual = mpc_solver.output_struct.speed_actual;
        mpc_msg.motor_torque_target = mpc_solver.output_struct.motor_torque_target;
        mpc_msg.steering_angle_target = mpc_solver.output_struct.steering_angle_target;
        mpc_msg.brake_pressure_target = mpc_solver.output_struct.brake_pressure_target;
        std::cout << "The lap is----->" << mpc_solver.total_laps << std::endl;
        std::cout << "The target torque is: " << mpc_solver.output_struct.motor_torque_target << std::endl;
        std::cout << "The target velocity is: " << mpc_solver.output_struct.speed_target << std::endl;
        std::cout << "The current velocity is " << X[3] << std::endl;
        //and publish it (send it to the CAN-Reader)
        this->dc_publisher->publish(mpc_msg);
    }
}


//Finally this is the step where everything starts
int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Handler>());
    rclcpp::shutdown();
    return 0;
}