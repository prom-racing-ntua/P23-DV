#include "mpc_new_node.h"

void Handler::parameterload(){
    declare_parameter<double>("max_velocity", 3.0);
    declare_parameter<double>("ds", 0.15);
    declare_parameter<double>("dt", 0.025);
    declare_parameter<int>("horizon", 30);
    declare_parameter<int>("laps", 5);
    declare_parameter<double>("F_init", 300.0);
    declare_parameter<double>("F_max", 1000.0);
    declare_parameter<double>("F_min", -1000.0);
    declare_parameter<std::string>("mission", "autocross");
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
    int laps = get_parameter("laps").as_int();
    std::string mission = get_parameter("mission").as_string();
    mpc_solver = new_MpcSolver(F_init, horizonLength, ds, dt, vel_max, maxF, minF, mission, laps);

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

    if(spline != nullptr || mpc_solver.mission != "autocross"){
        //Build track first
        if(mpc_solver.mission == "autocross"){
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