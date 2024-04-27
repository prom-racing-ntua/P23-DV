#include "l_mpc_new.h"
using namespace mpc_new;

void LifecycleHandler::parameterload(){
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

LifecycleHandler::LifecycleHandler() : LifecycleNode("NewMPC_Controller"){
    //Loading the parameters
    parameterload();
    RCLCPP_WARN(get_logger(), "\n-- MPC Node Created");
}

LifecycleHandler::~LifecycleHandler(){}

//This callback takes the message from the path planning and here we can actually do some 
//checks that this message includes and create the spline
void LifecycleHandler::waypoints_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr path_msg){
    //Clearing from the previous spline
    if(!this->spline){
        free(this->spline);
        this->spline = nullptr;
    }
    should_exit = path_msg->should_exit;
    mpc_solver.is_out_of_map = path_msg->is_out_of_map;
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
}

//When the data is taken this callback is executed
void LifecycleHandler::pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr pose_msg){
    rclcpp::Time starting_time = this->now();
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

    if(should_exit)
        return;
    else if(spline != nullptr || !(mpc_solver.mission == "autocross" || mpc_solver.mission == "trackdrive")){
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
        std::cout << "The target torque is: " << mpc_solver.output_struct.motor_torque_target << std::endl;
        std::cout << "The target velocity is: " << mpc_solver.output_struct.speed_target << std::endl;
        std::cout << "The current velocity is " << X[3] << std::endl;
        //and publish it (send it to the CAN-Reader)
        pub_time_1 = this->now().nanoseconds()/1e6;
        this->dc_publisher->publish(mpc_msg);
        pub_time_2 = this->now().nanoseconds()/1e6;
    }
    rclcpp::Duration total_time = this->now() - starting_time;
    total_execution_time += total_time.nanoseconds() / 1000000.0;

    std::cout << "Time of Pose Execution: " << total_time.nanoseconds() / 1000000.0 << " ms." << std::endl;

    pose_timestamp_log.log(starting_time.nanoseconds()/1e6, 0, pose_msg->velocity_state.global_index);
    pose_timestamp_log.log((pub_time_2 + pub_time_1)/2, 1, pose_msg->velocity_state.global_index);
}


//Finally this is the step where everything starts
int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto mpc_node = std::make_shared<LifecycleHandler>();
    executor.add_node(mpc_node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}