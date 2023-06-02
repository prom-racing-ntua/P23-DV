#include "mpc_node_official.h"

using namespace mpc;
using namespace path_planning;
using namespace std::chrono_literals;
namespace mpc {
MpcHandler::MpcHandler():Node("mpc"), count_(0){
    // mpc_solver.InitializeMpcSolver();
    loadParameters();
    mpc_solver.mem = FORCESNLPsolver_internal_mem(0);
    std::cout << "mpika mpc constructor" << std::endl;
    if(!mpc_solver.known_track_){
        path_subscriber_ = this->create_subscription<custom_msgs::msg::WaypointsMsg>("waypoints", 10, std::bind(&MpcHandler::path_callback, this, std::placeholders::_1));
    }
    else {
        ReadKnownTrack();
    }

    pose_subscriber_ = this->create_subscription<custom_msgs::msg::PoseMsg>("pose", 10, std::bind(&MpcHandler::pose_callback, this, std::placeholders::_1));
    mpc_publisher_ = this->create_publisher<custom_msgs::msg::TxControlCommand>("tx_control", 10);
    mpc_clock_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000/node_freq_)),std::bind(&MpcHandler::mpc_callback, this));
}

void MpcHandler::loadParameters() {
    mpc_solver.known_track_= declare_parameter<bool>("known_track",true); //they are set to false values to test yaml files
    mpc_solver.simulation_= declare_parameter<bool>("simulation",true);
    mpc_solver.midpoints_txt_= declare_parameter<std::string>("midpoints_txt","src/6.Controls/mpc/data/thomaidio_midpoints.txt");
    mpc_solver.search_window_ = declare_parameter<int>("search_window",15);
    mpc_solver.s_interval_ = declare_parameter<float>("s_interval",15);
    mpc_solver.distance_safe_ = declare_parameter<float>("distance_safe",1000.0);
    mpc_solver.emergency_forward_ = declare_parameter<float>("emergency_forward",2.2);
    mpc_solver.F_init = declare_parameter<float>("F_init",1111.0);
    mpc_solver.v_limit_ = declare_parameter<float>("v_limit",1111.0);
    node_freq_ = declare_parameter<int>("node_freq",50);
    mpc_solver.dt = (1/node_freq_);
    std::cout << "param is: " << mpc_solver.known_track_ << " " << mpc_solver.simulation_ << " " << node_freq_ << std::endl;
    std::cout << "declared params" << std::endl;
}
    

void MpcHandler::pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr pose_msg) {
    std::cout << "mpika pose_call" << std::endl;
    mpc_solver.pose_struct.theta = float(pose_msg->theta);
    mpc_solver.pose_struct.x = float(pose_msg->position.x);
    mpc_solver.pose_struct.y = float(pose_msg->position.y);
    mpc_solver.vel_struct.velocity_x = float(pose_msg->velocity_state.velocity_x);
    mpc_solver.vel_struct.velocity_y = float(pose_msg->velocity_state.velocity_y);
    mpc_solver.vel_struct.yaw_rate = float(pose_msg->velocity_state.yaw_rate);
    std::cout << "I get pose " << mpc_solver.pose_struct.x << " " << mpc_solver.pose_struct.y << " " << mpc_solver.pose_struct.theta << std::endl;
    std::cout << "I get velocity " << mpc_solver.vel_struct.velocity_x << " " << mpc_solver.vel_struct.velocity_y << " " << mpc_solver.vel_struct.yaw_rate << std::endl;
}

void MpcHandler::path_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr path_msg) {
    std::cout << "mpika path callback" << std::endl;
    if(path_flag==0) path_flag=1;
    int pp_points = int(path_msg->count);
    Eigen::MatrixXd spline_input{path_msg->count, 2};
    if(pp_points==2) spline_input.resize(pp_points+1,2);
    int rows_temp=0;
    for (custom_msgs::msg::Point2Struct midpoints_new : path_msg->waypoints) {
    if((pp_points==2) && (rows_temp==1)) {
        spline_input(rows_temp,0) = (spline_input(0,0) + midpoints_new.x)/2;
        spline_input(rows_temp,1) = (spline_input(0,1) + midpoints_new.y)/2;
        rows_temp++;
        spline_input(rows_temp,0) = midpoints_new.x;
        spline_input(rows_temp,1) = midpoints_new.y;
        break;
    }
    spline_input(rows_temp,0) = midpoints_new.x;
    spline_input(rows_temp,1) = midpoints_new.y;
    rows_temp++;
    }
    float sol_temp = std::sqrt(std::pow(spline_input(0,0)-spline_input(path_msg->count - 1,0),2) + std::pow(spline_input(0,1)-spline_input(path_msg->count - 1,1),2));
    std::cout << "sol is: " << sol_temp << std::endl;
    int points = int (sol_temp/mpc_solver.s_interval_);
    if(points <40) points = 40;
    std::cout << "points for splines are " << points <<std::endl;
    path_planning::PointsArray midpoints{spline_input};
    midpoints.conservativeResize(midpoints.rows()+1 , midpoints.cols());
    midpoints.row(midpoints.rows() - 1) = midpoints.row(0);
    path_planning::ArcLengthSpline spline{midpoints, path_planning::BoundaryCondition::Anchored};
    PointsData params_array_bef;
    mpc_solver.sol = spline.getApproximateLength();
    params_array_bef = spline.getSplineData(points);
    mpc_solver.params_array = params_array_bef.topRows(40);
    global_int++;
    std::cout << "finished path callback" << std::endl;
}

void MpcHandler::ReadKnownTrack() {
    std::cout << "Read points" << std::endl;
    Eigen::MatrixXd spline_input = readTrack(mpc_solver.midpoints_txt_);
    path_planning::PointsArray midpoints{spline_input};
    midpoints.conservativeResize(midpoints.rows() + 1, midpoints.cols());
    midpoints.row(midpoints.rows() - 1) = midpoints.row(0);
    path_planning::ArcLengthSpline *spline_init = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
    mpc_solver.spline_final = spline_init;
    mpc_solver.spline_resolution = int (spline_init->getApproximateLength()/mpc_solver.s_interval_);
    mpc_solver.sol = spline_init->getApproximateLength(); 
    std::cout << "length of track is: "<< mpc_solver.sol << std::endl;
    mpc_solver.whole_track = spline_init->getSplineData(mpc_solver.spline_resolution);
    path_flag=1;
    global_int++;
    std::cout << "read known track" << std::endl;
}

void MpcHandler::mpc_callback() {
    std::cout << "Im at mpc iteration " << global_int + 1 << std::endl;
    auto mpc_msg = custom_msgs::msg::TxControlCommand();  
    rclcpp::Time starting_time = this->now();
    if(path_flag==0) {
        std::cout << "havent data yet" << std::endl;
        mpc_msg.speed_target = (float)(0.0);
        mpc_msg.speed_actual = (float)(0.0);
        mpc_msg.motor_torque_target = (float)(0.0);
        mpc_msg.steering_angle_target = (float)(0.0);
        mpc_msg.brake_pressure_target = (bool)(0);
    }
    else {
        if(global_int==0) mpc_solver.Initialize_all_local();
        mpc_solver.UpdateFromLastIteration();
        if(mpc_solver.known_track_) {
            mpc_solver.generateFirstPoint();
            mpc_solver.writeParamsKnown(global_int);
        }
        else {
            mpc_solver.generateFirstPointUnknown();
            mpc_solver.writeParamsUnknown();
        }
        mpc_solver.callSolver(global_int);
        mpc_solver.generateOutput();         
        //define message to ROS2
        mpc_msg.speed_target = mpc_solver.output_struct.speed_target;
        mpc_msg.speed_actual = mpc_solver.output_struct.speed_target;
        mpc_msg.motor_torque_target = mpc_solver.output_struct.motor_torque_target;
        mpc_msg.steering_angle_target = mpc_solver.output_struct.steering_angle_target;
        mpc_msg.brake_pressure_target = mpc_solver.output_struct.brake_pressure_target;
        global_int++;
    }
    RCLCPP_INFO(this->get_logger(), "Publishing motor torque: %.6f" " ,wheel angle: %.6f" " ,brake pressure: %.6f" , mpc_msg.motor_torque_target, mpc_msg.steering_angle_target, mpc_msg.brake_pressure_target);
    mpc_publisher_->publish(mpc_msg);
    rclcpp::Duration total_time = this->now() - starting_time;
    total_execution_time += total_time.nanoseconds() / 1000000.0;
    std::cout << "Time of mpc Execution: "<<total_time.nanoseconds() / 1000000.0 << " ms." <<std::endl;
}


MpcHandler::~MpcHandler(){
    // delete mpc_solver.spline_final;
    std::cout << "mpc node destroyed!" << std::endl;
}
}//namespace mpc

int main(int argc, char * argv[]) {  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcHandler>());
  rclcpp::shutdown();
  return 0;
}