#include "lifecycle_mpc_node.hpp"

using namespace mpc;
using namespace path_planning;
using namespace std::chrono_literals;
namespace mpc {
    LifecycleMpcHandler::LifecycleMpcHandler():LifecycleNode("mpc"), count_(0) {
        declareParameters();
        std::cout << "simulation param is: " << mpc_solver.simulation_ << std::endl;
        RCLCPP_WARN(get_logger(), "\n-- MPC Node Created");
    }

    void LifecycleMpcHandler::setSubscribers() {
        pose_subscriber_ = this->create_subscription<custom_msgs::msg::PoseMsg>("pose", 10, std::bind(&LifecycleMpcHandler::pose_callback, this, std::placeholders::_1));
    }

    void LifecycleMpcHandler::setPublishers() {
        mpc_publisher_ = create_publisher<custom_msgs::msg::TxControlCommand>("control_commands", 10);
    }

    void LifecycleMpcHandler::setClient() {
        //client for total-laps request
        auto response_received_callback = [this](rclcpp::Client<custom_msgs::srv::SetTotalLaps>::SharedFuture future) {
            auto result = future.get();
            if (result->success) \
                RCLCPP_INFO(get_logger(), "Total mission laps set successfully");
        };

        if (!total_laps_client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "P23 Status service is not available");
        }
        else {
            auto request = std::make_shared<custom_msgs::srv::SetTotalLaps::Request>();
            request->total_laps = mpc_solver.total_laps_;
            auto future_result = total_laps_client->async_send_request(request, response_received_callback);
        }
    }

    void LifecycleMpcHandler::declareParameters() {
        declare_parameter<std::string>("mission","skidpad");
        declare_parameter<bool>("simulation",false);
        declare_parameter<float>("s_interval",0.1);
        declare_parameter<float>("distance_safe",1.0);
        declare_parameter<float>("emergency_forward",1.0);
        declare_parameter<float>("F_init",300.0);
        declare_parameter<float>("v_limit",15.0);
        declare_parameter<float>("node_freq",40.0);
        declare_parameter<float>("s_space_max",0.5);
        declare_parameter<float>("s_space_min",0.1);
        declare_parameter<int>("total_laps",5); 
    }

    void LifecycleMpcHandler::loadParameters() {
        global_int=-1;
        mpc_solver.mission_ = get_parameter("mission").as_string();
        mpc_solver.simulation_= get_parameter("simulation").as_bool();
        mpc_solver.generateTrackConfig();
        mpc_solver.s_interval_ = get_parameter("s_interval").as_double();
        mpc_solver.distance_safe_ = get_parameter("distance_safe").as_double();
        mpc_solver.emergency_forward_ = get_parameter("emergency_forward").as_double();
        mpc_solver.F_init = get_parameter("F_init").as_double();
        mpc_solver.v_limit_ = get_parameter("v_limit").as_double();
        node_freq_ = get_parameter("node_freq").as_double();
        mpc_solver.dt = (float)(1/node_freq_);
        mpc_solver.s_space_max = get_parameter("s_space_max").as_double();
        mpc_solver.s_space_min = get_parameter("s_space_min").as_double();
        mpc_solver.total_laps_ = get_parameter("total_laps").as_int();
        std::cout << "param is: " << mpc_solver.known_track_ << " " << mpc_solver.simulation_ << " " << node_freq_ << std::endl;
        std::cout << "declared params" << std::endl;
    }
        
    void LifecycleMpcHandler::ReadKnownTrack() {
        std::cout << "File to read from is: " << mpc_solver.midpoints_txt_ << std::endl;
        Eigen::MatrixXd spline_input = readTrack(mpc_solver.midpoints_txt_);
        path_planning::PointsArray midpoints{spline_input};
        midpoints.conservativeResize(midpoints.rows(), midpoints.cols());
        // midpoints.row(midpoints.rows() - 1) = midpoints.row(0);
        std::cout << "midpoints are: " << midpoints << std::endl;
        path_planning::ArcLengthSpline *spline_init = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::NaturalSpline);
        mpc_solver.spline_final = spline_init;
        mpc_solver.spline_resolution = int (spline_init->getApproximateLength()/mpc_solver.s_interval_);
        mpc_solver.sol = spline_init->getApproximateLength(); 
        std::cout << "length of track and resolution are: "<< mpc_solver.sol << " " << mpc_solver.spline_resolution << std::endl;
        mpc_solver.whole_track = spline_init->getSplineData(mpc_solver.spline_resolution);
        path_flag=1;
        std::cout << "read known track" << std::endl;
    }

    void LifecycleMpcHandler::pose_callback(const custom_msgs::msg::PoseMsg::SharedPtr pose_msg) {
        std::cout << "Im at mpc iteration " << global_int + 1 << std::endl;
        std::cout << "Im at mission: " << mpc_solver.mission_ << " and lap: " << mpc_solver.lap_counter << " out of " << mpc_solver.total_laps_ << " laps." << std::endl;
        std::cout << "Finish and brake flags are: " << mpc_solver.finish_flag << " " << mpc_solver.brake_flag << std::endl;
        mpc_solver.pose_struct.theta = float(pose_msg->theta);
        mpc_solver.pose_struct.x = float(pose_msg->position.x);
        mpc_solver.pose_struct.y = float(pose_msg->position.y);
        mpc_solver.vel_struct.velocity_x = float(pose_msg->velocity_state.velocity_x);
        mpc_solver.vel_struct.velocity_y = float(pose_msg->velocity_state.velocity_y);
        mpc_solver.vel_struct.yaw_rate = float(pose_msg->velocity_state.yaw_rate);
        mpc_solver.lap_counter_official = uint8_t(pose_msg->lap_count);
        if(!mpc_solver.simulation_) { 
            std::cout << "I get pose " << mpc_solver.pose_struct.x << " " << mpc_solver.pose_struct.y << " " << mpc_solver.pose_struct.theta << std::endl;
            std::cout << "I get velocity " << mpc_solver.vel_struct.velocity_x << " " << mpc_solver.vel_struct.velocity_y << " " << mpc_solver.vel_struct.yaw_rate << std::endl;
            std::cout << "I get lap counter " << mpc_solver.lap_counter_official << std::endl;
        }
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
            if(global_int==-1) mpc_solver.Initialize_all_local();
            mpc_solver.UpdateFromLastIteration();
            mpc_solver.checkReliability();
            mpc_solver.customLapCounter();
            mpc_solver.generateFinishFlag(mpc_solver.lap_counter);
            if(mpc_solver.mission_=="skidpad") mpc_solver.updateSkidpadSpline(mpc_solver.lap_counter);
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
        std::cout << "Publishing brake pressure: " << mpc_msg.brake_pressure_target << std::endl;
        RCLCPP_INFO(this->get_logger(), "Publishing motor torque: %.6f" " ,wheel angle: %.6f" "",mpc_msg.motor_torque_target, 57.2958*mpc_msg.steering_angle_target);
        mpc_publisher_->publish(mpc_msg);
        rclcpp::Duration total_time = this->now() - starting_time;
        total_execution_time += total_time.nanoseconds() / 1000000.0;
        std::cout << "Time of mpc Execution: "<<total_time.nanoseconds() / 1000000.0 << " ms." <<std::endl;
    }

    void LifecycleMpcHandler::path_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr path_msg) {
        std::cout << "mpika path callback" << std::endl;
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
        //wrong needs to be changed
        std::vector<double> target_lengths_init;
        target_lengths_init.push_back(0.0);
        for (int i{ 1 }; i < path_msg->count; i++) {
            target_lengths_init.push_back(target_lengths_init[i - 1] + \
                std::sqrt(std::pow(spline_input(i, 0) - spline_input(i - 1, 0), 2) + std::pow(spline_input(i, 1) - spline_input(i - 1, 1), 2)));
        }
        double sol_temp = target_lengths_init[path_msg->count -1];
        std::cout << "sol is: " << sol_temp << std::endl;
        int points = int (sol_temp/mpc_solver.s_interval_);
        if(points <40) points = 40;
        std::cout << "points for splines are " << points <<std::endl;
        path_planning::PointsArray midpoints{spline_input};
        midpoints.conservativeResize(midpoints.rows()+1 , midpoints.cols());
        midpoints.row(midpoints.rows() - 1) = midpoints.row(0);
        path_planning::ArcLengthSpline spline{midpoints, path_planning::BoundaryCondition::Anchored};
        path_planning::ArcLengthSpline *spline_init = new path_planning::ArcLengthSpline(midpoints, path_planning::BoundaryCondition::Anchored);
        PointsData params_array_bef;
        mpc_solver.spline_final = spline_init;
        mpc_solver.sol = spline.getApproximateLength();
        params_array_bef = spline.getSplineData(points);
        mpc_solver.params_array = params_array_bef.topRows(40);
        std::cout << "first point of path callback is: " << mpc_solver.params_array(0,0) << " " << mpc_solver.params_array(0,1) << std::endl;
        std::cout << "finished path callback" << std::endl;
        if(path_flag==0) path_flag=1;
}

    LifecycleMpcHandler::~LifecycleMpcHandler() {
        // delete mpc_solver.spline_final;
        std::cout << "mpc node destroyed!" << std::endl;
    }
}//namespace mpc

int main(int argc, char * argv[]) {  
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto mpc_node = std::make_shared<mpc::LifecycleMpcHandler>();
  executor.add_node(mpc_node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}