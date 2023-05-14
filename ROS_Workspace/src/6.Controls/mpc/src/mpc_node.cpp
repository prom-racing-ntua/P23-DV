#include "mpc_node.h"

using namespace mpc_cpp;
using namespace path_planning;
using namespace std::chrono_literals;

namespace mpc_cpp {
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MpcHandler : public rclcpp::Node {
  public: MpcHandler(): Node("mpc_node"), total_execution_time(0)
    {
      path_subscriber_ = this->create_subscription<custom_msgs::msg::WaypointsMsg>("waypoints", 10, std::bind(&MpcHandler::mpc_callback, this, std::placeholders::_1));
      velocity_subscriber_ = this->create_subscription<custom_msgs::msg::VelocityToMpc>("velocity_to_mpc", 10, std::bind(&MpcHandler::velocity_callback, this, std::placeholders::_1));  
      publisher_ = this->create_publisher<custom_msgs::msg::MpcToCan>("mpc_to_can", 10);
      std::cout << "mpc node class created" << std::endl;
    }
  // const custom_msgs::msg::VelocityToMpc::SharedPtr vel_msg
  private:
    void velocity_callback(const custom_msgs::msg::VelocityToMpc::SharedPtr vel_msg){
      vel_struct.velocity_x = float(vel_msg->velocity_x);
      vel_struct.velocity_y = float(vel_msg->velocity_y);
      vel_struct.yaw_rate = float(vel_msg->yaw_rate);
      std::cout << "velocity_x from slam_from_file is: " << vel_struct.velocity_x << std::endl;
    }

    void mpc_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr path_msg) {
      //initialize interface
      std::cout << "I'm at mpc iteration " << global_int << std::endl;
      std::cout << "from velocity callback i received r: " << vel_struct.velocity_x << std::endl;
      // rclcpp::Time starting_time = this->now();
      int pp_points = int(path_msg->count);
      Eigen::MatrixXd spline_input{path_msg->count, 2 };
      if(pp_points==2) spline_input.resize(pp_points+1,2);
      int rows_temp=0;
      outputFile.open("src/6.Controls/mpc/data/pathplanner.csv",std::ios::out | std::ios::app); //log path planning AutoX data
      std::cout << "started writing to file" << std::endl;
      for (custom_msgs::msg::Point2Struct midpoints_new : path_msg->waypoints) {
        std::cout << "path planning points are: " << midpoints_new.x << " " << midpoints_new.y << std::endl;
        outputFile << midpoints_new.x << "," << midpoints_new.y << "\n";
        if((pp_points==2) && (rows_temp==1)) {
          spline_input(rows_temp,0) = (spline_input(0,0) + midpoints_new.x)/2;
          spline_input(rows_temp,1) = (spline_input(0,1) + midpoints_new.y)/2;
          std::cout << "path planning points are: " << spline_input(rows_temp,0) << " " << spline_input(rows_temp,1) << std::endl;
          rows_temp++;
          spline_input(rows_temp,0) = midpoints_new.x;
          spline_input(rows_temp,1) = midpoints_new.y;
          std::cout << "path planning points are: " << spline_input(rows_temp,0) << " " << spline_input(rows_temp,1) << std::endl;
          break;
        }
        spline_input(rows_temp,0) = midpoints_new.x;
        spline_input(rows_temp,1) = midpoints_new.y;
        rows_temp++;
      }
      outputFile.close();
      std::cout << "rows_temp counter is: " << rows_temp << std::endl;
      std::cout << "spline input size is: " << spline_input.rows() << " " << spline_input.cols() << std::endl;
      std::cout << "path planning counter is: " << int(path_msg->count) << std::endl;
      rclcpp::Time starting_time = this->now();
      path_planning::PointsArray midpoints{spline_input};
      // midpoints.conservativeResize(midpoints.rows()+1 , midpoints.cols());
      // midpoints.row(midpoints.rows() - 1) = midpoints.row(0);
      path_planning::ArcLengthSpline spline{midpoints, path_planning::BoundaryCondition::Anchored};
      std::ostringstream oss;
      params_array = spline.getSplineData(40);
      // for(int j =0; j<4; ++j){ 
      //   params_array(39,j)=params_array(38,j);
      // }
      rclcpp::Duration total_time = this->now() - starting_time ;
      oss  << "(" << params_array.rows() << ", " << params_array.cols() << ")";
      std::cout << "size of params array is " << oss.str() << std::endl;
      if(global_int==0){
          std::cout << "initialized all!!!" << std::endl;
          Initialize_all_local();
      }
        for (int j = 0; j < X_SIZE; ++j) params.xinit[j] = X[j]; //update init state
        writeParamsLocal();
        exitflag = FORCESNLPsolver_solve(&params, &output, &info, mem, NULL, extfunc_eval);
        if (exitflag != 1) {
            printf("\n\nFORCESNLPsolver did not return optimal solution at step %d. Exiting.\n", global_int);
            return_val = 1;
        }
        //dF ddelta dindex
        for(int k = 0; k<3; k++){
            U[k]=output.x02[k];
        }
        std::cout << "U array is: " << U[0] << " " << U[1] << " " << U[2] << std::endl;
        Integrator();
        std::cout << "X,Y,velocities are: " << X[0] << " " << X[1] << " " << X[3] << std::endl;
        std::cout << "inputs final are: " << (X[6]*Rw/(gr*eff)) << " " << (X[7]*sr*57.2958) << " " << X[8] << std::endl;
        // rclcpp::Duration total_time = this->now() - starting_time ;
        total_execution_time += total_time.nanoseconds() / 1000000.0;
        std::cout << "Time of spline Execution: "<<total_time.nanoseconds() / 1000000.0 << " ms." <<std::endl;
        std::cout << " " << std::endl;
        //define message to ROS2
        auto msg_final = custom_msgs::msg::MpcToCan();  
        msg_final.mt = (float)(X[6]*Rw/(gr*eff));
        msg_final.sa = (float) (X[7]);
        msg_final.bp = 0.0;
        RCLCPP_INFO(this->get_logger(), "Publishing motor torque: %.6f" " ,wheel angle: %.6f" " ,brake pressure: %.6f" , msg_final.mt, msg_final.sa, msg_final.bp);
        publisher_->publish(msg_final);
        global_int++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<custom_msgs::msg::WaypointsMsg>::SharedPtr path_subscriber_;
    rclcpp::Subscription<custom_msgs::msg::VelocityToMpc>::SharedPtr velocity_subscriber_;
    rclcpp::Publisher<custom_msgs::msg::MpcToCan>::SharedPtr publisher_;
    size_t count_;
    std::ofstream outputFile;
    double total_execution_time;
    int global_int=0;
};

} //namespace mpc

int main(int argc, char * argv[]) {  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcHandler>());
  rclcpp::shutdown();
  return 0;
}