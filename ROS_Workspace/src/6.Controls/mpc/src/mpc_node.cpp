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
      subscription_ = this->create_subscription<custom_msgs::msg::WaypointsMsg>("waypoints", 10, std::bind(&MpcHandler::mpc_callback, this, std::placeholders::_1));  
      publisher_ = this->create_publisher<custom_msgs::msg::MpcToCan>("mpc_to_can", 10);
    }

  private:
    void mpc_callback(const custom_msgs::msg::WaypointsMsg::SharedPtr msg) {
      //initialize interface
      std::cout << "I'm at mpc iteration " << global_int << std::endl;
      rclcpp::Time starting_time = this->now();
      Eigen::MatrixXd spline_input{ msg->count, 2 };
      int rows_temp=0;
      for (custom_msgs::msg::Point2Struct midpoints_new : msg->waypoints) {
        std::cout << "path planning points are: " << midpoints_new.x << " " << midpoints_new.y << std::endl;
        spline_input(rows_temp,0) = midpoints_new.x;
        spline_input(rows_temp,1) = midpoints_new.y;
        rows_temp++;
      }
      std::cout << "path planning counter is: " << int(msg->count) << std::endl;
      path_planning::PointsArray midpoints{spline_input};
      // midpoints.conservativeResize(midpoints.rows()+1 , midpoints.cols());
      // midpoints.row(midpoints.rows() - 1) = midpoints.row(0);
      path_planning::ArcLengthSpline spline{midpoints, path_planning::BoundaryCondition::Anchored};
      std::ostringstream oss;
      params_array = spline.getSplineData(40);
      for(int j =0; j<4; ++j){ 
        params_array(39,j)=params_array(38,j);
      }
      oss  << "(" << params_array.rows() << ", " << params_array.cols() << ")";
      std::cout << "size of params array is " << oss.str() << std::endl;
      if(global_int==0){
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
        rclcpp::Duration total_time = this->now() - starting_time ;
        total_execution_time += total_time.nanoseconds() / 1000000.0;
        std::cout << "Time of Execution: "<<total_time.nanoseconds() / 1000000.0 << " ms." <<std::endl;
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
    rclcpp::Subscription<custom_msgs::msg::WaypointsMsg>::SharedPtr subscription_;
    rclcpp::Publisher<custom_msgs::msg::MpcToCan>::SharedPtr publisher_;
    size_t count_;
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