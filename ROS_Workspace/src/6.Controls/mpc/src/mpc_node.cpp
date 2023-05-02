#include "mpc_node.h"

using namespace mpc_cpp;
using namespace std::chrono_literals;

namespace mpc_cpp {
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<custom_msgs::msg::MpcToCan>("mpc_to_can", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback() {
      //initialize interface
      rclcpp::Time starting_time = this->now();
      std::cout << "Im at iteration " << global_int+1 << std::endl;
        if(global_int==0){
            readData();  //various needed initalizations before first run
            Initialize_all();
        }
        //define message to ROS2
        auto msg_final = custom_msgs::msg::MpcToCan();  
        msg_final.mt = (float)(X[6]*Rw/gr);
        msg_final.sa = (float) (X[7]/sr);
        msg_final.bp = 0.0;
        RCLCPP_INFO(this->get_logger(), "Publishing motor torque: %.6f" " ,steering angle:%.6f" " ,brake pressure:%.6f" , msg_final.mt, msg_final.sa, msg_final.bp);
        publisher_->publish(msg_final);
        for (int j = 0; j < X_SIZE; ++j) params.xinit[j] = X[j]; //update init state
        writeParams();
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
        std::cout << "inputs final are: " << (X[6]*Rw/gr) << " " << (X[7]*sr*57.2958) << " " << X[8] << std::endl;
        rclcpp::Duration total_time = this->now() - starting_time ;
        total_execution_time += total_time.nanoseconds() / 1000000.0;
        std::cout << "Time of Execution: "<<total_time.nanoseconds() / 1000000.0 << " ms." <<std::endl;
        std::cout << " " << std::endl;
        global_int++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_msgs::msg::MpcToCan>::SharedPtr publisher_;
    size_t count_;
    double total_execution_time;
    int global_int=0;
};

} //namespace mpc

int main(int argc, char * argv[]) {  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}