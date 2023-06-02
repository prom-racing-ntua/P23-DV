#include "mpc_from_file.h"

using namespace mpc_cpp;
using namespace path_planning;
using namespace std::chrono_literals;

namespace mpc_cpp {
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MpcFromFile : public rclcpp::Node {
  public:
    MpcFromFile(): Node("mpc_from_file"), count_(0)
    {
      publisher_ = this->create_publisher<custom_msgs::msg::MpcToCan>("mpc_to_can", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MpcFromFile::timer_callback, this));
    }

  private:
    void timer_callback() {
      //initialize interface
      rclcpp::Time starting_time = this->now();
      std::cout << "Im at iteration " << global_int+1 << std::endl;
      path_planning::PointsArray midpoints{ readTrack("src/6.Controls/mpc/data/trackdrive_midpoints.txt")};
      std::cout << "Read points" << std::endl;
      midpoints.conservativeResize(midpoints.rows() + 1, midpoints.cols());
      midpoints.row(midpoints.rows() - 1) = midpoints.row(0);
      path_planning::ArcLengthSpline spline{midpoints, path_planning::BoundaryCondition::ClosedLoop};
      std::cout << "At s = 0.67" << '\n';
      std::cout << "Spline coordinates: " << spline.getPoint(0.67) << '\n';
      std::cout << "Spline curvature: " << spline.getCurvature(0.67) << '\n';
      std::cout << "Spline tangent: " << spline.getTangent(0.67) << "\n\n";
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
            U[k]=output.x01[k];
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
  rclcpp::spin(std::make_shared<MpcFromFile>());
  rclcpp::shutdown();
  return 0;
}