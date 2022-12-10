#include "velocity_estimation_handler.h"

// TODO: See if the whole node is thread safe...?
// TODO: General code optimization...

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ns_vel_est::VelocityEstimationHandler>());
    rclcpp::shutdown();
    return 0;
}