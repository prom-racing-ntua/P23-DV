#include "velocity_estimation_handler.h"

// TODO: See if the whole node is thread safe...?
// TODO: General code optimization...

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    ns_vel_est::VelocityEstimationHandler velocity_node{};
    rclcpp::spin(velocity_node.get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}