#include "lidar.h"

using namespace lidar_namespace;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto lidar_node = std::make_shared<LidarNode>();
    executor.add_node(lidar_node);
    executor.spin();
    rclcpp::shutdown();
    
    return 0;
    
}