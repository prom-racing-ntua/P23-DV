#include "can_reader.h"
#include "can_messages.h"

using namespace can_reader_namespace;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto can_reader_node = std::make_shared<CanReader>();
    executor.add_node(can_reader_node);
    executor.spin();
    rclcpp::shutdown();
    
    return 0;
    
}