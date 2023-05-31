from rclpy.node import Node
import rclpy

def main(args=None) -> None:
    rclpy.init(args=args)
    while (rclpy.ok()):
        node_names = rclpy.create_node('list')
        node_names.get_logger().info(f"{node_names.get_node_names()}")
    rclpy.shutdown()