import rclpy
from rclpy.node import Node
from custom_msgs.msg import CanVehicleVariables


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(CanVehicleVariables, 'state_pub', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = CanVehicleVariables()
        msg.lat_accel = 25922
        msg.long_accel = 29811
        msg.yaw_rate = 22084
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    try:
        rclpy.spin(minimal_publisher)
    except (KeyboardInterrupt):
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()