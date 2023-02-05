# System related imports
import os

import time
# ROS2 Related Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from custom_msgs.msg import AcquisitionMessage, Perception2Slam, VelEstimation
from ament_index_python.packages import get_package_share_directory

class FakeVelocityNode(Node):
    def __init__(self, filePath):
        super().__init__('fake_velocity_vlakaaa')

        self.fp = open(filePath)

        self.publisher_ = self.create_publisher(VelEstimation, 'state_estimation/vel_est', 10)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 1

    def timer_callback(self):
        msg = VelEstimation()
        msg.counter = int(self.fp.readline()) #line1

        msg.u_x = float(self.fp.readline()) #line2
        msg.u_y = float(self.fp.readline()) #line3
        msg.u_yaw = float(self.fp.readline()) #line4

        # msg.var_matrix = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]
        proxy = []
        for i in self.fp.readline().split(" "):
            proxy.append(float(i))

        msg.var_matrix = proxy
        self.i += 1
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Just published to slam, vlaka")

def main(args=None):
    rclpy.init(args=args)
    path = get_package_share_directory("fake_velocity_to_slam")
    data = os.path.join(path,"data")

    filePath = f"{data}/AutoX-3-Velocity.txt"

    time.sleep(1.5)
    fake_velocity_node = FakeVelocityNode(filePath=filePath)

    rclpy.spin(fake_velocity_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fake_velocity_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    