# System related imports
import os
import time

# ROS2 Related Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from custom_msgs.msg import AcquisitionMessage, Perception2Slam
from ament_index_python.packages import get_package_share_directory

class FakePercNode(Node):
    def __init__(self, filePath):
        super().__init__('fake_perc_vlakaaa')

        self.fp = open(filePath)

        self.publisher_ = self.create_publisher(Perception2Slam, 'perception2slam_topic', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 1

    def timer_callback(self):
        msg = Perception2Slam()
        msg.global_index = int(self.fp.readline())
        
        classesList = []
        rangeList = []
        thetaList = []
        for classes,ranges,theta in zip((self.fp.readline().split(' ')),(self.fp.readline().split(' ')),(self.fp.readline().split(' '))):
            classesList.append(int(classes))
            rangeList.append(float(ranges))
            thetaList.append(float(theta))
        msg.class_list = classesList
        msg.range_list = rangeList
        msg.theta_list = thetaList
        self.publisher_.publish(msg)
        # self.get_logger().info(f"globalIndex: {msg.global_index}, classesList: {msg.class_list}, rangeList: {msg.range_list}, thetaList: {msg.theta_list}")
        # self.get_logger().info(f"Just published to slam, vlaka")

def main(args=None):
    rclpy.init(args=args)
    path = get_package_share_directory("fake_perc_to_slam")
    data = os.path.join(path,"data")
    filePath = f"{data}/VasilisPerception.txt"
    fake_perc_node = FakePercNode(filePath=filePath)

    time.sleep(3)
    rclpy.spin(fake_perc_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fake_perc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    