import sys
import os
import numpy as np
from collections import defaultdict, namedtuple
from typing import NamedTuple
from math import atan2, pi, sqrt

#ROS2 Imports
import rclpy
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node
from rclpy.executors import  SingleThreadedExecutor, ExternalShutdownException
from sensor_msgs_py import point_cloud2


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.dec_parameters()
        self.horizontal_resolution = self.get_parameter("hor_res").value
        self.rj_min = self.get_parameter("rj_min").value
        self.rj_max = self.get_parameter("rj_max").value
        self.da = self.horizontal_resolution*2*pi/360 #deg -> rad
        self.num_of_segments = 2*pi/self.da
        field_names = ['x','y','z', 'ring']
        self.Point = NamedTuple('Point', field_names)
        self.prototype_points = defaultdict(lambda: defaultdict(self.Point))
        print("num of segments " + str(self.num_of_segments))
        print("da" + str(self.da))
        self.segments = defaultdict(lambda: defaultdict(list)) #each segment has a dict of bins. eg the segment 
        self.lowest_z_of_segment = defaultdict(lambda: [100, 0])

        self.get_logger().info(f"Initializing Lidar Node")

        self.pointCloudSub_ = self.create_subscription(PointCloud2, '/hesai/pandar', self.subscriber_callback, 10)

    def dec_parameters(self):
        self.declare_parameter("hor_res", 0.18)
        self.declare_parameter("rj_min", 1)
        self.declare_parameter("rj_max", 2)
    
    def subscriber_callback(self, msg):
        dataList= point_cloud2.read_points_list(msg, field_names=['x','y','z', 'ring'])
        self.segments.clear()
        print(type(dataList[0]))
        print(str(len(dataList)))
        for point in dataList:
           dist = sqrt(point.x**2 + point.y**2)
           bin_ = dist//1
           seg = round(atan2(point.y, point.x)/self.da)
           (self.segments[seg])[bin_].append([point, point.z, dist])
           if ((self.prototype_points[seg])[bin_]).z > point.z:
               ((self.prototype_points[seg])[bin_]) = self.Point._make(point)
               
               

        print("len of segments " + str(len(self.segments)))
        total_points = 0
        for segment in self.segments:
            for bin in self.segments[segment]:
                total_points += len(self.segments[segment][bin])
        print("total points in segmented cloud " + str(total_points))
        print(self.prototype_points)


def main(args=None):
    rclpy.init(args=args)

    lidar_node = LidarNode()
    executor = SingleThreadedExecutor()

    try:
        rclpy.spin(lidar_node, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        lidar_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
