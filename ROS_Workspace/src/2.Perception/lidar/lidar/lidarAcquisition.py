import os
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from ament_index_python import get_package_share_directory

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class LidarAcquisition(Node):
    def __init__(self):
        super().__init__('lidar_acquisition')
        # share_dir = get_package_share_directory('lidar')
        # self.save_dir = os.path.join(share_dir, "../LidarData")

        self.pointCloudSub_ = self.create_subscription(PointCloud2, 'hesai/pandar', self.subscriberCallback, 10)

        self.get_logger().warn("Lidar Acquisition Initialized")

    def subscriberCallback(self, msg):
        # xyz = np.array([[0,0,0]])
        # rgb = np.array([[0,0,0]])
        #self.lock.acquire()
        self.get_logger().info("mpika subscriber")
        # gen = pc2.read_points(msg, skip_nans=True)
        # self.get_logger().info("diavasa gen")
        # int_data = list(gen)

        # for x in int_data:
        #     xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)

        # out_pcd = o3d.geometry.PointCloud()    
        # out_pcd.points = o3d.utility.Vector3dVector(xyz)

        # file_name = f"PointCloud_{int(self.get_clock().now().nanoseconds*1000)}.ply"
        # o3d.io.write_point_cloud(file_name, out_pcd)

def main(args=None):
    rclpy.init(args=args)

    node = LidarAcquisition()
    # executor = SingleThreadedExecutor()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    # try:
    #     rclpy.spin(node, executor)
    # except (KeyboardInterrupt, ExternalShutdownException):
    #     pass
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

if __name__ == "__main__":
    main()