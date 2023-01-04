# System Related Imports
import sys
import os
from pathlib import Path
import time

# ROS2 Related imports
import rclpy
from rclpy.node import Node

# P23 Master Node (me to kalytero onoma)
class SaltasNode(Node):
    def __init__(self):
            super().__init__('saltas_node')

def main(args=None):
    rclpy.init(args=args)

    # Spin Master Node
    p23_master_node = SaltasNode()
    rclpy.spin(p23_master_node)

    p23_master_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()