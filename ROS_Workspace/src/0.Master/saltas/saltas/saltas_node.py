# System Related Imports
import sys
import os
from pathlib import Path
import time

# ROS2 Related imports
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from std_msgs.msg import Header
from custom_msgs.msg import NodeSync
from custom_msgs.srv import GetFrequencies

class SaltasNode(Node):
    '''P23 Master Node (me to kalytero onoma)'''
    def __init__(self) -> None:
        super().__init__('saltas_node')
        self.global_index = 0
        self.send_index = 0
        self.configure()

        # Publisher for the synchronization topic
        self.clock_publisher = self.create_publisher(NodeSync, 'saltas_clock', 10)

        # Master clock configuration and configuration of each task publish frequency
        self.clock_frequency = calcClockFrequency(self.velocity_estimation_frequency, self.perception_frequency)
        self.send_velocity = int(self.clock_frequency / self.velocity_estimation_frequency)
        self.send_perception = int(self.clock_frequency / self.perception_frequency)
        self.send_reset = calcClockFrequency(self.send_velocity, self.send_perception)
        self.saltas_clock = self.create_timer(1/self.clock_frequency, self.globalTimerCallback)

        # Service for nodes to get their frequencies from the master
        self.frequency_service = self.create_service(GetFrequencies, 'get_frequencies', self.frequency_srv_callback)

        self.get_logger().info(f'Master Clock is Online:')
        self.get_logger().info(f'Velocity Estimation Frequency {self.velocity_estimation_frequency} Hz')
        self.get_logger().info(f'Perception Frequency {self.perception_frequency} Hz')

    def configure(self) -> None:
        '''Load all ros parameters from config files'''
        self.velocity_estimation_frequency = self.declare_parameter('velocity_estimation_frequency', 50).value
        self.perception_frequency = self.declare_parameter('perception_frequency', 10).value

    def globalTimerCallback(self) -> None:
        '''
        Used this algorithm with the send_index. The faster way would be to check the modulo of the global_index every time. 
        Didn't do that because it seems slower (global_index can reach very high values and maybe it slows down because of that??),
        but not 100% sure if that is true.
        '''
        if self.send_index >= self.send_reset:
            self.send_index = 0

        # Set node message
        msg = NodeSync()
        msg.global_index = self.global_index
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.exec_velocity = False
        msg.exec_perception = False
        if (self.send_index%self.send_velocity == 0):
            # Send velocity estimation execution msg
            msg.exec_velocity = True
        if (self.send_index%self.send_perception == 0):
            # Send perception execution msg
            msg.exec_perception = True
        self.clock_publisher.publish(msg)

        self.global_index += 1
        self.send_index += 1
    
    def frequency_srv_callback(self, request, response):
        '''Callback for the GetFrequencies service. Enters all the node frequencies into the response and returns it to the client.'''
        response.perception_frequency = self.perception_frequency
        response.velocity_estimation_frequency = self.velocity_estimation_frequency
        return response

def calcClockFrequency(velocity_freq, perception_freq):
    '''This function calculates the least common multiplier of the two node frequencies as the necessary frequency of the master clock.'''
    if velocity_freq > perception_freq:
        greater = velocity_freq
    else:
        greater = perception_freq
    while(True):
        if((greater % velocity_freq == 0) and (greater % perception_freq == 0)):
            clock_freq = greater
            break
        greater += 1
    return clock_freq


def main(args=None):
    rclpy.init(args=args)
    time.sleep(3)
    # Spin Master Node
    p23_master_node = SaltasNode()
    executor = SingleThreadedExecutor()
    try:
        rclpy.spin(p23_master_node, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        p23_master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()