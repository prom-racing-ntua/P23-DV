import rclpy
from rclpy.node import Node
from custom_msgs.msg import HallSensors

import can
from .basic_classes import CANBusHandler

class CanReader(Node):
    def __init__(self):
        super().__init__("can_reader")   
        self.bus = can.ThreadSafeBus(channel = "can0", interface = "socketcan")
        self.publisher = self.create_publisher(HallSensors, "can_bus/hall", 10)
        
        thread = CANBusHandler(self)
        thread.start()
        

def main(args=None):
    rclpy.init(args=args)
    can_reader = CanReader()
    rclpy.spin(can_reader)
    
    can_reader.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()