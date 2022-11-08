import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from custom_msgs.msg import Trash

import serial
import yaml
import os

from dataclasses import dataclass
from typing import Callable

# num = 0b1001
# bool_list = [int(b) for b in bin(num)[2:]]

# for val in bool_list:
#     if val:
#         print(True)
#     else:
#         print(False)

@dataclass
class CanMsgStruct:
    sensor_name: str
    can_id: int
    topic_name: str
    msg_type: None      # this type changes with each new message type
    parser: Callable
    pub: Publisher = None

    def setPub(self, publisher_obj):
        self.pub = publisher_obj


class CommonUtilities:
    def __init__(self) -> None:
        self.msgs_lib = {'hall_msg': Twist, 'brake_pressure_msg': Twist, 'trash_msg': Trash}
        self.parser_lib = {'hall_parser': self.parse_hall_msg, 'steering_parser': self.parse_steering_msg, 'brakes_parser': self.parse_brakes_msg, 'parse_trash': self.parse_trash}
    
    def get_msg_type(self, msg_type:str):
        if msg_type not in self.msgs_lib.keys():
            raise KeyError(f"Message type {msg_type} not found in library. Please update.")
        else:
            return self.msgs_lib[msg_type]

    def get_parser(self, parser_func:str) -> Callable:
        if parser_func not in self.parser_lib:
            raise KeyError(f"Parser function {parser_func} not found in library. Please update.")
        else:
            return self.parser_lib[parser_func]

    def parse_hall_msg(self, parent, data:bytearray) -> None:
        pass

    def parse_steering_msg(self, parent, data:bytearray) -> None:
        pass

    def parse_brakes_msg(self, parent, data:bytearray) -> None:
        pass

    def parse_trash(self, parent, data:bytearray) -> None:
        print('Entered parser')
        val = int(data[0:4], base=16)
        msg = parent.msg_type()
        msg.trash = val
        parent.pub.publish(msg)
        


class CanListener(Node):
    def __init__(self) -> None:
        super().__init__("can_listener")
        self.set_params()
        # Port Configuration
        port = self.get_parameter('port').value
        b_rate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value       

        self.get_logger().info(f'Connected to {port} @ {b_rate} baud')
        self.get_logger().info(f'Timeout: {timeout}')
        
        enabled_sensors = self.get_parameter('enabled_sensors')
        if enabled_sensors.value == ['None']:
            self.get_logger().warn('No enabled sensors found. Check config files.')
        else:
            self.get_logger().info(f'Enabling Sensors: {enabled_sensors.value}')

        self.utils = CommonUtilities() 
        self.setup()

        # Open Serial Port
        self.ser = serial.Serial(port, baudrate=b_rate, timeout=timeout)
        self.run()

    def set_params(self) -> None:
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 10)
        self.declare_parameter('enabled_sensors', ['None'])
        this_dir = get_package_share_directory('can_reader')
        self.declare_parameter('sensor_config_file', os.path.join(this_dir, 'config', 'sensor_config.yaml'))

    def setup(self):
        config_file = self.get_parameter('sensor_config_file').value
        with open(config_file, 'r') as stream:
            try:
                config_dict = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                self.get_logger().error(exc)

        self.struct_array = []
        for sensor in self.get_parameter('enabled_sensors').value:
            if sensor not in config_dict.keys():
                raise KeyError(f"Sensor {sensor} has not been configured in sensor_config.yaml. Please update file.")
            vals_dict = config_dict[sensor]
            temp = CanMsgStruct(sensor, vals_dict['id'], vals_dict['topic'], self.utils.get_msg_type(vals_dict['msg_type']), self.utils.get_parser(vals_dict['parser']))
            temp.setPub(self.create_publisher(temp.msg_type, temp.topic_name, qos_profile_sensor_data))
            self.struct_array.append(temp)
        print(self.struct_array)

    def run(self):
        while True:
            msg = self.ser.readline()
            print(msg)
            print(type(msg))
            print(len(msg))
            msg_id = int(msg[0:4], base=16)
            print(msg_id)
            data = msg[6:len(msg)]
            for can in self.struct_array:
                print(can.can_id)
                if msg_id == can.can_id:
                    can.parser(can, data)


def main(args=None):
    rclpy.init(args=args)
    can_listener = CanListener()
    rclpy.spin(can_listener)
    
    can_listener.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()


# bytearr = '\x11\x02'.encode('UTF-8')
# num = 0
# print(num)