# ROS Imports
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from ament_index_python.packages import get_package_share_directory

# ROS Message Imports
from custom_msgs.msg import BrakePressure, SteeringAngle, WheelSpeed, CanVehicleVariables, CanWheelSpeed

# Python Imports
import serial
import yaml
import os
import atexit
from dataclasses import dataclass
from typing import Callable, Any
from functools import partial


@dataclass
class IncomingCanMsg:
    '''
    Data class containing unique incoming messages with their ids, parser functions, ROS topics and publishers
    '''
    sensor_name: str
    can_id: int
    topic_name: str
    msg_type: Any
    parser: Callable
    pub: Publisher = None

    def setPub(self, publisher_obj) -> None:
        self.pub = publisher_obj


@dataclass
class OutgoingCanMsg:
    '''
    Data class containing unique outgoing messages with their ids, callback functions, ROS topics and subscribers
    '''
    message_name: str
    can_id: int
    topic_name: str
    msg_type: Any
    callback: Callable
    sub: Subscription = None

    def setSub(self, subscriber_obj) -> None:
        self.sub = subscriber_obj


class CommonUtilities:
    '''
    Contains all the parser/callback functions and message types and dictionaries to match them to config file string names.
    New functions and messages should be added to these dictionaries with their corresponding key for the config file.
    '''
    def __init__(self) -> None:
        # Dictionaries to match config file entries to functions and ros message types
        self.msgs_lib = {'hall_msg': WheelSpeed, 'brake_pressure_msg': BrakePressure, 'steering_msg': SteeringAngle, 'can_state': CanVehicleVariables,
                         'sim_halls': CanWheelSpeed}
        self.func_lib = {'hall_parser': self.parse_hall_msg, 'steering_parser': self.parse_steering_msg, 'brakes_parser': self.parse_brakes_msg,
                         'state_pub_callback': self.state_pub_callback, 'hall_callback': self.simulation_hall_callback}
    
    def get_msg_type(self, msg_type:str) -> Any:
        '''
        Returns the actual message class given in the dictionaries created within the object (see __init__)
        The message type is then used either to subscribe or to publish to a ros topic, depending whether the CanMsg is Incoming or Outgoing
        '''
        if msg_type not in self.msgs_lib.keys():
            raise KeyError(f"Message type {msg_type} not found in library. Please update.")
        else:
            return self.msgs_lib[msg_type]

    def get_func(self, func:str) -> Callable:
        '''
        Returns the actual callback function given in the dictionaries created within the object (see __init__)
        The function is then used to process the data coming or going to the Can-USB module (transform from/to serial port)
        '''
        if func not in self.func_lib:
            raise KeyError(f"Function {func} not found in library. Please update.")
        else:
            return self.func_lib[func]

    def makeXBytes(self, hex_val, num_bytes=2):
        '''Given a hex string add necessary digits so the length matches the number of bytes wanted'''
        size = num_bytes * 2
        hex_val = hex_val.lstrip('0x')
        s = len(hex_val)
        if s <= size:
            new_hex_val = '0'*(size - s) + hex_val
        elif s > size:
            raise OverflowError("Message overflowed!")
        return new_hex_val

    def encode_msg(self, msg):
        new_msg = ''
        for i in range(int(len(msg)/2)):
            new_msg += '\\x' + msg[2*i] + msg[2*i+1]
        new_msg = new_msg.encode('UTF-8')
        return new_msg

    def parse_hall_msg(self, parent, data:bytearray) -> None:
        msg = parent.msg_type()
        # We need bytes 4-5 and 6-7 -> hex characters 8-11 and 12-15 respectively
        msg.left_wheel = int.from_bytes(bytearray.fromhex(data[8:12]), byteorder='big', signed=True)
        msg.right_wheel = int.from_bytes(bytearray.fromhex(data[12:16]), byteorder='big', signed=True)
        msg.actual_speed = (msg.right_wheel + msg.left_wheel) / 2 / 9.5493 * 0.1995
        parent.pub.publish(msg)

    def parse_steering_msg(self, parent, data:bytearray) -> None:
        msg = parent.msg_type()
        # We need byte 4 -> hex characters 8-9
        msg.steering_angle = int.from_bytes(bytearray.fromhex(data[8:10]), byteorder='big', signed=True)
        parent.pub.publish(msg)

    def parse_brakes_msg(self, parent, data:bytearray) -> None:
        msg = parent.msg_type()
        # We need bytes 2 and 3 -> hex characters 4-5 and 6-7 respectively
        msg.front_cylinder = int.from_bytes(bytearray.fromhex(data[4:6]), byteorder='big', signed=True)
        msg.rear_cylinder = int.from_bytes(bytearray.fromhex(data[6:8]), byteorder='big', signed=True)
        parent.pub.publish(msg)

    def simulation_hall_callback(self, parent, data:bytearray) -> None:
        msg = parent.msg_type()
        msg.front_left  = int.from_bytes(bytearray.fromhex(data[0:4]), byteorder='big', signed=True)
        msg.front_right = int.from_bytes(bytearray.fromhex(data[4:8]), byteorder='big', signed=True)
        msg.rear_left   = int.from_bytes(bytearray.fromhex(data[8:12]), byteorder='big', signed=True)
        msg.rear_right  = int.from_bytes(bytearray.fromhex(data[12:16]), byteorder='big', signed=True)
        parent.pub.publish(msg)

    def state_pub_callback(self, msg, parent, ids) -> None:
        time_start = parent.get_clock().now()
        size = '06'
        lat_accel = msg.lat_accel
        long_accel = msg.long_accel
        yaw_rate = msg.yaw_rate
        can_id = self.makeXBytes(hex(ids), 1)
        lat_accel_hex = self.makeXBytes(hex(lat_accel), 2)
        long_accel_hex = self.makeXBytes(hex(long_accel), 2)
        yaw_rate_hex = self.makeXBytes(hex(yaw_rate), 2)
        can_msg = can_id + size + lat_accel_hex[2:4] + lat_accel_hex[0:2] + long_accel_hex[2:4] + long_accel_hex[0:2] \
            + yaw_rate_hex[2:4] + yaw_rate_hex[0:2]
        # parent.get_logger().info(f"Raw hex msg {can_msg}")
        can_msg = bytes.fromhex(can_msg).decode('ascii', errors='ignore').encode('utf-8')
        parent.get_logger().info(f"Sending message: {can_msg}")
        parent.ser.write(can_msg)
        # parent.get_logger().info(f"Time to process {(parent.get_clock().now() - time_start).nanoseconds / 10**6} ms")


class CanInterface(Node):
    '''
    ROS Node used for receiving/sending can msgs form/to the Can/USB module and passing them to ROS topics to be utilized by other nodes.

    The node can be configured thorough the corresponding params.yaml file.
    '''
    def __init__(self) -> None:
        super().__init__("can_interface")
        self.set_params()
        
        # Port Configuration
        port = self.get_parameter('port').value
        b_rate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value       

        self.get_logger().info(f'Connected to {port} @ {b_rate} baud')
        self.get_logger().info(f'Timeout: {timeout}')
        
        # Initialize utils instance and run setup
        self.utils = CommonUtilities() 
        self.setup()

        # Open Serial Port
        self.ser = serial.Serial(port, baudrate=b_rate, timeout=timeout)
        # Send heartbeat to CanUSB module
        self.ser.write(b'0000')

        # Timer to perform read actions from the Can/USB module
        self.read_timer = self.create_timer(1 / self.get_parameter('read_frequency').value, self.run)

        # Close port at exit
        atexit.register(self.ser.close)

    def set_params(self) -> None:
        '''
        Sets basic parameters of the node.
        '''
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('read_frequency', 10)
        self.declare_parameter('enabled_sensors', ['None'])
        self.declare_parameter('outgoing_messages', ['None'])
        this_dir = get_package_share_directory('can_reader')
        self.declare_parameter('canbus_config_file', os.path.join(this_dir, 'config', 'canbus_config.yaml'))

    def setup(self) -> None:
        '''
        Creates the CanMsg objects according to the enabled sensors and outgoing messages.

        The parameters of each object should be set in the canbus_config.yaml file.
        '''
        # Loads data from the config file
        config_file = self.get_parameter('canbus_config_file').value
        with open(config_file, 'r') as stream:
            try:
                config_dict = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                self.get_logger().error(exc)
        
        # Check enabled sensors and outgoing messages
        enabled_sensors = self.get_parameter('enabled_sensors')
        if enabled_sensors.value == ['None']:
            self.get_logger().warn('No enabled sensors found. Check config files.')
        else:
            self.get_logger().info(f'Enabling Sensors: {enabled_sensors.value}')
            self.incoming_msgs_array = []
            for sensor in self.get_parameter('enabled_sensors').value:
                if sensor not in config_dict.keys():
                    raise KeyError(f"Message {sensor} has not been configured in canbus_config.yaml. Please update file.")
                vals_dict = config_dict[sensor]
                
                # Initializes IncomingCanMsg objects based on the parameters loaded from the .yaml file
                temp = IncomingCanMsg(sensor, vals_dict['id'], vals_dict['topic'], self.utils.get_msg_type(vals_dict['msg_type']), self.utils.get_func(vals_dict['parser']))
                # Sets ros publisher to defined topic and message type
                temp.setPub(self.create_publisher(temp.msg_type, temp.topic_name, qos_profile_sensor_data))
                self.incoming_msgs_array.append(temp)

        out_msgs = self.get_parameter('outgoing_messages')
        if out_msgs.value == ['None']:
            self.get_logger().warn('No outgoing messages found. Check config files.')
        else:
            self.get_logger().info(f'Sending Messages: {out_msgs.value}')
            self.outgoing_msgs_array = []
            for msg in self.get_parameter('outgoing_messages').value:
                if msg not in config_dict.keys():
                    raise KeyError(f"Message {msg} has not been configured in canbus_config.yaml. Please update file.")
                vals_dict = config_dict[msg]

                # Initializes OutgoingCanMsg objects based on the parameters loaded from the .yaml file
                temp = OutgoingCanMsg(msg, vals_dict['id'], vals_dict['topic'], self.utils.get_msg_type(vals_dict['msg_type']), self.utils.get_func(vals_dict['callback']))
                # Sets ros subscriber to defined topic and message type
                temp.setSub(self.create_subscription(temp.msg_type, temp.topic_name, partial(temp.callback, parent=self, ids=temp.can_id), qos_profile_sensor_data))
                self.incoming_msgs_array.append(temp)
        # To output all the Can message objects (commented out)
        # print(self.incoming_msgs_array)
        # print(self.outgoing_msgs_array)


    def run(self) -> None:
        '''
        Checks for new messages finds their id and passes the data to the corresponding parser function
        '''
        self.time_rec = self.get_clock().now()
        msg = self.ser.readline()
        if self.ser.in_waiting > (11*self.get_parameter("read_frequency").value*0.2):
            self.ser.reset_input_buffer()
            self.get_logger().warn("Input buffer overflow. Flushing buffer.")
        if msg != b'':
            self.get_logger().info(f"Received message: {msg}")
            msg = msg.strip().decode('utf-8')
            # Can id is the first 2 bytes - 4 hex characters
            msg_id = int.from_bytes(bytearray.fromhex(msg[0:4]), byteorder='big', signed=False)
            
            # Data is the rest of the characters excluded the 5th - 6th which indicate the length of the message
            data = msg[6:len(msg)]
            # Check all IncomingCanMsgs for the message id
            for can in self.incoming_msgs_array:
                if msg_id == can.can_id:
                    can.parser(can, data)
                    # self.get_logger().info(f"Time to process msg: {(self.get_clock().now() - self.time_rec).nanoseconds / 10**6} ms")


def main(args=None) -> None:
    rclpy.init(args=args)
    can_interface = CanInterface()
    executor = SingleThreadedExecutor()

    try:
        rclpy.spin(can_interface, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        can_interface.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()