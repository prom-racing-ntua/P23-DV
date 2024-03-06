import sys
import numpy as np
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from custom_msgs.msg import *
from math import pi


### --------------------------- Helper Functions --------------------------- ###

# Break Pressure Target for service brake [bar]
# SERVICE_BRAKE_TARGET = 30

# These are specific to P23 steering geometry. Positive displacement and wheel angle is to the right of the car as the positive x-axis.
RACK_DISPLACEMENT_LOOKUP = np.array([
# Steer Travel [mm] # Avg Wheel Angle [deg]
    [ 24.0,       (23.96173 + 27.01392) / 2],
    [ 19.2,       (19.15972 + 20.96173) / 2],
    [ 14.4,       (14.39368 + 15.3505) / 2 ],
    [  9.6,       (9.633149 + 10.04216) / 2],
    [  4.8,       (4.846694 + 4.946667) / 2],
    [  0.0,        0.0                     ],
    [ -4.8,       (-4.946667 -4.846694) / 2],
    [ -9.6,       (-10.04216 -9.633149) / 2],
    [-14.4,       (-15.3505 -14.39368) / 2 ],
    [-19.2,       (-20.96173 -19.15972) / 2],
    [-24.0,       (-27.01392 -23.96173) / 2]
])

def floatToBytes(num:float, byteorder:str='big', multiplier:int=256, signed=False, num_bytes=2) -> bytes:
    '''
    Transforms a floating point value to a byte array of *num_bytes*. The value is first multiplied by a standard number in order
    keep its decimal accuracy and then converted into an integer value which is then converted to bytes.
    
    :param num: The floating point value to be transformed into bytes
    :param byteorder: If the bytes should be in 'big; endian or 'little' endian order
    :param multiplier: The number the floating point value is multiplied by
    :param signed: If the input value is signed or not
    :param num_bytes: The number of bytes that should be returned

    :returns: The input value in bytes
    '''
    i_num = int(num * multiplier)
    if signed:
        np.clip(i_num, - 2**(8*num_bytes-1), 2**(8*num_bytes-1)-1)
    else:
        np.clip(i_num, 0, 2**(8*num_bytes)-1)
    return i_num.to_bytes(num_bytes, byteorder, signed=signed)

def bitsToByte(bit_list:list) -> bytes:
    '''
    Given a list of 8 bits with true or false values (0s and 1s) returns the same bit values in a python byte object.

    :param bit_list: The list of bit values to be transformed into the byte object
    :raises ValueError: Input size not equal to 8
    :returns: A single byte consisting of the input bits
    '''
    if len(bit_list) != 8:
        raise ValueError("bitsToByte() -> Input not equal to 8 bits, cannot convert to byte")
    bits = iter(bit_list)
    out = 0
    for _ in range(8):
        try: 
            bit = next(bits)
        except StopIteration:
            return out
        out = (out << 1) | bit
    return out

def get_bit(value: int, bit: int) -> bool:
    '''
    Gets a specific bit from a byte.

    :param value: The byte in integer value from which the bit is to be extracted
    :param bit: The number of the bit that is needed

    :returns: The value of the requested bit
    '''
    return bool((value >> bit) & 1)


def rad2rackDisplacement(steering_angle: float) -> float:
    '''
    Transforms the wheel angle from rad to the rack displacement in mm. Note that values are for p23 steering assembly and kinematics.
    The method does not take into account Ackermann steering.

    :param steering_angle: The wheel angle in rad
    :raises ValueError: Angle out of range due to steering geometry

    :returns: The rack displacement
    '''
    # Input should be in rad, so transform to deg
    steering_angle = steering_angle * 180/pi
    # Check if input is in range of the possible values
    if abs(steering_angle) > 24.0: raise ValueError(f"The steering angle given is out of range: {steering_angle} deg")

    for i in range(RACK_DISPLACEMENT_LOOKUP.shape[0]):
        # Current angle is bigger
        if steering_angle < RACK_DISPLACEMENT_LOOKUP[i,1]: continue

        # Found a smaller angle, do linear interpolation
        rack_displacement = RACK_DISPLACEMENT_LOOKUP[i-1,0] + (steering_angle - RACK_DISPLACEMENT_LOOKUP[i-1,1]) * (RACK_DISPLACEMENT_LOOKUP[i,0] - RACK_DISPLACEMENT_LOOKUP[i-1,0]) / (RACK_DISPLACEMENT_LOOKUP[i,1] - RACK_DISPLACEMENT_LOOKUP[i-1,1])
        return float(rack_displacement)

def rackDisplacement2rad(displacement: float) -> float:
    '''
    Transforms the steering rack displacement from mm rad to the wheel angle in rad. Note that values are for p23 steering assembly and kinematics.
    The method does not take into account Ackermann steering.

    :param displacement: The steering rack displacement in mm
    :raises ValueError: Displacement out of range due to steering geometry

    :returns: The average wheel angle
    '''
    # Check if input is in range of the possible values
    if abs(displacement) > 24.0: raise ValueError(f"The rack displacement given is out of range: {displacement} mm")

    for i in range(RACK_DISPLACEMENT_LOOKUP.shape[0]):
        # Current angle is bigger
        if displacement < RACK_DISPLACEMENT_LOOKUP[i,0]: continue

        # Found a smaller angle, do linear interpolation
        wheel_angle = RACK_DISPLACEMENT_LOOKUP[i-1,1] + (displacement - RACK_DISPLACEMENT_LOOKUP[i-1,0]) * (RACK_DISPLACEMENT_LOOKUP[i,1] - RACK_DISPLACEMENT_LOOKUP[i-1,1]) / (RACK_DISPLACEMENT_LOOKUP[i,0] - RACK_DISPLACEMENT_LOOKUP[i-1,0])

        # Output should be in rad, so transform output angle
        wheel_angle = wheel_angle * pi/180 
        return float(wheel_angle)


### --------------------------- Base Message Class --------------------------- ###

class CanInterfaceMessage():
    '''
    The base class for every message of the CanBus to/from ROS Interface

    :attr node_handle: The parent ROS2 node this object is part of
    :attr can_id: The canbus identifier of the message this object represents
    :attr byte_size: The size of the canbus message NOT including can id or DLC bytes
    :attr msg_type: The ROS2 message type this message corresponds to
    :attr ros_publisher: The ROS2 publisher object to which the received message from the can2usb module is published to
    :attr ros_subscriber: The ROS2 subscriber object from which dv system messages are received

    :param received_message: The message received either from the ros_subscriber or the serial port. Is respectively bytes or bytearray 
    containing the can id and DLC bytes or the ROS2 msg_type specified by the class attribute.
    '''
    # Node Handle for the CanInterface Node
    node_handle : Node

    # Can Message Variables
    can_id :int = 0
    byte_size :int = 0

    # ROS Message Variables
    msg_type : type = type(None)
    ros_publisher : Publisher = None
    ros_subscriber : Subscription = None

    def __init__(self, received_message) -> None:
        '''
        Class constructor, identifies if message is from ROS2 or can2usb module
        '''
        # Check if the given message is from CanBus or from ROS
        if (isinstance(received_message, (bytes, bytearray))):
            # If it is from CanBus process it and send it to ROS
            
            if len(received_message) != self.byte_size + 3:
                raise ValueError(f"Received message has invalid size {self}")
            received_id = int.from_bytes(received_message[0:2], byteorder='big', signed=False)
            received_byte_size = int.from_bytes(received_message[2:3], byteorder='big', signed=False)

            # Checks that the received message is valid
            if received_id != self.can_id: raise ValueError(f'Received message id does not match the default value: {received_id} != {self.can_id}')
            if received_byte_size != self.byte_size: raise ValueError(f'Received message id does not match the default value {received_byte_size} != {self.byte_size}')

            self._can_msg = received_message[3:]
            return

        elif (isinstance(received_message, self.msg_type)):
            # If it is from ROS encoded into bytes and transmit to CanBus
            self._ros_msg = received_message
            return

        else:
            raise TypeError(f"The given message is of unknown type {type(received_message)}")

    def to_ROS(self) -> msg_type:
        '''
        Transforms the input message from a canbus style message to a ROS2 message

        :raises NotImplementedError: Inside the base class, functionality not implemented yet

        :returns: The input message data in a ROS2 message 
        '''
        raise NotImplementedError("To ROS parser has not been defined in child class")

    def to_CanMsg(self) -> bytearray:
        '''
        Transforms the input message from a ROS2 message to a canbus style message
        
        :raises NotImplementedError: Inside the base class, functionality not implemented yet

        :returns: The data of the input message to a canbus message as a bytearray
        '''
        raise NotImplementedError("To CanBus parser has not been defined in child class")



### --------------------------- P23 CanBus Messages --------------------------- ###

class ActuatorCommandsMsg(CanInterfaceMessage):
    can_id = 0x01
    byte_size = 8
    msg_type = TxControlCommand

    def data(self) -> tuple:
        return [float(self._ros_msg.steering_angle_target),
                float(self._ros_msg.motor_torque_target),
                float(self._ros_msg.brake_pressure_target),
                float(self._ros_msg.speed_actual),
                float(self._ros_msg.speed_target)]

    def to_CanMsg(self) -> bytearray:
        out_msg = bytearray(self.byte_size)
        out_msg[0] = self.can_id

        if self.node_handle._shuting_down:
            self._ros_msg.steering_angle_target = 0.0
            self._ros_msg.motor_torque_target = 0.0
            self._ros_msg.brake_pressure_target = 0.0
            self._ros_msg.speed_actual = 0
            self._ros_msg.speed_target = 0

        # Steering Limiter
        try:
            temp = rad2rackDisplacement(self._ros_msg.steering_angle_target)
        except ValueError:
            temp = 23.5 * self._ros_msg.steering_angle_target / abs(self._ros_msg.steering_angle_target)

        out_msg[1:3] = floatToBytes(temp, multiplier=1024, signed=True)
        # out_msg[1:3] = floatToBytes(self._ros_msg.steering_angle_target, multiplier=1024, signed=True)

        if not self._ros_msg.motor_control:
            multiplier = 1
        else:
            multiplier = 128
        out_msg[3:5] = floatToBytes(self._ros_msg.motor_torque_target, multiplier=multiplier,signed=True)
        out_msg[5] = int(np.clip(self._ros_msg.brake_pressure_target, 0, 2**7-1))
        # floatToBytes(self._ros_msg.brake_pressure_target,multiplier=2,num_bytes=1, signed=True)
        # else: out_msg[5] = 0

        # Speed Actual and speed target
        out_msg[6] = np.uint8(np.clip(self._ros_msg.speed_actual*3.6, 0, 2**8-1))
        out_msg[6] = out_msg[6] | (self._ros_msg.motor_control << 7)
        out_msg[7] = np.uint8(np.clip(self._ros_msg.speed_target*3.6, 0, 2**8-1))
        return out_msg


class KinematicVariablesMsg(CanInterfaceMessage):
    can_id = 0x02
    byte_size = 7
    msg_type = VelEstimation

    def data(self):
        return [self._ros_msg.velocity_x,
                self._ros_msg.velocity_y,
                self._ros_msg.yaw_rate,
                self._ros_msg.acceleration_x,
                self._ros_msg.acceleration_y,]

    def to_CanMsg(self) -> bytearray:
        out_msg = bytearray(self.byte_size)
        out_msg[0] = self.can_id

        out_msg[1:3] = floatToBytes(self._ros_msg.acceleration_x, multiplier=512, signed=True)
        out_msg[3:5] = floatToBytes(self._ros_msg.acceleration_y, multiplier=512, signed=True)
        out_msg[5:7] = floatToBytes(self._ros_msg.yaw_rate*180/pi, multiplier=128, signed=True)
        return out_msg


class SystemHealthMsg(CanInterfaceMessage):
    can_id = 0x03
    byte_size = 8
    msg_type = TxSystemState

    def data(self):
        return [
            self._ros_msg.dv_status.id,
            self._ros_msg.lap_counter << 4,
            self.sensor_status,
            self.node_status,
            self.pc_temp
        ]

    def to_CanMsg(self) -> bytearray:
        out_msg = bytearray(self.byte_size)
        out_msg[0] = self.can_id

        # Set lap counter and dv-state flags
        dv_state = self._ros_msg.dv_status.id
        if dv_state == DriverlessStatus.LV_ON or dv_state == DriverlessStatus.STARTUP:
            state = 0x00
        elif dv_state == DriverlessStatus.MISSION_SELECTED:
            state = 0x01
        elif dv_state == DriverlessStatus.DV_READY:
            state = 0x02
        elif dv_state == DriverlessStatus.DV_DRIVING:
            state = 0x04
        elif dv_state == DriverlessStatus.MISSION_FINISHED:
            state = 0x08
        elif dv_state == DriverlessStatus.NODE_PROBLEM:
            if self.node_handle._shuting_down:
                state = 0x08
            else:
                state = 0x0F
        else:
            self.node_handle.get_logger().error(f"Unknown DV State received: {dv_state}")
            state = 0x0F
        
        if state == 0x0F: self.node_handle.get_logger().error("Received Node Error. Entering AS Emergency!")
        try:
            out_msg[1] = (self._ros_msg.lap_counter << 4) | state
        except Exception as e:
            self.node_handle.get_logger().warn(f"Error setting lap count: {repr(e)}. msg = {(self._ros_msg.lap_counter << 4) | state}. lap = {self._ros_msg.lap_counter}")
            raise ValueError(f"Error setting lap count: {repr(e)}. msg = {(self._ros_msg.lap_counter << 4) | state}. lap = {self._ros_msg.lap_counter}")

        out_msg[2] = self._ros_msg.cones_count_actual
        out_msg[3:5] = self._ros_msg.cones_count_all.to_bytes(2, byteorder='big')

        # Set sensor status byte
        sensor_status_bits = [0,0,0,0,0,0,0,0]
        sensor_status_bits[0] = int(self._ros_msg.vn_200_error)
        sensor_status_bits[1] = int(self._ros_msg.vn_300_error)
        sensor_status_bits[2] = int(self._ros_msg.camera_right_error)
        sensor_status_bits[3] = int(self._ros_msg.camera_left_error)
        sensor_status_bits[4] = int(self._ros_msg.ins_mode & 2)
        sensor_status_bits[5] = int(self._ros_msg.ins_mode & 1)
        out_msg[5] = bitsToByte(sensor_status_bits)
        self.sensor_status = ''.join([str(int(x)) for x in sensor_status_bits])

        # Set node status byte
        node_status_bits = [0,0,0,0,0,0,0,0]
        node_status_bits[0] = self._ros_msg.clock_error
        node_status_bits[1] = self._ros_msg.camera_inference_error
        node_status_bits[2] = self._ros_msg.velocity_estimation_error
        node_status_bits[3] = self._ros_msg.slam_error
        node_status_bits[4] = self._ros_msg.mpc_controls_error
        node_status_bits[5] = self._ros_msg.path_planning_error
        node_status_bits[6] = self._ros_msg.pi_pp_controls_error
        out_msg[6] = bitsToByte(node_status_bits)
        self.node_status = ''.join([str(int(x)) for x in node_status_bits])

        # Get the CPU temperature reading and set the corresponding byte
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as file:
                pc_temp = file.read().strip()
                out_msg[7] = int(int(pc_temp)/1000)
        except Exception:
            self.node_handle.get_logger().error(f"{Exception}")
            out_msg[7] = 0
        self.pc_temp = out_msg[7]
        return out_msg
    
#xx    
class SteeringParamsMsg(CanInterfaceMessage):
    can_id = 0x06
    byte_size = 13
    msg_type = TxSteeringParams

    def data(self) -> tuple:
        return [float(self._ros_msg.kd),
                float(self._ros_msg.kp),
                float(self._ros_msg.ki),
                float(self._ros_msg.dt),
                float(self._ros_msg.maxvel),
                float(self._ros_msg.minvel)]

    def to_CanMsg(self) -> bytearray:
        out_msg = bytearray(self.byte_size)
        out_msg[0] = self.can_id

        if self.node_handle._shuting_down:
            self._ros_msg.kd = 0.0
            self._ros_msg.kp = 0.0
            self._ros_msg.ki = 0.0
            self._ros_msg.dt = 0.0
            self._ros_msg.minvel = 0.0
            self._ros_msg.maxvel = 0.0

        #send steering params
        out_msg[1:3] = floatToBytes(self._ros_msg.kp, multiplier=1, signed=True)
        out_msg[3:5] = floatToBytes(self._ros_msg.kd, multiplier=1, signed=True)
        out_msg[5:7] = floatToBytes(self._ros_msg.ki, multiplier=1, signed=True)
        out_msg[7:9] = floatToBytes(self._ros_msg.dt, multiplier=512, signed=True)
        out_msg[9:11] = floatToBytes(self._ros_msg.minvel, multiplier=1, signed=True)
        out_msg[11:13] = floatToBytes(self._ros_msg.maxvel, multiplier=1, signed=True)
        
        return out_msg
#xx


class AsStatusMsg(CanInterfaceMessage):
    can_id = 0x306
    byte_size = 1
    msg_type = AutonomousStatus

    def to_ROS(self) -> msg_type:
        ros_msg = self.msg_type()

        # Set the AS-Status
        received_status = int.from_bytes(self._can_msg[0:1], byteorder='big')
        if received_status == 0x01: 
            ros_msg.id = AutonomousStatus.AS_OFF
            ros_msg.label = "AS_OFF"
        elif received_status == 0x02:
            ros_msg.id = AutonomousStatus.AS_READY
            ros_msg.label = "AS_READY"
        elif received_status == 0x04:
            ros_msg.id = AutonomousStatus.AS_DRIVING
            ros_msg.label = "AS_DRIVING"
        elif received_status == 0x08:
            ros_msg.id = AutonomousStatus.AS_FINISHED
            ros_msg.label = "AS_FINISHED"
            self.node_handle._shuting_down = True
        elif received_status == 0x10:
            ros_msg.id = AutonomousStatus.AS_EMERGENCY
            ros_msg.label = "AS_EMERGENCY"
            self.node_handle._shuting_down = True
        else:
            self.node_handle.get_logger().error(f"Invalid AS Status received: {received_status}")
            ros_msg.id = 0
        return ros_msg

class MissionMsg(CanInterfaceMessage):
    can_id = 0x304
    byte_size = 1
    msg_type = MissionSelection

    LOCKED = 7

    # Handles the mission handshake and locking
    def handle_mission(self) -> bool:
        self.node_handle.get_logger().error("Started decoding ROS Mission Msg")
        mission_confirmed = False
        mission_byte = int.from_bytes(self._can_msg, byteorder='big')
        # self.node_handle.get_logger().info(f"{mission_byte}")
        bits = []
        for bit in range(8):
            bits.append(get_bit(mission_byte, bit))
        # self.node_handle.get_logger().info(f"{bits}")
        is_locked = bits[self.LOCKED]
        ones = bits.count(True) - is_locked
        
        if ones > 1:
            # This should not happen ever
            self.node_handle.get_logger().error("Received Multiple Missions")
            raise ValueError("Received Multiple Missions")

        elif ones == 0:
            if self.node_handle.get_clock().now().nanoseconds / 10**9 - self.node_handle.time_of_lock < 0.5: #0.5s
                self.node_handle.ignore_unlock = True
            else:    
                # Unlock mission
                self.node_handle.get_logger().warn("Mission Unlocked")
                mission_confirmed = True
                self.node_handle._locked_mission = 0
                self.node_handle._received_mission = None
            
        else:
            # Received valid mission
            mission = bits.index(1) + 1
            assert mission != self.LOCKED + 1, "Mission Locked but non selected?!"

            if is_locked and self.node_handle._received_mission == mission:
                # Received mission a second time with the locked flag
                mission_confirmed = True
                self.node_handle._locked_mission = mission
                self.node_handle.get_logger().warn(f"Mission Confirmed {hex(self.node_handle._locked_mission)}")
                self.node_handle.time_of_lock = self.node_handle.get_clock().now().nanoseconds / 10**9
            
            else:
                # Received mission the first time
                self.node_handle._received_mission = mission
            
        # Send back received message for acknowledgment
        self.node_handle.get_logger().warn("Sent aknowledgement.")
        out_msg = bytearray(2)
        out_msg[0] = 0x04
        out_msg[1] = mission_byte
        self.node_handle._serial_port.write(out_msg)
        return mission_confirmed

    def to_ROS(self) -> msg_type:
        ros_msg = self.msg_type()
        ros_msg.mission_selected = self.node_handle._locked_mission
        return ros_msg


class SensorVariablesMsg(CanInterfaceMessage): 
    can_id = 0x300
    byte_size = 8
    msg_type = RxVehicleSensors

    def data(self):
        return self._data

    def to_ROS(self) -> msg_type: #xx
        msg = self.msg_type()
        
        # Set Motor Torque
        motor_torque = int.from_bytes(self._can_msg[2:4], byteorder='big', signed=True)
        msg.motor_torque_actual = motor_torque/100 
        
        #Set Motor RPM
        motor_rpm = int.from_bytes(self._can_msg[4:6], byteorder='big', signed=True)
        msg.motor_rpm = motor_rpm

        # Set Brake Hydraulic Pressure
        msg.brake_pressure_front = float(int.from_bytes(self._can_msg[0:1], byteorder='big', signed=False))
        msg.brake_pressure_rear = float(int.from_bytes(self._can_msg[1:2], byteorder='big', signed=False))

        # Set linear voltage 
        linear_voltage_temp = (float(int.from_bytes(self._can_msg[6:8],byteorder='big', signed=False)))/1024

        self._data = [msg.motor_torque_actual, motor_rpm * 0.2 / (9.5493*3.9), msg.brake_pressure_front, msg.brake_pressure_rear, linear_voltage_temp]

        return msg


class WheelEncodersMsg(CanInterfaceMessage):
    can_id = 0x301
    byte_size = 8
    msg_type = RxWheelSpeed

    def data(self):
        return self._data
    def to_ROS(self) -> msg_type:
        msg = self.msg_type()

        msg.front_left = int.from_bytes(self._can_msg[0:2], byteorder='big', signed=False)
        msg.front_right = int.from_bytes(self._can_msg[2:4], byteorder='big', signed=False)
        msg.rear_left = int.from_bytes(self._can_msg[4:6], byteorder='big', signed=False)
        msg.rear_right = int.from_bytes(self._can_msg[6:8], byteorder='big', signed=False)

        self._data = [msg.front_left, msg.front_right, msg.rear_left, msg.rear_right]

        return msg


class SteeringAngleMsg(CanInterfaceMessage):
    exitflag=1
    can_id = 0x05
    byte_size = 8
    msg_type = RxSteeringAngle

    def data(self):
        return self._data
    def to_ROS(self) -> msg_type: 
        msg = self.msg_type()
        temp = int.from_bytes(self._can_msg[0:2], byteorder='little', signed=True) / 1024
        try:
            msg.steering_angle = rackDisplacement2rad(temp)
        except ValueError:
            msg.steering_angle = rackDisplacement2rad(24 * temp/abs(temp))
        linear_voltage = float(int.from_bytes(self._can_msg[2:4],byteorder='big',signed=False)/1024)
        requested_velocity = int.from_bytes(self._can_msg[4:6], byteorder='little', signed=False)
        actual_velocity = int.from_bytes(self._can_msg[6:8], byteorder='little', signed=False)
        self._data = [msg.steering_angle,linear_voltage,requested_velocity,actual_velocity]

        return msg



### --------------------------- P22 CanBus Messages --------------------------- ###

class FrontWheelEncodersMsgP22(CanInterfaceMessage):
    can_id = 0x310
    byte_size = 8
    msg_type = WheelSpeed

    def to_ROS(self) -> msg_type:
        msg = self.msg_type()

        # We need bytes 4-5 and 6-7 -> hex characters 8-11 and 12-15 respectively
        msg.left_wheel = int.from_bytes(self._can_msg[4:6], byteorder='big', signed=False)
        msg.right_wheel = int.from_bytes(self._can_msg[6:8], byteorder='big', signed=False)
        msg.actual_speed = (msg.right_wheel + msg.left_wheel) / 2 / 9.5493 * 0.1995
        return msg


class RearWheelEncodersMsgP22(CanInterfaceMessage):
    can_id = 0x4D1
    byte_size = 8
    msg_type = WheelSpeed

    def to_ROS(self) -> msg_type:
        msg = self.msg_type()

        # We need bytes 4-5 and 6-7 -> hex characters 8-11 and 12-15 respectively
        msg.left_wheel = int.from_bytes(self._can_msg[4:6], byteorder='big', signed=False)
        msg.right_wheel = int.from_bytes(self._can_msg[6:8], byteorder='big', signed=False)
        msg.actual_speed = (msg.right_wheel + msg.left_wheel) / 2 / 9.5493 * 0.1995
        return msg


class SteeringAngleMsgP22(CanInterfaceMessage):
    can_id = 0x4D2
    byte_size = 8
    msg_type = SteeringAngle

    def to_ROS(self) -> msg_type:
        msg = self.msg_type()
        
        # We need byte 4 -> hex characters 8-9
        msg.steering_angle = int.from_bytes(self._can_msg[4:5], byteorder='big', signed=True)
        return msg


class BrakePressureMsgP22(CanInterfaceMessage):
    can_id = 0x301
    byte_size = 8
    msg_type = BrakePressure

    def to_ROS(self) -> msg_type:
        msg = self.msg_type()

        # We need bytes 2 and 3 -> hex characters 4-5 and 6-7 respectively
        msg.front_cylinder = int.from_bytes(self._can_msg[2:3], byteorder='big', signed=True)
        msg.rear_cylinder = int.from_bytes(self._can_msg[3:4], byteorder='big', signed=True)
        return msg