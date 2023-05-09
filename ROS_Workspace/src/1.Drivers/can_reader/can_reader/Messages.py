from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from custom_msgs.msg import *
from math import pi


### --------------------------- Helper Functions --------------------------- ###

def floatToBytes(num:float, byteorder:str='big', multiplier:int=256, signed=False) -> bytes:
    i_num = int(num * multiplier)
    print(i_num)
    return i_num.to_bytes(2, byteorder, signed=signed)

def bitsToByte(bit_list:list) -> bytes:
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

def get_bit(value, bit):
    return (value >> bit) & 1


### --------------------------- Base Message Class --------------------------- ###

class CanInterfaceMessage():
    '''
    The base class for every message of the CanBus to/from ROS Interface
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
        # Check if the given message is from CanBus or from ROS
        if (isinstance(received_message, (bytes, bytearray))):
            # If it is from CanBus process it and send it to ROS
            
            if len(received_message) != self.byte_size + 3:
                raise ValueError("Received message has invalid size")
            received_id = int.from_bytes(received_message[0:2], byteorder='big', signed=False)
            received_byte_size = int.from_bytes(received_message[2:3], byteorder='big', signed=False)

            # Checks that the received message is valid
            if received_id != self.can_id: raise ValueError('Received message id does not match the default value')
            if received_byte_size != self.byte_size: raise ValueError('Received message id does not match the default value')

            self.node_handle.get_logger().info(f"{received_message[3:]}")
            self._can_msg = received_message[3:]
            return

        elif (isinstance(received_message, self.msg_type)):
            # If it is from ROS encoded into bytes and transmit to CanBus
            self._ros_msg = received_message
            return

        else:
            raise TypeError(f"The given message is of unknown type {type(received_message)}")

    def to_ROS(self) -> msg_type:
        raise NotImplementedError("To ROS parser has not been defined in child class")

    def to_CanMsg(self) -> bytearray:
        raise NotImplementedError("To CanBus parser has not been defined in child class")



### --------------------------- P22 CanBus Messages --------------------------- ###

class FrontWheelEncodersMsg(CanInterfaceMessage):
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


class RearWheelEncodersMsg(CanInterfaceMessage):
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


class SteeringAngleMsg(CanInterfaceMessage):
    can_id = 0x4D2
    byte_size = 8
    msg_type = SteeringAngle

    def to_ROS(self) -> msg_type:
        msg = self.msg_type()
        
        # We need byte 4 -> hex characters 8-9
        msg.steering_angle = int.from_bytes(self._can_msg[4:5], byteorder='big', signed=True)
        return msg


class BrakePressureMsg(CanInterfaceMessage):
    can_id = 0x301
    byte_size = 8
    msg_type = BrakePressure

    def to_ROS(self) -> msg_type:
        msg = self.msg_type()

        # We need bytes 2 and 3 -> hex characters 4-5 and 6-7 respectively
        msg.front_cylinder = int.from_bytes(self._can_msg[2:3], byteorder='big', signed=True)
        msg.rear_cylinder = int.from_bytes(self._can_msg[3:4], byteorder='big', signed=True)
        return msg


### --------------------------- P22 CanBus Messages --------------------------- ###

class ActuatorCommandsMsg(CanInterfaceMessage):
    can_id = 0x01
    byte_size = 8
    msg_type = TxControlCommand

    def to_CanMsg(self) -> bytearray:
        DEFAULT_SERVICE_BRAKE_PRESSURE = 5
        out_msg = bytearray(self.byte_size)
        out_msg[0] = self.can_id

        # TODO: Needs to be transformed from rads to steering rack displacement first
        out_msg[1:3] = floatToBytes(self._ros_msg.steering_angle_target, signed=True)
        out_msg[3:5] = floatToBytes(self._ros_msg.motor_torque_target, signed=True)
        
        # Brake pressure target is bool so we output a standard brake pressure to the service brake
        out_msg[5] = DEFAULT_SERVICE_BRAKE_PRESSURE if self._ros_msg.brake_pressure_target else 0

        # Speed Actual and speed target
        out_msg[6] = self._ros_msg.speed_actual
        out_msg[7] = self._ros_msg.speed_target
        return out_msg


class KinematicVariablesMsg(CanInterfaceMessage):
    can_id = 0x02
    byte_size = 7
    msg_type = VelEstimation

    def to_CanMsg(self) -> bytearray:
        out_msg = bytearray(self.byte_size)
        out_msg[0] = self.can_id

        out_msg[1:3] = floatToBytes(self._ros_msg.acceleration_x, multiplier=512, signed=True)
        out_msg[3:5] = floatToBytes(self._ros_msg.acceleration_y, multiplier=512, signed=True)
        out_msg[5:7] = floatToBytes(self._ros_msg.yaw_rate*180/pi, multiplier=128, signed=True)
        return out_msg


class SystemHealthMsg(CanInterfaceMessage):
    can_id = 0x03
    byte_size = 7
    msg_type = TxSystemState

    def to_CanMsg(self) -> bytearray:
        out_msg = bytearray(self.byte_size)
        out_msg[0] = self.can_id

        # Set lap counter and dv-state flags TODO
        first_byte = 0
        out_msg[1] = first_byte

        out_msg[2] = self._ros_msg.cones_count_actual
        out_msg[3:5] = self._ros_msg.cones_count_all
        
        # Set sensor status byte TODO
        sensor_status = 0
        out_msg[5] = sensor_status
        
        # Set node status byte TODO
        node_status =0
        out_msg[6] = node_status

        return out_msg


class MissionHandshakeMsg(CanInterfaceMessage):
    can_id = 0x04
    byte_size = 2

    def __init__(self, received_message) -> None:
        super().__init__(received_message)


class MissionMsg(CanInterfaceMessage):
    can_id = 0x304
    byte_size = 1
    msg_type = Mission

    LOCKED = 7

    # Handles the mission handshake and locking
    def handle_mission(self) -> bool:
        mission_confirmed = False
        mission_byte = int.from_bytes(self._can_msg, byteorder='big')
        self.node_handle.get_logger().info(f"{mission_byte}")
        bits = []
        for bit in range(8):
            bits.append(get_bit(mission_byte, bit))
        self.node_handle.get_logger().info(f"{bits}")
        is_locked = bits[self.LOCKED]
        ones = bits.count(True) - is_locked
        # This should not happen ever
        if ones != 1:
            # TODO Error Handling so no reselection of mission
            self.node_handle.get_logger().error("Received Multiple Missions")
            out_msg = b'\x04\x00'
            return False
        else:
            mission = bits.index(1) + 1
            assert mission != self.LOCKED + 1, "Mission Locked but non selected?!"

            if is_locked and self.node_handle._received_mission == mission:
                mission_confirmed = True
                self.node_handle._locked_mission = mission
                self.node_handle.get_logger().warn(f"Mission Confirmed {hex(self.node_handle._locked_mission)}")
            else:
                self.node_handle._received_mission = mission
            
            # Send back received message for acknowledgment
            out_msg = bytearray(2)
            out_msg[0] = 0x04
            out_msg[1] = mission_byte
        self.node_handle._serial_port.write(out_msg)
        return mission_confirmed
    
    def to_ROS(self) -> msg_type:
        ros_msg = self.msg_type()
        ros_msg.id = self.node_handle._locked_mission
        return ros_msg



if __name__=='__main__':
    a = FrontWheelEncodersMsg(0)
    b = FrontWheelEncodersMsg(b'00')