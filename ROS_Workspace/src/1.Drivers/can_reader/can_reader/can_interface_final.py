import serial
import atexit

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException

from custom_msgs.msg import *
from .Messages import *


class CanInterface(Node):
    def __init__(self) -> None:
        super().__init__("can_interface")

        # Load port and node parameters from config file
        self.load_parameters()
        CanInterfaceMessage.node_handle = self

        # Mission Selection Variables
        self._received_mission = None
        self._locked_mission = None
        self._shuting_down = False

        # Create the comm port to the Can/USB board
        self._serial_port = serial.Serial(port=self._port, baudrate=self._baud_rate, timeout=self._timeout)
        # Send empty message to Can/USB board to initialize its operation (is necessary by design)
        self._serial_port.write(b'0000')

        # Define Incoming Messages
        self._in_msgs = {
            MissionMsg.can_id               : MissionMsg,
            SensorVariablesMsg.can_id       : SensorVariablesMsg,
            WheelEncodersMsg.can_id         : WheelEncodersMsg,
            SteeringAngleMsg.can_id         : SteeringAngleMsg,
            AsStatusMsg.can_id              : AsStatusMsg
        }

        # Define Outgoing Messages
        self._out_msgs = {
            ActuatorCommandsMsg.msg_type    : ActuatorCommandsMsg,
            KinematicVariablesMsg.msg_type  : KinematicVariablesMsg,
            SystemHealthMsg.msg_type        : SystemHealthMsg
        }

        # Define ROS Publishers for the incoming messages
        MissionMsg.ros_publisher = self.create_publisher(MissionSelection, 'mission_selection', 10)
        SensorVariablesMsg.ros_publisher = self.create_publisher(RxVehicleSensors, 'sensor_data', 10)
        WheelEncodersMsg.ros_publisher = self.create_publisher(RxWheelSpeed, 'wheel_encoders', 10)
        SteeringAngleMsg.ros_publisher = self.create_publisher(RxSteeringAngle, 'steering_angle', 10)
        AsStatusMsg.ros_publisher = self.create_publisher(AutonomousStatus, 'autonomous_status', 10)

        # Define ROS Subscribers for the outgoing messages
        ActuatorCommandsMsg.ros_subscriber = self.create_subscription(TxControlCommand, '/control_commands', self.universal_callback, 10)
        KinematicVariablesMsg.ros_subscriber = self.create_subscription(VelEstimation, '/velocity_estimation', self.universal_callback, 10)
        SystemHealthMsg.ros_subscriber = self.create_subscription(TxSystemState, '/system_state', self.universal_callback, 10)


        ## --- For P22 --- ##
        # self._in_msgs = {
        #     FrontWheelEncodersMsgP22.can_id : FrontWheelEncodersMsgP22,
        #     RearWheelEncodersMsgP22.can_id : RearWheelEncodersMsgP22,
        #     SteeringAngleMsgP22.can_id : SteeringAngleMsgP22,
        #     BrakePressureMsgP22.can_id : BrakePressureMsgP22
        # }
        # FrontWheelEncodersMsg.ros_publisher = self.create_publisher(WheelSpeed, 'front_hall_sensors', 10)
        # RearWheelEncodersMsg.ros_publisher = self.create_publisher(WheelSpeed, 'rear_hall_sensors', 10)
        # SteeringAngleMsg.ros_publisher = self.create_publisher(SteeringAngle, 'steering_angle', 10)
        # BrakePressureMsg.ros_publisher = self.create_publisher(BrakePressure, 'brake_pressure', 10)


        # Timer to perform read actions from the Can/USB board
        self._read_timer = self.create_timer(1 / self.get_parameter('read_frequency').value, self.read_serial)

        # Close port at exit
        atexit.register(self._serial_port.close)
        self.get_logger().info("Can Interface Node is online")


    def load_parameters(self) -> None:
        '''
        Loads the nodes parameters from the given file
        '''
        self._port = self.declare_parameter('port', '/dev/ttyACM0').value
        self._baud_rate = self.declare_parameter('baudrate', 115200).value
        self._timeout = self.declare_parameter('timeout', 0.001).value
        self._read_frequency = self.declare_parameter('read_frequency', 200).value
        self.declare_parameter('service_target_pressure', 10).value


    def universal_callback(self, msg) -> None:
        '''
        Callback for all subscribers calling the to_CanMsg method of the Message class
        '''
        # self.get_logger().info("started uni callback")
        start_time = self.get_clock().now()

        try:
            Message = self._out_msgs[type(msg)]
        except KeyError:
            self.get_logger().error(f"Invalid message ros-type received: {type(msg)}")
            return

        temp_msg = Message(msg)
        out_bytes = temp_msg.to_CanMsg()

        # Write message to terminal and serial port
        # self.get_logger().info(f"Outgoing Can message in bytes:\n{out_bytes}\n")
        self._serial_port.write(out_bytes)

        # Print the total processing time
        # self.get_logger().info(f"Time to process message inside uni callback: {(self.get_clock().now() - start_time).nanoseconds / 10**6} ms")
        return



    def read_serial(self) -> None:
        '''
        Checks for new messages finds their id and passes the data to the corresponding parser function
        '''
        # self.get_logger().info("Started reading from serial")
        # Check serial port's buffer size. If a lot of messages have accumulated in the buffer we flush the old ones 
        # to receive the latests ones. 
        # The current limit is defined by the time delay we want to allow the messages to have (assuming an average 
        # message size of 11 bytes). So the number multiplied by 11*read_frequency is the allowed time delay.
        # Generally we don't want this to happen so if it occurs we should adjust the read speed accordingly.
        
        if self._serial_port.in_waiting > (11*self.get_parameter("read_frequency").value*0.2):
            self._serial_port.reset_input_buffer()
            self.get_logger().warn("Input buffer overflow. Flushing buffer.")
            return
        
        start_time = self.get_clock().now()
        serial_msg = self._serial_port.readline()
        if serial_msg != b'':
            # Receiving strange message that cannot be decoded, don't know why... [b'\xa8\xfe\x01 P\x00\x00\x00\x00\x00\x00\x00\xc8S\x00']
            try:
                serial_msg = bytearray.fromhex(serial_msg.strip().decode())
            except UnicodeDecodeError:
                # self.get_logger().error(f"Received message cannot be decoded {serial_msg}")
                return
            except:
                return

            # self.get_logger().info(f"Received message: {serial_msg}")
            # Can id is the first 2 bytes - 4 hex characters
            msg_id = int.from_bytes(serial_msg[0:2], byteorder='big', signed=False)

            # Get the message object from the received id, if it exists or throw an error
            try:
                Message = self._in_msgs[msg_id]
            except KeyError:
                self.get_logger().error(f"Invalid message can-id received: {hex(msg_id)}")
                return

            temp_msg = Message(serial_msg)

            # Check if we received new mission and handle it
            if Message == MissionMsg:
                try:
                    confirmed = temp_msg.handle_mission()
                except ValueError:
                    return
                # If mission is not locked wait for acknowledgment
                # else push the mission to the rest of the system
                if not confirmed:
                    self.get_logger().warn(f"Received new mission {hex(self._received_mission)}")
                    return
            
            # Create the message object according to its type
            ros_msg = temp_msg.to_ROS()
            temp_msg.ros_publisher.publish(ros_msg)
            
            # Print the total processing time
            # self.get_logger().info(f"Time to process message in read_serial: {(self.get_clock().now() - start_time).nanoseconds / 10**6} ms")
        # self.get_logger().info("Finished reading from serial")
        else:
            # self.get_logger().info("Receiving empty stream")
            return
        return



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