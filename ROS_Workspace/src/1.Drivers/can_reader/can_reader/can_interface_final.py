import serial
import atexit
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException

from custom_msgs.msg import *
from .Messages import *
from node_logger.node_logger import *
class CanInterface(Node):
    def __init__(self) -> None:
        super().__init__("can_interface")

        # Load port and node parameters from config file
        self.load_parameters()
        if self.disable_send:
            self.get_logger().warn("Output to USB DISABLED !!!")
        else:
            self.get_logger().warn("Output to USB ENABLED !!!")
        CanInterfaceMessage.node_handle = self

        # Mission Selection Variables
        self._received_mission = None
        self._locked_mission = None
        self._shuting_down = False
        self.exitflag = 0
        self.only_logs = False
        self.ignore_unlock = False
        self.time_of_lock = 0

        try:
            # Create the comm port to the Can/USB board
            self._serial_port = serial.Serial(port=self._port, baudrate=self._baud_rate, timeout=self._timeout)
            # Send empty message to Can/USB board to initialize its operation (is necessary by design)
            self._serial_port.write(b'0000')
        except Exception as e:
            self.get_logger().error("Unable to open serial port. \nError: {:s}\n Entering log-only mode".format(repr(e)))
            self.only_logs = True

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
            SystemHealthMsg.msg_type        : SystemHealthMsg,
            SteeringParamsMsg.msg_type      : SteeringParamsMsg
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
        SteeringParamsMsg.ros_subscriber = self.create_subscription(TxSteeringParams,'/steering_params',self.universal_callback,10)

        self.get_logger().info(create_new_run_log())

        #for timestamp logging 
        self.sensor_timestamp_log: Logger = Logger("canbus_sensor")
        self.get_logger().info(self.sensor_timestamp_log.check())
        self.wheel_timestamp_log: Logger = Logger("canbus_wheel")
        self.get_logger().info(self.wheel_timestamp_log.check())
        self.steer_timestamp_log: Logger = Logger("canbus_steering")
        self.get_logger().info(self.steer_timestamp_log.check())
        self.controls_timestamp_log: Logger = Logger("canbus_controls")
        self.get_logger().info(self.controls_timestamp_log.check())
        self.velocity_timestamp_log: Logger = Logger("canbus_velocity")
        self.get_logger().info(self.velocity_timestamp_log.check())
        self.health_timestamp_log: Logger = Logger("canbus_health")
        self.get_logger().info(self.health_timestamp_log.check())

        

        # Define Incoming Message Logger
        self._in_msgs_logger = {
            MissionMsg.can_id               : None,
            SensorVariablesMsg.can_id       : self.sensor_timestamp_log,
            WheelEncodersMsg.can_id         : self.wheel_timestamp_log,
            SteeringAngleMsg.can_id         : self.steer_timestamp_log,
            AsStatusMsg.can_id              : None
        }

        # Define Outgoing Message Logger
        self._out_msgs_logger = {
            ActuatorCommandsMsg.msg_type    : self.controls_timestamp_log,
            KinematicVariablesMsg.msg_type  : self.velocity_timestamp_log,
            SystemHealthMsg.msg_type        : self.health_timestamp_log,
            SteeringParamsMsg.msg_type      : None
        }

        self.run_path = None

        for log in [self.sensor_timestamp_log,self.wheel_timestamp_log,self.steer_timestamp_log,self.controls_timestamp_log,self.velocity_timestamp_log,self.health_timestamp_log]:
            if log.get_run_path() is not None:
                self.run_path = log.get_run_path()
                break

        
        if self.run_path is not None:
            try:
                self.error_file = open(os.path.join(self.run_path, 'can_reader_error_log.txt'), 'w')
            except:
                self.error_file = None

        if self.run_path is None or self.error_file is None:
            self.get_logger().error(f'Unable to open error file. Continuing without error logging! \n')
        else:
            self.get_logger().info(f'Error log file opened successfully.\n')

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
        if not self.only_logs:atexit.register(self._serial_port.close)
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
        self.disable_send = self.declare_parameter('disable_send', False).value


    def universal_callback(self, msg) -> None:
        try:
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

            if self.disable_send and type(msg)==ActuatorCommandsMsg.msg_type:
                return

            # Write message to terminal and serial port
            # self.get_logger().info(f"Outgoing Can message in bytes:\n{out_bytes}\n")
            end_time_1 = self.get_clock().now().nanoseconds/10**6
            if not self.only_logs:self._serial_port.write(out_bytes)
            end_time_2 = self.get_clock().now().nanoseconds/10**6

            logger = self._out_msgs_logger[type(msg)]
            try:
                global_index = msg.global_index
            except:
                global_index = 0
            if logger is not None:
                logger( start_time.nanoseconds/10**6    , 0, global_index, temp_msg.data())
                logger( (end_time_1 + end_time_2) / 2   , 1, global_index, temp_msg.data())

            # Print the total processing time
            # self.get_logger().info(f"Time to process message inside uni callback: {(self.get_clock().now() - start_time).nanoseconds / 10**6} ms")
            return
        except Exception as e:
            # Error detected during writing
            # Current error handling policy (kai kala) is:
            # if KinematicVariablesMsg: just dont send (only necessary for datalogger)
            # if Controls/Health/SteeringParams: Enter AS Emergency
            should_abort: bool = False

            self.get_logger().error(f"Error during writing: {repr(e)}\n")
            self.error_file.write(f'{self.get_clock().now().nanoseconds/10**6:0.3f}\t Error during writing: {repr(e)}\n')

            try:
                tp = type(msg)
            except Exception as f:
                self.get_logger().error(f"Error during determining msg type: {repr(f)}\n")
                self.error_file.write(f'{self.get_clock().now().nanoseconds/10**6:0.3f}\t Error during determining msg type: {repr(f)}\n"')
                should_abort = True
            
            if tp == KinematicVariablesMsg.msg_type:
                self.get_logger().error("Error is from Velocity msg. Not necessary to abort.\n")
                self.error_file.write(f'{self.get_clock().now().nanoseconds/10**6:0.3f}\t Error is from Velocity msg. Not necessary to abort.\n')
                should_abort = False
            else:
                self.get_logger().error(f"Error is System Critical ({tp}). Necessary to abort.\n")
                self.error_file.write(f'{self.get_clock().now().nanoseconds/10**6:0.3f}\t "Error is System Critical ({tp}). Necessary to abort.\n')
                should_abort = True

            if should_abort:
                # Create Node Problem System Health msg and send it. The LV System will promptly return an AS Emergency Status
                Message = SystemHealthMsg
                to_send = TxSystemState()
                to_send.dv_status.id = DriverlessStatus.NODE_PROBLEM
                temp_msg = Message(to_send)
                out_bytes = temp_msg.to_CanMsg()

                if not self.only_logs:
                    self._serial_port.write(out_bytes)

                self.get_logger().error(f"Sent Node Problem DV_Status Expecting AS Emergency soon\n")
                self.error_file.write(f'{self.get_clock().now().nanoseconds/10**6:0.3f}\t "Sent Node Problem DV_Status Expecting AS Emergency soon\n')




    def read_serial(self) -> None:
        '''
        Checks for new messages finds their id and passes the data to the corresponding parser function
        '''
        try:
            raise ValueError('0')
            if self.only_logs:return
            # self.get_logger().info("Started reading from serial")
            # Check serial port's buffer size. If a lot of messages have accumulated in the buffer we flush the old ones 
            # to receive the latests ones. 
            # The current limit is defined by the time delay we want to allow the messages to have (assuming an average 
            # message size of 11 bytes). So the number multiplied by 11*read_frequency is the allowed time delay.
            # Generally we don't want this to happen so if it occurs we should adjust the read speed accordingly.
            # assert self.exitflag == 1, "bad exitflag"
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
                # self.get_logger().info(f"Message_can_id is : {Message.can_id}")
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

                    if self.ignore_unlock:
                        self.ignore_unlock = False
                        self.get_logger().warn("Unlocked too fast. Assuming sparking and ignoring ...")
                        return
                
                # Create the message object according to its type
                ros_msg = temp_msg.to_ROS()
                pub_time_1 = self.get_clock().now().nanoseconds/10**6
                temp_msg.ros_publisher.publish(ros_msg)
                pub_time_2 = self.get_clock().now().nanoseconds/10**6

                logger = self._in_msgs_logger[msg_id]
                if logger is not None:
                    logger(start_time.nanoseconds/10**6 , 0, 0, temp_msg.data())
                    logger((pub_time_1 + pub_time_2) / 2, 1, 0, temp_msg.data())

                # Print the total processing time
                # self.get_logger().info(f"Time to process message in read_serial: {(self.get_clock().now() - start_time).nanoseconds / 10**6} ms")
            # self.get_logger().info("Finished reading from serial")
            else:
                return
            #      self.get_logger().info("Receiving empty stream")
            return
        except Exception as e:
            # Error detected during reading
            # Current error handling policy (kai kala) is:
            # if WheelEncoders(for now) or SteeringAngle: just dont send (only necessary for velocity if encoders were available)
            # if mission/sensor: send node error and wait for as emergency
            # if as status: send node error out AND as emergency in (we consider an invalid as status msg as an as emergency msg)

            should_abort: int  = 0

            self.get_logger().error(f"Error during reading: {repr(e)}\n")
            self.error_file.write(f'{self.get_clock().now().nanoseconds/10**6:0.3f}\t Error during reading: {repr(e)}\n')

            if 'Message' not in locals():
                self.get_logger().error(f"Unable to discern incoming message type.\n")
                self.error_file.write(f'{self.get_clock().now().nanoseconds/10**6:0.3f}\t Unable to discern incoming message type.\n')
                should_abort = 2

            elif Message in [WheelEncodersMsg, SteeringAngleMsg]:
                self.get_logger().error("Error is from WheelEncoders or SteeringAngle msg. Not necessary to abort.\n")
                self.error_file.write(f'{self.get_clock().now().nanoseconds/10**6:0.3f}\t Error is from WheelEncoders or SteeringAngle msg. Not necessary to abort.\n')
                should_abort = 0
            elif Message in [MissionMsg, SensorVariablesMsg]:
                self.get_logger().error(f"Error is System Critical ({Message}). Sending Node Error.\n")
                self.error_file.write(f'{self.get_clock().now().nanoseconds/10**6:0.3f}\t "Error is System Critical ({Message}). Sending Node Error.\n')
                should_abort = 1
            else:
                self.get_logger().error(f"Error is System Critical ({Message}). Sending Node Error AND enter AS Emergency AND abort.\n")
                self.error_file.write(f'{self.get_clock().now().nanoseconds/10**6:0.3f}\t "Error is System Critical ({Message}). Sending Node Error AND enter AS Emergency AND abort.\n')
                should_abort = 2


            if should_abort == 1:
                # Create Node Problem System Health msg and send it. The LV System will promptly return an AS Emergency Status
                Message = SystemHealthMsg
                temp_msg = Message()
                out_bytes = temp_msg.to_CanMsg()

                if not self.only_logs:
                    self._serial_port.write(out_bytes)

                self.get_logger().error(f"Sent Node Problem DV_Status Expecting AS Emergency soon\n")
                self.error_file.write(f'{self.get_clock().now().nanoseconds/10**6:0.3f}\t "Sent Node Problem DV_Status Expecting AS Emergency soon\n')
            elif should_abort == 2:
                # Error is from as status so the above handling would not work. Instead we:
                # - Send node problem outward so lv system enters as emergency
                # - Send control command outward with full breaks so the last command stops being applied
                # - Send AS emergency msg inward so lifecycle manager shuts down everyone else
                # - Shutdown Can Interface to isolate systems
                Message = SystemHealthMsg
                to_send = TxSystemState()
                to_send.dv_status.id = DriverlessStatus.NODE_PROBLEM
                temp_msg = Message(to_send)
                out_bytes = temp_msg.to_CanMsg()

                if not self.only_logs:
                    self._serial_port.write(out_bytes)

                    self.get_logger().error(f"Sent Node Problem DV_Status\n")
                    self.error_file.write(f'{self.get_clock().now().nanoseconds/10**6:0.3f}\t "Sent Node Problem DV_Status\n')

                # -

                Message = ActuatorCommandsMsg
                to_send = TxControlCommand()
                to_send.brake_pressure_target = 20.0
                temp_msg = Message(to_send)
                out_bytes = temp_msg.to_CanMsg()

                if not self.only_logs:
                    self._serial_port.write(out_bytes)

                self.get_logger().error(f"Sent Full brakes\n")
                self.error_file.write(f'{self.get_clock().now().nanoseconds/10**6:0.3f}\t "Sent Full brakes\n')

                # - 

                to_pub = AutonomousStatus()
                to_pub.id = AutonomousStatus.AS_EMERGENCY
                to_pub.label = "AS_EMERGENCY"

                AsStatusMsg.ros_publisher.publish(to_pub)

                self.get_logger().error(f"Sent AS_Emergency AS_Status\n")
                self.error_file.write(f'{self.get_clock().now().nanoseconds/10**6:0.3f}\t "Sent AS_Emergency AS_Status\n')

                # -

                exit(1)



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