import rclpy
from rclpy.node import Node
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy._rclpy_pybind11 import InvalidHandle
from pynput import keyboard
import time
from custom_msgs.msg import *
from custom_msgs.srv import SetTotalLaps
from math import sin, cos, pi, radians

COMMAND_FREQUENCY = 40          # [Hz]

TORQUE_COMMAND = 4.0            # [N*m]
MAX_STEERING = radians(15.0)    # [rad]
STEERING_PERIOD = 8             # [sec]
MISSION_DURATION = 24           # [sec]

class InspectionMission(Node):
    def __init__(self,name=None) -> None:
        super().__init__('inspection')
        self.node = rclpy.create_node(name or type(self).__name__)
        self.mission_finished = False
        self._steering_angle = 0.0
        self._actual_torque = 0.0
        #changes
        self.mode = self.declare_parameter('mode','bench').value
        self._steering_command = self.declare_parameter('steering', 0.0).value  #[mm]
        self._brake_command = self.declare_parameter('brake', 0.0).value        #[bar]
        self._torque_command = self.declare_parameter('torque', 0.0).value      #[Nm]
        self.mode = self.declare_parameter('mode','bench').value
        #changes
        self.get_logger().warn(f"\n-- Inspection Node Created in mode",self.mode)

    def on_configure(self, state:State) -> TransitionCallbackReturn:
        self._command_publisher = self.create_publisher(TxControlCommand ,'/control_commands', 10)
        self._state_publisher = self.create_publisher(TxSystemState ,'/system_state', 10)

        self._steering_sub = self.create_subscription(RxSteeringAngle, 'canbus/steering_angle', self.set_steering, 10)
        self._motor_sub = self.create_subscription(RxVehicleSensors, 'canbus/sensor_data', self.set_motor, 10)

        self._command_timer = self.create_timer(1/COMMAND_FREQUENCY, self.send_commands)
        self._command_timer.cancel()

        self.get_logger().warn(f"\n-- Inspection Configured!")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state:State) -> TransitionCallbackReturn:
        self._command_timer.cancel()
        self._command_timer.reset()

        self._start_time = self.get_time()

        self.get_logger().warn(f"\n-- Inspection Activated!")
        return super().on_activate(state)
    
    def on_deactivate(self, state:State) -> TransitionCallbackReturn:
        self._command_timer.cancel()
        
        self.get_logger().warn(f"\n-- Inspection Deactivated!")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state:State) -> TransitionCallbackReturn:
        self._command_timer.cancel()
        self.destroy_timer(self._command_timer)

        self.destroy_publisher(self._command_publisher)
        self.destroy_publisher(self._state_publisher)

        self.destroy_subscription(self._steering_sub)
        self.destroy_subscription(self._motor_sub)
        
        self._steering_angle = 0.0
        self._actual_torque = 0.0

        self.get_logger().warn(f"\n-- Inspection Un-Configured!")
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state:State) -> TransitionCallbackReturn:
        if (state.state_id == 1):
            self.get_logger().info(f'\n-- Inspection Shutdown!')
            return TransitionCallbackReturn.SUCCESS

        self._command_timer.cancel()
        self.destroy_timer(self._command_timer)

        self.destroy_publisher(self._command_publisher)
        self.destroy_publisher(self._state_publisher)

        self.destroy_subscription(self._steering_sub)
        self.destroy_subscription(self._motor_sub)

        self.get_logger().info(f"\n-- Inspection Shutdown!")
        return TransitionCallbackReturn.SUCCESS

    def send_commands(self) -> None:
        if self.mission_finished:
            msg = TxControlCommand()
            msg.brake_pressure_target = 5.0 #value at finish
            self._command_publisher.publish(msg)  
            return

        time = self.get_time() - self._start_time
        msg = TxControlCommand()

        msg.speed_actual = 0
        msg.speed_target = 0
        # Constant torque and sinusoidal movement of the steering wheel
        msg.motor_torque_target = TORQUE_COMMAND
        msg.steering_angle_target = MAX_STEERING * sin(2*pi/STEERING_PERIOD * time)
   
        self._command_publisher.publish(msg)       
        
        if time > MISSION_DURATION:
            self.mission_finished = True

            finished_cli = self.create_client(SetTotalLaps, "/p23_status/set_total_laps")
            if not finished_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("P23 Service is not available")
            else:
                req = SetTotalLaps.Request()
                req.total_laps = 0
                self.future = finished_cli.call_async(req)
                self.future.add_done_callback(self.client_callback)
        return

    def set_steering(self, msg:RxSteeringAngle) -> None:
        self._steering_angle = msg.steering_angle
        return

    def set_motor(self, msg:RxVehicleSensors) -> None:
        self._actual_torque = msg.motor_torque_actual
        return

    def get_time(self):
        sec, nano = self.get_clock().now().seconds_nanoseconds()
        return sec + round(nano / 10**9, 3)
    
    def client_callback(self, response):
        if response is not None:
            self.get_logger().warn("Mission Finished")
        else:
            self.get_logger().error("Client call failed")
        return

def main(args=None) -> None:
    rclpy.init(args=args)
    lifecycle_inspection = InspectionMission()
    executor = SingleThreadedExecutor()

    try:
        rclpy.spin(lifecycle_inspection, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            lifecycle_inspection.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass