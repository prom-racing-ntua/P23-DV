import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException

from custom_msgs.msg import *
from math import sin, cos, pi, radians


COMMAND_FREQUENCY = 40          # [Hz]

TORQUE_COMMAND = 1.2            # [N*m]
MAX_STEERING = radians(31.2)    # [rad]
STEERING_PERIOD = 6             # [sec]
MISSION_DURATION = 30           # [sec]


class InspectionMission(Node):
    def __init__(self) -> None:
        super().__init__('inspection')
        self._steering_angle = 0.0
        self._actual_torque = 0.0
        self.get_logger().info(f"Inspection Node Initialized")

    def on_configure(self, state:State) -> TransitionCallbackReturn:
        self._command_publisher = self.create_publisher(TxControlCommand ,'/control_commands', 10)
        self._state_publisher = self.create_publisher(TxSystemState ,'/system_state', 10)

        self._steering_sub = self.create_subscription(RxSteeringAngle, 'canbus/steering_angle', self.set_steering, 10)
        self._motor_sub = self.create_subscription(RxVehicleSensors, 'canbus/sensor_data', self.set_motor, 10)

        self._command_timer = self.create_timer(1/COMMAND_FREQUENCY, self.send_commands)
        self._command_timer.cancel()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state:State) -> TransitionCallbackReturn:
        self._command_timer.cancel()
        self._command_timer.reset()

        self._start_time = self.get_time()
        return super().on_activate(state)
    
    def on_deactivate(self, state:State) -> TransitionCallbackReturn:
        self._command_timer.cancel()
        
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state:State) -> TransitionCallbackReturn:
        self._command_timer.cancel()
        self._command_timer.destroy()

        self._command_publisher.destroy()
        self._state_publisher.destroy()

        self._steering_sub.destroy()
        self._motor_sub.destroy()
        
        self._steering_angle = 0.0
        self._actual_torque = 0.0

        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state:State) -> TransitionCallbackReturn:
        self._command_timer.cancel()
        self._command_timer.destroy()

        self._command_publisher.destroy()
        self._state_publisher.destroy()

        self._steering_sub.destroy()
        self._motor_sub.destroy()

        return TransitionCallbackReturn.SUCCESS

    def send_commands(self) -> None:
        time = self.get_time() - self._start_time
        msg = TxControlCommand()

        msg.speed_actual = 0
        msg.speed_target = 0
        # Constant torque and sinusoidal movement of the steering wheel
        msg.motor_torque_target = TORQUE_COMMAND
        msg.steering_angle_target = MAX_STEERING * sin(2*pi/STEERING_PERIOD * time)
   
        self._command_publisher.publish(msg)        
        
        if time > MISSION_DURATION:
            # Mission Finished, should call p23_status or something
            # Send a SLAM message that has the mission_finished flag set
            self.get_logger().warn("Mission Finished", once=True)
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



def main(args=None) -> None:
    rclpy.init(args=args)
    lifecycle_inspection = InspectionMission()
    executor = SingleThreadedExecutor()

    try:
        rclpy.spin(lifecycle_inspection, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        lifecycle_inspection.destroy_node()
        rclpy.shutdown()