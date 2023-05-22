import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from custom_msgs.msg import *
from math import sin, pi


COMMAND_FREQUENCY = 60  # [Hz]
STATE_FREQUENCY = 5     # [Hz]

TORQUE_COMMAND = 10.0   # [N*m]
MAX_STEERING = -24.0     # [mm]
STEERING_PERIOD = 1     # [sec]
WAIT_TIME = 2           # [sec]
MISSION_DURATION = 10   # [sec]

class InspectionMission(Node):
    def __init__(self) -> None:
        super().__init__('inspection')
        self._mission_finished = False
        self._steering_angle = 0
        self._start_time = self.get_time()
        self._send_time = None

        self._command_publisher = self.create_publisher(TxControlCommand ,'p23_status/control_commands', 10)
        self._state_publisher = self.create_publisher(TxSystemState ,'p23_status/system_state', 10)
        self._steering_sub = self.create_subscription(RxSteeringAngle, 'steering_angle', self.set_steering, 10)

        self._command_timer = self.create_timer(1/COMMAND_FREQUENCY, self.send_commands)
        self._state_timer = self.create_timer(1/STATE_FREQUENCY, self.send_state)

        self.get_logger().info(f"Inspection Node Initialized. Start time: {self._start_time}")


    def set_steering(self, msg:RxSteeringAngle) -> None:
        self._steering_angle = msg.steering_angle
        self.get_logger().info(f"Received rack displacement {self._steering_angle}")
        if abs(self._steering_angle - MAX_STEERING) < 1e-3:
            response_time = self.get_time() - self._send_time
            self.get_logger().warn(f"Response time to reach {MAX_STEERING} mm: {response_time}", once=True)
        return


    def send_commands(self) -> None:
        time = self.get_time() - self._start_time
        if self._mission_finished or time < WAIT_TIME:
            return
        
        msg = TxControlCommand()

        if int(time) % 2 == 0:
            msg.brake_pressure_target = True
        else:
            msg.brake_pressure_target = False
        msg.speed_actual = 0
        msg.speed_target = 0
        msg.motor_torque_target = TORQUE_COMMAND

        msg.steering_angle_target = MAX_STEERING #* sin(2*pi/STEERING_PERIOD * time)
        if self._send_time is None:
            self._send_time = self.get_time()
        
        self._command_publisher.publish(msg)        
        
        # self.get_logger().info(f"Time passed: {time}")
        if time > MISSION_DURATION + WAIT_TIME:
            self._mission_finished = True
            self.get_logger().warn("Mission Finished", once=True)
        return


    def send_state(self) -> None:
        msg = TxSystemState()

        if self._mission_finished:
            msg.dv_status.id = DriverlessStatus.MISSION_FINISHED
        else:
            msg.dv_status.id = DriverlessStatus.DV_DRIVING

        msg.vn_200_ok = False
        msg.vn_300_ok = False
        msg.camera_left_ok = False
        msg.camera_right_ok = False

        msg.clock_ok = False
        msg.camera_inference_ok = False
        msg.velocity_estimation_ok = False
        msg.slam_ok = False
        msg.mpc_controls_ok = False
        msg.path_planning_ok = False
        msg.pi_pp_controls_ok = False

        msg.lap_counter = 0
        msg.cones_count_actual = 0
        msg.cones_count_all = 0

        self._state_publisher.publish(msg)
        return


    def get_time(self):
        sec, nano = self.get_clock().now().seconds_nanoseconds()
        return sec + round(nano / 10**9, 3)



def main(args=None) -> None:
    rclpy.init(args=args)
    can_interface = InspectionMission()
    executor = SingleThreadedExecutor()

    try:
        rclpy.spin(can_interface, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        can_interface.destroy_node()
        rclpy.shutdown()