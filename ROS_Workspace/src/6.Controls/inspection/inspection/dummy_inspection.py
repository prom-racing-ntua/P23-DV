import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from custom_msgs.msg import *
from math import sin, cos, pi


COMMAND_FREQUENCY = 40  # [Hz]
STATE_FREQUENCY = 5     # [Hz]

TORQUE_COMMAND = 0.7    # [N*m]
MAX_STEERING = 24.0     # [mm]
STEERING_PERIOD = 5     # [sec]
MISSION_DURATION = 10   # [sec]

class InspectionMission(Node):
    def __init__(self) -> None:
        super().__init__('inspection')

        self._status = DriverlessStatus.LV_ON
        self._steering_angle = 0
        self._actual_torque = 0
        self._send_time = None
        self._prev_send = 0.0
        self._reached_goal = False

        self._steering_command = self.declare_parameter('steering', 0.0).value
        self._brake_command = self.declare_parameter('brake', 0.0).value
        # self._steering_command = MAX_STEERING

        self._command_publisher = self.create_publisher(TxControlCommand ,'/control_commands', 10)
        self._state_publisher = self.create_publisher(TxSystemState ,'/system_state', 10)
        self._steering_sub = self.create_subscription(RxSteeringAngle, 'canbus/steering_angle', self.set_steering, 10)
        self._motor_sub = self.create_subscription(RxVehicleSensors, 'canbus/sensor_data', self.set_motor, 10)
        self._mission_sub = self.create_subscription(MissionSelection, 'canbus/mission_selection', self.lock_mission, 10)
        self._as_status_sub = self.create_subscription(AutonomousStatus, 'canbus/autonomous_status', self.go_signal, 10)

        self._state_timer = self.create_timer(1/STATE_FREQUENCY, self.send_state)

        self.get_logger().info(f"Inspection Node Initialized")


    def set_steering(self, msg:RxSteeringAngle) -> None:
        self._steering_angle = msg.steering_angle
        # self.get_logger().info(f"Received rack displacement {self._steering_angle}")
        # if abs(self._steering_angle - self._steering_command) < 1e-3 and not self._reached_goal and self._send_time is not None:
        #     self._reached_goal = True
        #     response_time = self.get_time() - self._send_time
        #     self.get_logger().warn(f"Response time to reach {self._steering_command} mm: {response_time}")
        #     self._steering_command *= -1
        #     self._reached_goal = False
        return


    def set_motor(self, msg:RxVehicleSensors) -> None:
        self._actual_torque = msg.motor_torque_actual
        # self.get_logger().info(f"Received motor actual torque {self._steering_angle}")
        # if abs(self._actual_torque - TORQUE_COMMAND) < 1e-3:
        #     response_time = self.get_time() - self._send_time
        #     self.get_logger().warn(f"Response time to reach {TORQUE_COMMAND} mm: {response_time}", once=True)
        return


    def send_commands(self) -> None:
        time = self.get_time() - self._start_time
        if self._status != DriverlessStatus.DV_DRIVING:
            return
        
        msg = TxControlCommand()

        self._steering_command = self.get_parameter('brake').value
        msg.brake_pressure_target = self._steering_command

        self._steering_command = self.get_parameter('steering').value
        msg.steering_angle_target = self._steering_command


        msg.speed_actual = 0
        msg.speed_target = 0
        msg.motor_torque_target = TORQUE_COMMAND

        # Normal inspection
        # msg.steering_angle_target = MAX_STEERING * sin(2*pi/STEERING_PERIOD * time)
        # msg.steering_angle_target = self._steering_command

        # This is for dynamic setting of commands
        # prev = self._steering_command
        # if self._steering_command != prev: 
            # self.get_logger().error("Why??")
            # self._send_time = None
            # self._reached_goal = False
        
        # if self._send_time is None:
        #     self.get_logger().warn(f'Steering command set to: {self._steering_command} mm')
        #     self._send_time = self.get_time()
        
        self._command_publisher.publish(msg)        
        
        # self.get_logger().info(f"Time passed: {time}")
        # if time > MISSION_DURATION:
            # self._status = DriverlessStatus.MISSION_FINISHED
            # self.get_logger().warn("Mission Finished", once=True)
        return


    def send_state(self) -> None:
        msg = TxSystemState()

        msg.dv_status.id = self._status

        msg.vn_200_error = False
        msg.vn_300_error = False
        msg.camera_left_error = False
        msg.camera_right_error = False

        msg.clock_error = False
        msg.camera_inference_error = False
        msg.velocity_estimation_error = False
        msg.slam_error = False
        msg.mpc_controls_error = False
        msg.path_planning_error = False
        msg.pi_pp_controls_error = False

        msg.lap_counter = 0
        msg.cones_count_actual = 0
        msg.cones_count_all = 0

        self._state_publisher.publish(msg)
        return


    def lock_mission(self, msg:MissionSelection) -> None:
        if msg.mission_selected == Mission.INSPECTION:
            self._status = DriverlessStatus.DV_READY
        elif msg.mission_selected == Mission.MISSION_UNLOCKED:
            self._status = DriverlessStatus.LV_ON


    def go_signal(self, msg:AutonomousStatus) -> None:
        if msg.id == AutonomousStatus.AS_DRIVING and self._status != DriverlessStatus.DV_DRIVING:
            self._status = DriverlessStatus.DV_DRIVING
            self.send_state()
            self._command_timer = self.create_timer(1/COMMAND_FREQUENCY, self.send_commands)
            self._start_time = self.get_time()
            self.get_logger().info(f"Received Go-Signal. Start time: {self._start_time}")


    def get_time(self):
        sec, nano = self.get_clock().now().seconds_nanoseconds()
        return sec + round(nano / 10**9, 3)



def main(args=None) -> None:
    rclpy.init(args=args)
    inspection = InspectionMission()
    executor = SingleThreadedExecutor()

    try:
        rclpy.spin(inspection, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        inspection.destroy_node()
        rclpy.shutdown()