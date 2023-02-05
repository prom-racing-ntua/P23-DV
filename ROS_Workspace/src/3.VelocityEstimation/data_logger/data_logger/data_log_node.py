import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor, ExternalShutdownException

import pandas as pd
from datetime import datetime
import atexit

from vectornav_msgs.msg import ImuGroup, InsGroup, GpsGroup, AttitudeGroup
from custom_msgs.msg import WheelSpeed, BrakePressure, SteeringAngle, VelEstimation

FREQUENCY = 40  # Hz

class DataLogger(Node):
    def __init__(self) -> None:
        super().__init__("data_logger")
        # Parameter to configure the amount of logged data
        self.start_time = int(self.get_clock().now().nanoseconds/10**9)
        self.declare_parameter('verbose', False)
        self._verbose = self.get_parameter('verbose').value

        # Quality of Service preset for the subscribers
        sensor_data_profile = QoSPresetProfiles.get_from_short_key("SENSOR_DATA")

        # Create dictionary to hold incoming data
        if self._verbose:
            dict_vars = ["Time_Received",
                        "accel_x", "accel_y", "accel_z", "ang_vel_x", "ang_vel_y", "ang_vel_z",
                        "ins_mode", "vel_x", "vel_y", "vel_z", "velu",
                        "fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel", "steering_angle",
                        "f_brake", "r_brake",
                        "accel2_x", "accel2_y", "accel2_z", "ang_vel2_x", "ang_vel2_y", "ang_vel2_z",
                        "gps_vel_N", "gps_vel_E", "gps_vel_D", "gps2_vel_N", "gps2_vel_E", "gps2_vel_D",
                        "yaw", "pitch", "roll",
                        "u_x", "u_y", "yaw_rate", "cov"]
        else:
            dict_vars = ["Time_Received",
                        "accel_x", "accel_y", "accel_z", "ang_vel_x", "ang_vel_y", "ang_vel_z",
                        "ins_mode", "vel_x", "vel_y", "vel_z", "velu",
                        "fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel", "steering_angle",
                        "f_brake", "r_brake"]
        self._dict = dict(zip(dict_vars, [None]*len(dict_vars)))
        self._df = pd.DataFrame(columns=dict_vars)

        # Subscribers for all the sensor topics needed
        self._vn_200_subscriber = self.create_subscription(ImuGroup, '/vn_200/raw/imu', self.vn_imu_callback, sensor_data_profile)
        self._vn_300_subscriber = self.create_subscription(InsGroup, '/vn_300/raw/ins', self.vn_ins_callback, sensor_data_profile)
        self._front_hall_subscriber = self.create_subscription(WheelSpeed, '/canbus/front_hall_sensors', self.front_hall_callback, sensor_data_profile)
        self._rear_hall_subscriber = self.create_subscription(WheelSpeed, '/canbus/rear_hall_sensors', self.rear_hall_callback, sensor_data_profile)
        self._steering_subscriber = self.create_subscription(SteeringAngle, '/canbus/steering_angle', self.steering_callback, sensor_data_profile)
        self._brakes_subscriber = self.create_subscription(BrakePressure, '/canbus/brake_pressure', self.brake_callback, sensor_data_profile)
        self._velocity_subscriber = self.create_subscription(VelEstimation, '/velocity_estimation', self.velocity_callback, sensor_data_profile)
        if self._verbose:
            # Extra topics for data logging
            self.get_logger().warn('Verbosity Set to True')
            self._vn_300_imu_subscriber = self.create_subscription(ImuGroup, '/vn_300/raw/imu', self.vn_imu2_callback, sensor_data_profile)
            self._gps_subscriber = self.create_subscription(GpsGroup, "/vn_300/raw/gps", self.vn_gps_callback, sensor_data_profile)
            self._gps2_subscriber = self.create_subscription(GpsGroup, "/vn_300/raw/gps2", self.vn_gps2_callback, sensor_data_profile)
            self._attitude_subscriber = self.create_subscription(AttitudeGroup, "/vn_300/raw/attitude", self.vn_att_callback, sensor_data_profile)

        # Timer for taking the measurements
        time_step = 1 / FREQUENCY
        self._timer = self.create_timer(time_step, self.timer_callback)

        self.get_logger().warn('Starting Logging')

        # Command to save file at exit
        atexit.register(self.save_data)

    def vn_imu_callback(self, msg) -> None:
        self._dict.update({'accel_x': msg.accel.x, 'accel_y': msg.accel.y, 'accel_z': msg.accel.z, 
                          'ang_vel_x': msg.angularrate.x, 'ang_vel_y': msg.angularrate.y, 'ang_vel_z': msg.angularrate.z})

    def vn_imu2_callback(self, msg) -> None:
        self._dict.update({'accel2_x': msg.accel.x, 'accel2_y': msg.accel.y, 'accel2_z': msg.accel.z, 
                          'ang_vel2_x': msg.angularrate.x, 'ang_vel2_y': msg.angularrate.y, 'ang_vel2_z': msg.angularrate.z})

    def vn_ins_callback(self, msg) -> None:
        self._dict.update({'ins_mode': msg.insstatus.mode, 'vel_x': msg.velbody.x, 'vel_y': msg.velbody.y, 'vel_z': msg.velbody.z, 'velu': msg.velu})

    def vn_gps_callback(self, msg) -> None:
        self._dict.update({'gps_vel_N': msg.velned.x, 'gps_vel_E': msg.velned.y, 'gps_vel_D': msg.velned.z})

    def vn_gps2_callback(self, msg) -> None:
        self._dict.update({'gps2_vel_N': msg.velned.x, 'gps2_vel_E': msg.velned.y, 'gps2_vel_D': msg.velned.z})

    def vn_att_callback(self, msg) -> None:
        self._dict.update({'yaw': msg.yawpitchroll.x, 'pitch': msg.yawpitchroll.y, 'roll': msg.yawpitchroll.z})

    def front_hall_callback(self, msg) -> None:
        self._dict.update({'fl_wheel': msg.left_wheel, 'fr_wheel': msg.right_wheel})

    def rear_hall_callback(self, msg) -> None:
        self._dict.update({'rl_wheel': msg.left_wheel, 'rr_wheel': msg.right_wheel})

    def steering_callback(self, msg) -> None:
        self._dict.update({'steering_angle': msg.steering_angle})

    def brake_callback(self, msg) -> None:
        self._dict.update({'f_brake': msg.front_cylinder, 'r_brake':msg.rear_cylinder})

    def velocity_callback(self, msg) -> None:
        self._dict.update({'u_x': msg.u_x, 'u_y': msg.u_y, 'yaw_rate': msg.u_yaw, 'cov': msg.var_matrix})

    def timer_callback(self) -> None:
        time = self.get_clock().now().seconds_nanoseconds()
        t = time[0] + round(time[1] / 10**9, 3)
        self._dict.update({'Time_Received': t})
        new_df = pd.DataFrame([self._dict])
        self._df = pd.concat([self._df, new_df], axis=0, ignore_index=True)

    def save_data(self):
        self._df.to_csv(f"~/Prom_Driverless/logs/DataLogger_{self.start_time}.csv")
        self.get_logger().warn("Log Data Saved", )


def main(args=None):
    rclpy.init(args=args)
    logger = DataLogger()

    executor = MultiThreadedExecutor(num_threads=4)
    #executor = SingleThreadedExecutor()
    
    try:
        rclpy.spin(logger, executor=executor)
    except(KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()