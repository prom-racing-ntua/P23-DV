import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor, ExternalShutdownException
from ament_index_python import get_package_share_directory
import csv
import os
import pandas as pd
import atexit
import time

from vectornav_msgs.msg import ImuGroup, InsGroup, GpsGroup, AttitudeGroup
from custom_msgs.msg import WheelSpeed, BrakePressure, SteeringAngle, VelEstimation

FREQUENCY = 10  # Hz
class DataLogger(Node):
    def __init__(self) -> None:
        super().__init__("data_logger")
        # Parameter to configure the amount of logged data
        self.start_time = int(self.get_clock().now().nanoseconds/10**9)
        self.declare_parameter('verbose', True)
        self._verbose = self.get_parameter('verbose').value

        # Quality of Service preset for the subscribers
        sensor_data_profile = QoSPresetProfiles.get_from_short_key("SENSOR_DATA")
        self.imu_flag = 0
        self.ins_flag = 0
        self.string_file_300 = ''
        self.string_file_200 = ''
        # Create dictionary to hold incoming data
        if self._verbose:
            dict_vars_200 = ["Time_Received","ins_mode",
                             "accel_x","accel_y","accel_z",
                             "u_x","u_y","yaw_rate",
                             "roll","pitch","yaw",
                             "pos_x","pos_y","pos_z"]
            
            dict_vars_300 = ["Time_Received","ins_mode",
                             "accel_x","accel_y","accel_z",
                             "u_x","u_y","yaw_rate",
                             "roll","pitch","yaw",
                             "pos_x","pos_y","pos_z"]
            
            dict_vars_common = ["Time_Received","ins_mode_200","ins_mode_300",
                                "u_x_dv","u_y_dv","yaw_rate_dv","cov_dv",
                                "fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel", 
                                "steering_angle",
                                "f_brake", "r_brake"]
        else:
            dict_vars = ["Time_Received", "ins_mode_200", "ins_mode_300",
                         "u_x_dv","u_y_dv","yaw_rate_dv","cov",
                         "accel_x", "accel_y", "accel_z", "ang_vel_x", "ang_vel_y", "ang_vel_z",
                         "ins_mode", "vel_x", "vel_y", "vel_z", "velu",
                         "fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel", "steering_angle",
                         "f_brake", "r_brake"]
        self._dict_200 = dict(zip(dict_vars_200, [None]*len(dict_vars_200)))
        self._df_200 = pd.DataFrame(columns=dict_vars_200)
        self._dict_300 = dict(zip(dict_vars_300, [None]*len(dict_vars_300)))
        self._df_300 = pd.DataFrame(columns=dict_vars_300)
        self._dict_common = dict(zip(dict_vars_common, [None]*len(dict_vars_common)))
        self._df_common = pd.DataFrame(columns=dict_vars_common)

        # Subscribers for all the sensor topics needed
        self._vn_200_imu_subscriber = self.create_subscription(ImuGroup, '/vn_200/raw/imu', self.vn_200_imu_callback, sensor_data_profile)
        self._vn_300_ins_subscriber = self.create_subscription(InsGroup, '/vn_300/raw/ins', self.vn_300_ins_callback, sensor_data_profile)
        self._front_hall_subscriber = self.create_subscription(WheelSpeed, '/canbus/front_hall_sensors', self.front_hall_callback, sensor_data_profile)
        self._rear_hall_subscriber = self.create_subscription(WheelSpeed, '/canbus/rear_hall_sensors', self.rear_hall_callback, sensor_data_profile)
        self._steering_subscriber = self.create_subscription(SteeringAngle, '/canbus/steering_angle', self.steering_callback, sensor_data_profile)
        self._brakes_subscriber = self.create_subscription(BrakePressure, '/canbus/brake_pressure', self.brake_callback, sensor_data_profile)
        self._velocity_subscriber = self.create_subscription(VelEstimation, '/velocity_estimation', self.velocity_callback, sensor_data_profile)
        if self._verbose:
            # Extra topics for data logging
            self.get_logger().warn('Verbosity Set to True')
            self._vn_200_ins_subscriber = self.create_subscription(ImuGroup, '/vn_200/raw/ins', self.vn_200_ins_callback, sensor_data_profile)
            self._vn_300_imu_subscriber = self.create_subscription(ImuGroup, '/vn_300/raw/imu', self.vn_300_imu_callback, sensor_data_profile)
            self._vn_200_gps_subscriber = self.create_subscription(GpsGroup, "/vn_200/raw/gps2", self.vn_200_gps_callback, sensor_data_profile)
            self._vn_300_gps_subscriber = self.create_subscription(GpsGroup, "/vn_300/raw/gps2", self.vn_300_gps_callback, sensor_data_profile)
            self._vn_200_attitude_subscriber = self.create_subscription(AttitudeGroup, "/vn_200/raw/attitude", self.vn_200_att_callback, sensor_data_profile)
            self._vn_300_attitude_subscriber = self.create_subscription(AttitudeGroup, "/vn_300/raw/attitude", self.vn_300_att_callback, sensor_data_profile)

        # Timer for taking the measurements
        time_step = 1 / FREQUENCY
        self._timer = self.create_timer(time_step, self.timer_callback)

        self.get_logger().warn('Starting Logging')

        # Command to save file at exit
        atexit.register(self.save_data)
        
    #vn200/300 logging
    def vn_200_imu_callback(self, msg) -> None:
        self.get_logger().warn('IMU callback')
        share_dir = get_package_share_directory("data_logger")
        if(self.imu_flag==0):
            self.string_file_200 = os.path.join(share_dir, "../../../../testingLogs", f"vn200_{self.get_clock().now().seconds_nanoseconds()[0]}.txt" )
            with open(self.string_file_200, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([time.time(), msg.accel.x,msg.accel.y,msg.accel.z])
            self.imu_flag=1
        else:
            with open(self.string_file_200, 'a', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([time.time(), msg.accel.x,msg.accel.y,msg.accel.z])
        self.get_logger().warn('Wrote IMU data')
        # self._dict_200.update({'accel_x': msg.accel.x, 'accel_y': msg.accel.y, 'accel_z': msg.accel.z})
        
    def vn_300_imu_callback(self, msg) -> None:
        return None
        # self._dict_300.update({'accel_x': msg.accel.x, 'accel_y': msg.accel.y, 'accel_z': msg.accel.z})

    def vn_200_ins_callback(self, msg) -> None:
        self.get_logger().warn('Mpika INS_200')
        return None
        # self._dict_200.update({'ins_mode': msg.insstatus.mode, 'u_x': msg.velbody.x, 'u_y': msg.velbody.y, 'yaw_rate': msg.velbody.z})
        # self._dict_common.update({'ins_mode_200': msg.insstatus.mode})
    
    def vn_300_ins_callback(self, msg) -> None:
        self.get_logger().warn('INS callback')
        share_dir = get_package_share_directory("data_logger")
        self.get_logger().warn(f'INS_MODE {msg.insstatus.mode}')
        if(self.ins_flag==0):
            self.string_file_300 = os.path.join(share_dir, "../../../../testingLogs", f"vn300_{self.get_clock().now().seconds_nanoseconds()[0]}.txt" )
            with open(self.string_file_300, 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([self.get_clock().now().seconds_nanoseconds(), msg.insstatus.mode,msg.velbody.x,msg.velbody.y,msg.velbody.z])
            self.ins_flag=1
        else:
            with open(self.string_file_300, 'a', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([self.get_clock().now().seconds_nanoseconds(), msg.insstatus.mode,msg.velbody.x,msg.velbody.y,msg.velbody.z])
        self.get_logger().warn('Wrote INS data')
        # self._dict_300.update({'ins_mode': msg.insstatus.mode, 'u_x': msg.velbody.x, 'u_y': msg.velbody.y, 'yaw_rate': msg.velbody.z})
        # self._dict_common.update({'ins_mode_300': msg.insstatus.mode})
    
    def vn_200_att_callback(self, msg) -> None:
        return None
        # self._dict_200.update({'yaw': msg.yawpitchroll.x, 'pitch': msg.yawpitchroll.y, 'roll': msg.yawpitchroll.z})
    
    def vn_300_att_callback(self, msg) -> None:
        return None
        # self._dict_300.update({'yaw': msg.yawpitchroll.x, 'pitch': msg.yawpitchroll.y, 'roll': msg.yawpitchroll.z})

    def vn_200_gps_callback(self, msg) -> None:
        return None
        # self._dict_200.update({'pos_x': msg.posu.x, 'pos_y': msg.posu.y, 'pos_z': msg.posu.z})

    def vn_300_gps_callback(self, msg) -> None:
        return None
        # self._dict_300.update({'pos_x': msg.posu.x, 'pos_y': msg.posu.y, 'pos_z': msg.posu.z})

    #begin common logging
    def front_hall_callback(self, msg) -> None:
        return None
        # self._dict_common.update({'fl_wheel': msg.left_wheel, 'fr_wheel': msg.right_wheel})

    def rear_hall_callback(self, msg) -> None:
        return None
        # self._dict_common.update({'rl_wheel': msg.left_wheel, 'rr_wheel': msg.right_wheel})

    def steering_callback(self, msg) -> None:
        return None
        # self._dict_common.update({'steering_angle': msg.steering_angle})

    def brake_callback(self, msg) -> None:
        return None
        # self._dict_common.update({'f_brake': msg.front_cylinder, 'r_brake':msg.rear_cylinder})

    def velocity_callback(self, msg) -> None:
        return None
        # self._dict_common.update({'u_x_dv': msg.velocity_x, 'u_y_dv': msg.velocity_y, 'yaw_rate_dv': msg.yaw_rate, 'cov_dv': msg.variance_matrix})

    def timer_callback(self) -> None:
        time = self.get_clock().now().seconds_nanoseconds()
        t = time[0] + round(time[1] / 10**9, 3)
        # self._dict_200.update({'Time_Received': t})
        # self._dict_300.update({'Time_Received': t})
        # self._dict_common.update({'Time_Received': t})
        new_df_200 = pd.DataFrame([self._dict_200])
        self._df_200 = pd.concat([self._df_200, new_df_200], axis=0, ignore_index=True)
        new_df_300 = pd.DataFrame([self._dict_300])
        self._df_300 = pd.concat([self._df_300, new_df_300], axis=0, ignore_index=True)
        new_df_common = pd.DataFrame([self._dict_common])
        self._df_common = pd.concat([self._df_common, new_df_common], axis=0, ignore_index=True)

    def save_data(self):
        share_dir = get_package_share_directory("data_logger")
        # self._df_common.to_csv(os.path.join(share_dir, "../../../../testingLogs", f"velCommonLog_{self.get_clock().now().seconds_nanoseconds()[0]}.csv" ))
        # self._df_200.to_csv(os.path.join(share_dir, "../../../../testingLogs", f"vn200_{self.get_clock().now().seconds_nanoseconds()[0]}.csv" ))
        # self._df_300.to_csv(os.path.join(share_dir, "../../../../testingLogs", f"vn300_{self.get_clock().now().seconds_nanoseconds()[0]}.csv" ))
        self.get_logger().warn("Log Data Saved")


def main(args=None):
    rclpy.init(args=args)
    logger = DataLogger()

    executor = MultiThreadedExecutor(num_threads=2)
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