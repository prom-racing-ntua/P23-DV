#!/usr/bin/env python3

import rclpy
from custom_msgs.msg import VelEstimation, TxControlCommand, TxSystemState, RxVehicleSensors, RxSteeringAngle, RxWheelSpeed, AutonomousStatus, MissionSelection
from custom_msgs.srv import SetTotalLaps
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from .telemetry_base import *

import threading

NODE = None
GUI = None

class Data:
    def __init__(self):
        self.actual_speed = 0
        self.target_speed = 0
        self.accel_x = 0
        self.accel_y = 0
        self.lap_count = 0
        self.total_laps = 0
        self.target_torque = 0
        self.actual_torque = 0
        self.target_steer = 0
        self.actual_steer = 0
        self.target_brake = 0
        self.actual_brake_f = 0
        self.actual_brake_r = 0
        self.errors = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.yaw_i = [0, 0, 0, 0]
        self.yaw = 0
        self.vy = 0
        self.dv_status = 0
        self.as_status = 0
        self.mission = 0
        self.ins_mode = 0

class TelemetryNode(Node):
    def __init__(self):
        super().__init__("telemetry_data")
        
        self.sub_velest = self.create_subscription(VelEstimation, '/velocity_estimation', self.velocity_callback, 10)
        
        self.sub_controls = self.create_subscription(TxControlCommand, '/control_command', self.controls_callback, 10)
        
        self.sub_syst = self.create_subscription(TxSystemState, '/system_state', self.system_callback, 10)
        
        self.sub_sensors = self.create_subscription(RxVehicleSensors, '/sensor_data', self.sensor_callback, 10)
        
        self.sub_steering = self.create_subscription(RxSteeringAngle, '/steering_angle', self.steering_callback, 10)
        
        self.sub_wheels = self.create_subscription(RxWheelSpeed, '/wheel_encoders', self.wheel_callback, 10)
        
        self.sub_as = self.create_subscription(AutonomousStatus, '/autonomous_status', self.as_callback, 10)
        
        self.sub_miss = self.create_subscription(MissionSelection, '/mission_selection', self.mission_callback, 10)
        
        self.data = Data()
        
        timer_period = 0.1
        self.time = self.create_timer(timer_period, self.update_all)
        
    def update_all(self):
        #GUI.state.change_all(self.data.dv_status, self.data.as_status, self.data.mission)
        global GUI
        start = time.time()
        GUI.velocity.set_target(self.data.target_speed)
        GUI.velocity.set_actual(self.data.actual_speed)
        
        GUI.accel.update_ax(self.data.accel_x)
        GUI.accel.update_ay(self.data.accel_y)
        
        GUI.lap.set_lap(self.data.lap_count)
        
        GUI.torque.set_target(self.data.target_torque)
        GUI.torque.set_actual(self.data.actual_torque)
        
        GUI.steer.set_target(self.data.target_steer)
        GUI.steer.set_actual(self.data.actual_steer)
        
        GUI.brake.set_target(self.data.target_brake)
        GUI.brake.set_both(self.data.actual_brake_f, self.data.actual_brake_r)
        
        GUI.error.update(self.data.errors)
        end = time.time()
        print(end-start)
    
    def velocity_callback(self, msg):
        self.data.actual_speed = msg.velocity_x
        self.data.vy = msg.velocity_y
        self.data.yaw = msg.yaw_rate
        self.data.accel_x = msg.acceleration_x
        self.data.accel_y = msg.acceleration_y
        
    def controls_callback(self, msg):
        self.data.target_speed = msg.speed_target / 3.6
        self.data.target_torque = msg.motor_torque_target
        self.data.target_steer = msg.steering_angle_target
        self.data.target_brake = msg.brake_pressure_target
        
    def system_callback(self, msg):
        self.data.dv_status = msg.dv_status.id
        self.data.errors =  [msg.camera_inference_error, 
                            msg.velocity_estimation_error, 
                            msg.slam_error, 
                            msg.mpc_controls_error, 
                            msg.pi_pp_controls_error, 
                            msg.path_planning_error,
                            msg.camera_left_error, 
                            msg.camera_right_error,
                            msg.vn_200_error, 
                            msg.vn_400_error]
        self.data.ins_mode = msg.ins_mode
        self.data.lap_count = msg.lap_counter
        
    def sensor_callback(self, msg):
        self.data.actual_torque = msg.motor_torque_actual
        self.data.actual_brake_f = msg.brake_pressure_front
        self.data.actual_brake_r = msg.brake_pressure_rear
        
    def steering_callback(self, msg):
        self.data.actual_steer = msg.steering_angle
        
    def wheel_callback(self, msg):
        self.data.yaw_i = [msg.front_left, msg.front_right, msg.rear_left, msg.rear_right]
        
    def as_callback(self, msg):
        self.data.as_status = msg.id
        
    def mission_callback(self, msg):
        self.data.mission = msg.id
        
        
        
def main(args=None):
    
    # GUI = TelemetryApp()
    
    
    # rclpy.init(args=args)
    # NODE = TelemetryNode()
        
    # executor = SingleThreadedExecutor()
    # try:
    #     rclpy.spin(NODE, executor)
    #     GUI.mainloop()
    # except (KeyboardInterrupt, ExternalShutdownException):
    #     pass
    # finally:
    #     NODE.destroy_node()
    #     rclpy.shutdown()
    global GUI
    global NODE   
    rclpy.init(args=args)
    NODE = TelemetryNode()
    GUI = TelemetryApp()
    thread_spin = threading.Thread(target = rclpy.spin, args = (NODE, ))
    thread_spin.start()
    
    
    GUI.mainloop()
    
    NODE.destroy_node()
    rclpy.shutdown()
    thread_spin.join()
    

if __name__ == "__main__":
        main()