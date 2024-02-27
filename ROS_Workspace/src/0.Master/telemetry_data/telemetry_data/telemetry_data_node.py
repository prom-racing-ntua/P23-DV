#!/usr/bin/env python3

import rclpy
from custom_msgs.msg import VelEstimation, TxControlCommand, TxSystemState, RxVehicleSensors, RxSteeringAngle, \
    RxWheelSpeed, AutonomousStatus, MissionSelection, LifecycleNodeTransitionState
from custom_msgs.srv import SetTotalLaps
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node

from .telemetry_base import *

import numpy as np
import math
import threading

NODE = None
GUI = None


class constants:
    def __init__(self):
        self.m = 193.5
        self.Iz = 110
        self.wheelbase = 1.590  # m
        self.C_tyre = 0.66
        self.P_air = 1.225
        self.Cd_A = 2
        self.Cl_A = 7
        self.gear_ratio = 3.9
        self.eff = 0.85
        self.R_wheel = 0.2054  # mm
        self.g = 9.81
        self.h_cog = 0.275
        self.wd = 0.467
        self.ftw = 1.22
        self.rtw = 1.22


class q_per_tire:
    def __init__(self):
        self.fl = 0
        self.fr = 0
        self.rl = 0
        self.rr = 0

    def set(self,a, b, c, d):
        self.fl = a
        self.fr = b
        self.rl = c
        self.rr = d
    def __str__(self):
        return str(self.fl) + ", " + str(self.fr) + ", " + str(self.rl) + ", " + str(self.rr)
    def __call__(self) -> tuple:
        return self.fl, self.fr, self.rl, self.rr

def comp_slip_ratio(wheel_speed: q_per_tire, vx, vy, d, yaw, wd, wb, ftw, rtw, R) -> q_per_tire:
    sr = q_per_tire()
    sr.rl = (wheel_speed.rl * R - (vx + (rtw / 2) * yaw)) / np.sqrt(1 + (vx + (rtw / 2) * yaw) ** 2)
    sr.rr = (wheel_speed.rr * R - (vx - (rtw / 2) * yaw)) / np.sqrt(1 + (vx - (rtw / 2) * yaw) ** 2)
    sr.fl = (wheel_speed.fl * R - (np.cos(d) * (vx + yaw * ftw / 2) + np.sin(d) * (vy + yaw * wd * wb))) / np.sqrt(1 + (np.cos(d) * (vx + yaw * ftw / 2) + np.sin(d) * (vy + yaw * wd * wb)) ** 2)
    sr.fr = (wheel_speed.fr * R - (np.cos(d) * (vx - yaw * ftw / 2) + np.sin(d) * (vy + yaw * wd * wb))) / np.sqrt(1 + (np.cos(d) * (vx - yaw * ftw / 2) + np.sin(d) * (vy + yaw * wd * wb)) ** 2)
    return sr

def comp_slip_angle(vy, yaw, vx, rtw, ftw, d, wd, wb) -> q_per_tire:
    sa = q_per_tire()
    sa.rl = np.arctan((vy - yaw * (1 - wd) * wb) / np.sqrt(1 + (vx + yaw * rtw / 2) ** 2))
    sa.rr = np.arctan( (vy - yaw * (1 - wd) * wb) / np.sqrt(1 + (vx - yaw * rtw / 2) ** 2))
    sa.fl = np.arctan((vy + yaw * wd * wb) / np.sqrt( 1 + (vx + yaw * ftw / 2) ** 2)) - d
    sa.fr = np.arctan((vy + yaw * wd * wb) / np.sqrt(1 + (vx - yaw * ftw / 2) ** 2)) - d
    return sa

def f_y(slip_angle: float, f_z: float) -> float:
    P = [-1.9391836, 2.46810824, -0.21654031, 5.53792739, 0.29777483,
         -0.87901614, -0.0990718, 7.02929697, -29.59686021, 0.97520799,
         -8.20731378, 0.5275139, -1.0086604, -0.61898062, -55.82849645,
         70.38725604, 4.67966457, 0.9878318]
    ex = 0.001
    Fz0 = 1112.0554070627252
    dfz = (f_z - Fz0) / Fz0
    IA = 0
    D = (abs(P[1]) + P[2] * dfz) * f_z / (1 + P[3] * IA * IA)
    C = abs(P[0])
    SVg = f_z * (P[12] + P[13] * dfz) * IA
    Kg = f_z * (P[16] + P[17] * dfz)
    BCD = P[8] * Fz0 * math.sin(2.0 * math.atan(f_z / ((P[9] + P[15] * IA * IA) * Fz0))) / (1.0 + P[10] * IA * IA)
    Sh = (Kg * IA - SVg) / (BCD + ex)
    ay = slip_angle + Sh
    E = (P[4] + P[5] * dfz) * (1.0 + P[14] * IA * IA - (P[6] + P[7] * IA) * np.sign(ay))
    B = BCD / (C * D + ex)
    Sv = f_z * (0 + 0 * dfz) + SVg
    psi = D * math.sin(C * math.atan(B * ay - E * (B * ay - math.atan(B * ay))))
    return psi + Sv

def f_x(slip_ratio: float, Fz: float, IA = 0) -> float:
    C_tire = 0.66
    P = [1.49897197e+00, 2.26070458e+00, -8.19377573e-02, -4.30583270e+00,
         - 3.94373761e-01, -3.25004019e+00, -1.35389672e+00, 3.95605635e+01,
         - 3.52985738e+01, -1.68401638e+00, 6.12624634e-04]
    kappa = slip_ratio
    ex = 0.001
    Fz0 = 1112.0554070627252
    dfz = (Fz - Fz0) / Fz0

    D = (abs(P[1]) - abs(P[2]) * dfz) * Fz * (1.0 - P[3] * IA * IA)
    C = P[0]
    Sh = 0.0 + 0.0 * dfz
    kappax = kappa + Sh
    E = (P[4] + P[5] * dfz + P[6] * dfz * dfz) * (1.0 - P[10] * np.sign(kappax))
    Kx = Fz * (P[7] + abs(P[8]) * dfz) * math.exp(P[9] * dfz)
    B = Kx / (C * D + ex)

    Sv = Fz * (0.0 + 0.0 * dfz)

    psi = D * math.sin(C * math.atan(B * kappax - E * (B * kappax - math.atan(B * kappax))))
    return  C_tire * (psi + Sv)

def compute_Fz(m, g, wd, ax, wb, ay, h, rtw, ftw, air_dens, ClA, vy, vx, ad =0.5) -> q_per_tire():
    Fz = q_per_tire()
    ax = ax / g
    ay = ay / g
    Fz.rl = (1/2)*m*g*wd + (1/2)*m*h*ax/wb + (1/2)*m*h*ay/rtw + (1/2)*ad*(1/2)*air_dens*ClA*vx**2
    Fz.rr = (1/2)*m*g*wd + (1/2)*m*h*ax/wb - (1/2)*m*h*ay/rtw + (1/2)*ad*(1/2)*air_dens*ClA*vx**2
    Fz.fl = (1/2)*m*g*(1-wd) - (1/2)*m*h*ax/wb + (1/2)*m*h*ay/ftw + (1/2)*(1-ad)*(1/2)*air_dens*ClA*vx**2
    Fz.fr = (1/2)*m*g*(1-wd) - (1/2)*m*h*ax/wb - (1/2)*m*h*ay/ftw + (1/2)*(1-ad)*(1/2)*air_dens*ClA*vx**2
    #print("Fz = ", (1/2)*m*g*wd, (1/2)*m*g*(1-wd), (1/2)*m*h*ax/wb)
    return Fz

# def f_f_z(v_x, lf, lr, m, g, P_air, CdA) -> float:
#     weight = m * g * lf / (lf + lr)
#     aero = 0.25 * P_air * CdA * v_x * v_x
#     return weight + aero
# 
# def f_r_z(v_x, lf, lr, m, g, P_air, CdA) -> float:
#     weight = m * g * lr / (lf + lr)
#     aero = 0.25 * P_air * CdA * v_x * v_x
#     return weight + aero

def Lr(ax, wheelbase, hcog, wd, w) -> float:
    a_x = ax / 9.81
    dwx = hcog * w * a_x / wheelbase
    wf = w * wd - dwx

    return wheelbase * (wf / w)

def Lf(Lr, wheelbase) -> float:
    return wheelbase - Lr

def mu(Fz: float) -> tuple:
    Fz0 = 1112.0554070627252
    dfz = (Fz - Fz0) / Fz0
    C_tire = 0.66
    mx_max = C_tire * (2.21891927 - 1.36151651e-07 * dfz)
    my_max = C_tire * (2.46810824 - 0.21654031 * dfz)
    return mx_max, my_max



class Data:
    def __init__(self) -> None:
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
        self.status = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.id = 0
        self.yaw_i = q_per_tire()
        self.yaw = 0
        self.vy = 0
        self.dv_status = 0
        self.as_status = 0
        self.mission = 0
        self.ins_mode = 0

        self.actual_speed_last_time = 0
        self.target_speed_last_time = 0
        self.accel_x_last_time = 0
        self.accel_y_last_time = 0
        self.lap_count_last_time = 0
        self.target_torque_last_time = 0
        self.actual_torque_last_time = 0
        self.target_steer_last_time = 0
        self.actual_steer_last_time = 0
        self.target_brake_last_time = 0
        self.actual_brake_f_last_time = 0
        self.actual_brake_r_last_time = 0
        self.errors_last_time = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.status_last_time = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.dv_status_last_time = 0
        self.as_status_last_time = 0
        self.mission_last_time = 0
        self.ins_mode_last_time = 0

        self.fx = q_per_tire()
        self.fy = q_per_tire()
        self.fz = q_per_tire()
        self.mx = q_per_tire()
        self.my = q_per_tire()



class TelemetryNode(Node):
    def __init__(self) -> None:
        super().__init__("telemetry_data")

        self.sub_velest = self.create_subscription(VelEstimation, '/velocity_estimation', self.velocity_callback, 10)

        self.sub_controls = self.create_subscription(TxControlCommand, '/control_commands', self.controls_callback, 10)

        self.sub_syst = self.create_subscription(TxSystemState, '/system_state', self.system_callback, 10)

        self.sub_sensors = self.create_subscription(RxVehicleSensors, '/canbus/sensor_data', self.sensor_callback, 10)

        self.sub_steering = self.create_subscription(RxSteeringAngle, '/canbus/steering_angle', self.steering_callback, 10)

        self.sub_wheels = self.create_subscription(RxWheelSpeed, '/canbus/wheel_encoders', self.wheel_callback, 10)

        self.sub_as = self.create_subscription(AutonomousStatus, '/canbus/autonomous_status', self.as_callback, 10)

        self.sub_miss = self.create_subscription(MissionSelection, '/canbus/mission_selection', self.mission_callback, 10)

        self.sub_trans_status = self.create_subscription(LifecycleNodeTransitionState, 'transition_state', self.transition_callback, 10)

        self.data = Data()
        self.const = constants()

        self.time_of_as_ready = None
        self.as_ready_delay = False

        timer_period = 0.1
        self.time = self.create_timer(timer_period, self.update_all)

    def uptodate(self, last, current):
        if current - last <= 0.1 * 1e9: return 0
        elif current - last <= 1: return 1
        else:return 2

    def update_all(self) -> None:
        global GUI
        # start = time.time()
        tm = time.monotonic_ns()

        GUI.state_.change_all(self.data.dv_status, self.data.as_status, self.data.mission, self.as_ready_delay)

        GUI.velocity.set_target(self.data.target_speed, self.uptodate(self.data.target_speed_last_time, tm))
        GUI.velocity.set_actual(self.data.actual_speed, self.uptodate(self.data.actual_speed_last_time, tm))

        GUI.accel.update_ax(self.data.accel_x)
        GUI.accel.update_ay(self.data.accel_y)

        GUI.lap.set_lap(self.data.lap_count)

        GUI.torque.set_target(self.data.target_torque)
        GUI.torque.set_actual(self.data.actual_torque)

        GUI.steer.set_target(self.data.target_steer)
        GUI.steer.set_actual(self.data.actual_steer)

        GUI.brake.set_target(self.data.target_brake)
        GUI.brake.set_both(self.data.actual_brake_f, self.data.actual_brake_r)

        GUI.error.update(self.data.errors, self.data.ins_mode, self.data.id, self.data.status)

        GUI.update_idletasks()
        fx, fy, fz, mx, my = self.create_ellipses()

        #print(fz)

        GUI.ell_fl.update(fx.fl, fy.fl, fz.fl, mx.fl, my.fl)
        GUI.ell_fr.update(fx.fr, fy.fr, fz.fr, mx.fr, my.fr)
        GUI.ell_rl.update(fx.rl, fy.rl, fz.rl, mx.rl, my.rl)
        GUI.ell_rr.update(fx.rr, fy.rr, fz.rr, mx.rr, my.rr)

        # end = time.time()
        # print(end-start)

    def velocity_callback(self, msg: VelEstimation) -> None:
        self.data.actual_speed = msg.velocity_x
        self.data.vy = msg.velocity_y
        self.data.yaw = msg.yaw_rate
        self.data.accel_x = msg.acceleration_x
        self.data.accel_y = msg.acceleration_y

        tm = time.monotonic_ns()

        self.data.actual_speed_last_time = tm
        self.data.accel_x_last_time = tm
        self.data.accel_y_last_time = tm

    def controls_callback(self, msg: TxControlCommand) -> None:
        tm = time.monotonic_ns()

        self.data.target_speed = msg.speed_target / 3.6
        self.data.target_torque = msg.motor_torque_target
        self.data.target_steer = msg.steering_angle_target*180/np.pi
        self.data.target_brake = msg.brake_pressure_target

        self.data.target_speed_last_time = tm
        self.data.target_torque_last_time = tm
        self.data.target_steer_last_time = tm
        self.data.target_brake_last_time = tm

    def system_callback(self, msg: TxSystemState) -> None:
        tm = time.monotonic_ns()

        self.data.dv_status = msg.dv_status.id
        self.data.errors = [msg.camera_inference_error,
                            msg.velocity_estimation_error,
                            msg.slam_error,
                            msg.mpc_controls_error,
                            msg.pi_pp_controls_error,
                            msg.path_planning_error,
                            msg.camera_left_error,
                            msg.camera_right_error,
                            msg.vn_200_error,
                            msg.vn_300_error]
        self.data.ins_mode = msg.ins_mode
        self.data.lap_count = msg.lap_counter

        self.data.dv_status_last_time = tm
        self.data.errors_last_time = tm
        self.data.ins_mode_last_time = tm
        self.data.lap_count_last_time = tm

    def transition_callback(self, msg: LifecycleNodeTransitionState) -> None:
        self.data.id = msg.transition_requested
        self.data.status = [msg.acquisition_right, 
                            msg.acquistion_left, 
                            msg.inference, 
                            msg.velocity_estimation, 
                            msg.slam, 
                            msg.path_planning, 
                            msg.pid_pp_controller, 
                            msg.mpc_controller, 
                            msg.inspection_controller, 
                            msg.saltas]
        GUI.error.update(self.data.errors, self.data.ins_mode, self.data.id, self.data.status)

    def sensor_callback(self, msg: RxVehicleSensors) -> None:
        tm = time.monotonic_ns()

        self.data.actual_torque = msg.motor_torque_actual
        self.data.actual_brake_f = msg.brake_pressure_front
        self.data.actual_brake_r = msg.brake_pressure_rear

        self.data.actual_torque_last_time = tm
        self.data.actual_brake_f_last_time = tm
        self.data.actual_brake_r_last_time = tm

    def steering_callback(self, msg: RxSteeringAngle) -> None:
        tm = time.monotonic_ns()

        self.data.actual_steer = msg.steering_angle * 180 / 3.14159

        self.data.actual_steer_last_time = tm

    def wheel_callback(self, msg: RxWheelSpeed) -> None:
        self.data.yaw_i.set(msg.front_left, msg.front_right, msg.rear_left, msg.rear_right)

    def as_callback(self, msg: AutonomousStatus) -> None:
        self.data.as_status = msg.id
        tm = time.monotonic_ns()
        self.data.as_status_last_time = tm

        if self.time_of_as_ready is None and msg.id==2:
            self.time_of_as_ready = tm
        elif self.time_of_as_ready is not None and msg.id==2:
            self.as_ready_delay = tm - self.time_of_as_ready >= 5e9
        

    def mission_callback(self, msg: MissionSelection) -> None:
        self.data.mission = msg.mission_selected

        tm = time.monotonic_ns()
        self.data.mission_last_time = tm

    def create_ellipses(self):
        vx = self.data.actual_speed
        vy = self.data.vy
        ax = self.data.accel_x
        ay = self.data.accel_y
        yaw = self.data.yaw
        d = self.data.actual_steer
        wheel_speed = self.data.yaw_i
        wheelbase = self.const.wheelbase
        h_cog = self.const.h_cog
        wd  = self.const.wd
        m =self.const.m
        rtw, ftw = self.const.rtw, self.const.ftw

        lr = Lr(ax = ax, wheelbase = wheelbase, hcog = h_cog, wd = h_cog, w = h_cog)
        lf = Lf(lr, wheelbase)

        sa = comp_slip_angle(vy, yaw, vx, rtw, ftw, d, h_cog, wheelbase)
        sr = comp_slip_ratio(wheel_speed, vx, vy, d, yaw, h_cog, wheelbase, ftw, rtw, self.const.R_wheel)

        fz = compute_Fz(m, self.const.g, wd, ax, wheelbase, ay, h_cog, rtw, ftw, self.const.P_air, self.const.Cl_A, vy, vx)

        # Front Left
        mx_max_1, my_max_1 = mu(Fz = fz.fl)
        fx_1 = f_x(sr.fl, fz.fl)
        fy_1 = f_y(sa.fl, fz.fl)

        # Front Right
        mx_max_2, my_max_2 = mu(Fz=fz.fr)
        fx_2 = f_x(sr.fr, fz.fr)
        fy_2 = f_y(sa.fr, fz.fr)

        # Rear Left
        mx_max_3, my_max_3 = mu(Fz=fz.rl)
        fx_3 = f_x(sr.rl, fz.rl)
        fy_3 = f_y(sa.rl, fz.rl)

        # Rear Right
        mx_max_4, my_max_4 = mu(Fz=fz.rr)
        fx_4 = f_x(sr.rr, fz.rr)
        fy_4 = f_y(sa.rr, fz.rr)

        if wheel_speed.fl == 0 and wheel_speed.fr == 0 and wheel_speed.rl == 0 and wheel_speed.rr == 0:
            fx_1, fx_2, fx_3, fx_4, fy_1, fy_2, fy_3, fy_4 = 0, 0, 0, 0, 0, 0, 0, 0

        self.data.fx.set(fx_1, fx_2, fx_3, fx_4)
        self.data.fy.set(fy_1, fy_2, fy_3, fy_4)
        self.data.mx.set(mx_max_1, mx_max_2, mx_max_3, mx_max_4)
        self.data.my.set(my_max_1, my_max_2, my_max_3, my_max_4)
        self.data.fz = fz

        # print(lf, lr)
        # print(sa)
        # print(sr)
        # print(fz)
        # print(self.data.fx)
        # print(self.data.fy)
        # print(self.data.mx)
        # print(self.data.my)

        return self.data.fx, self.data.fy, self.data.fz, self.data.mx, self.data.my


def main(args=None) -> None:
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
    thread_spin = threading.Thread(target=rclpy.spin, args=(NODE,))
    thread_spin.start()

    GUI.mainloop()

    NODE.destroy_node()
    rclpy.shutdown()
    thread_spin.join()


if __name__ == "__main__":
    main()
