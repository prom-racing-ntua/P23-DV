# Message sent to the Can-USB node
# Control Commands to be sent to the actuators and datalogger

uint8       speed_target            # [km/h] just some semi-random value (only for datalogger)
uint8       speed_actual            # [km/h] get actual speed from velocity estimation

float32     motor_torque_target     # [N*m]     - TODO(max and min values)
float32     steering_angle_target   # [rad]     - TODO(max and min values)
float32     brake_pressure_target   # [bar]
#bool     brake_pressure_target   # give as bool and output standard pressure target

