# Message sent to the Can-USB node
# Autonomous System and PC Signals for the datalogger and VCU


# DV Status Flags
DriverlessStatus dv_status

# Sensor Status Flags
bool        vn_200_error
bool        vn_300_error
bool        camera_right_error
bool        camera_left_error
# bool        camera_center_error

# Node Status Flags
bool        clock_error
bool        camera_inference_error
bool        velocity_estimation_error
bool        slam_error
bool        mpc_controls_error
bool        path_planning_error
bool        pi_pp_controls_error


# Lap and Cone Counters
uint8        lap_counter
uint8        cones_count_actual
uint16       cones_count_all