# Message sent to the Can-USB node
# Autonomous System and PC Signals for the datalogger and VCU


# DV Status Flags
DriverlessStatus dv_status


# Sensor Status Flags
bool        vn_200_ok
bool        vn_300_ok
bool        camera_right_ok
bool        camera_left_ok
# bool        camera_center_ok

# Node Status Flags
bool        clock_ok
bool        camera_inference_ok
bool        velocity_estimation_ok
bool        slam_ok
bool        mpc_controls_ok
bool        path_planning_ok
bool        pi_pp_controls_ok


# Lap and Cone Counters
uint8        lap_counter
uint8        cones_count_actual
uint16       cones_count_all