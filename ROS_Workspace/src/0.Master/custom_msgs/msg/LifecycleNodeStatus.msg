# Message that is sent by the Lifecycle Manager to the P23 Node and
# shows if there is a problem with a node. If there is, then we should
# go into NODE_PROBLEM state.

# bool=1 means ERROR (so as to match with the rest of the EV system)

bool        clock_error
bool        inference_error
bool        velocity_estimation_error
bool        slam_error
bool        mpc_controls_error
bool        path_planning_error
bool        pi_pp_controls_error

bool        camera_right_error
bool        camera_left_error