# Message that is sent by the Lifecycle Manager to the P23 Node and
# shows if there is a problem with a node. If there is, then we should
# go into NODE_PROBLEM state.

# bool=1 means ERROR (so as to match with the rest of the EV system)

bool        clock_ok
bool        inference_ok
bool        velocity_estimation_ok
bool        slam_ok
bool        mpc_controls_ok
bool        path_planning_ok
bool        pi_pp_controls_ok

bool        camera_right_ok
bool        camera_left_ok