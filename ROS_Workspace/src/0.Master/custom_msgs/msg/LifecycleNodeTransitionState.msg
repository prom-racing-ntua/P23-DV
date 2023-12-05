# Purpose is to monitor Lifecycle node transitions
# 0: Transition not requested (inactive nodes)
# 1: Transition Pending
# 2: Transition Complete
# 3: Transition Failed
 
uint8 transition_requested
# 0: none 1: on_startup 2: on_mission_locked 3: on_mission_unlocked 4: on_as_ready 5: on_as_driving 6: on_shutdown 10: same_transition 
uint8 acquisition_right
uint8 acquistion_left
uint8 inference
uint8 velocity_estimation 
uint8 slam 
uint8 path_planning 
uint8 pid_pp_controller
uint8 mpc_controller
uint8 inspection_controller
uint8 saltas
