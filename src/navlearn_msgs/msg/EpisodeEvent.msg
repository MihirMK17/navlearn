# Per Goal status codes
uint8 START=1
uint8 END=2

# Per Goal result codes
uint8 RESULT_NA=0
uint8 RESULT_SUCCEEDED=1
uint8 RESULT_FAILED=2
uint8 RESULT_CANCELED=3

std_msgs/Header header                      # stamp = event time
builtin_interfaces/Time stamp_received
builtin_interfaces/Time stamp_terminated
builtin_interfaces/Duration nav_time 
unique_identifier_msgs/UUID goal_id      
uint8 state                                 # 1=START, 2=END
uint8 result                                # 0=NA, 1=SUCCEEDED, 2=FAILED, 3=CANCELED
geometry_msgs/PoseStamped start_pose
geometry_msgs/PoseStamped goal_pose

int16 recoveries
uint32 collision_count      # contacts with static/dynamic obstacles during episode; 0 if not measured
float64 optimal_path_m      # shortest Euclidean start-to-goal distance [m]; -1.0 if unknown

# Where the robot actually was when the goal terminated, against where it believed it was.
# Populated only on END events; the flags inside are false on START. See
# TerminalPoseReport.msg for why a SUCCEEDED result is not sufficient evidence of arrival.
TerminalPoseReport terminal