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