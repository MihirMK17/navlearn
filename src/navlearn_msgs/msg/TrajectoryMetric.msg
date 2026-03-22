std_msgs/Header header
unique_identifier_msgs/UUID goal_id
builtin_interfaces/Duration duration
uint32 samples
float64 path_length_m        # total distance traveled in this episode

# Placeholders for future GT-based metrics
float64 ate_rmse_m
uint32  ate_count
float64 rpe_delta_s
float64 rpe_trans_rmse_m
uint32  rpe_count
uint8 ref_source
string ref_frame

# Safety metrics
float64 min_clearance_m     # minimum laser scan range observed during episode [m]; -1.0 if not measured
uint32 collision_count      # number of contact events (rising-edge) detected during episode; 0 if not measured