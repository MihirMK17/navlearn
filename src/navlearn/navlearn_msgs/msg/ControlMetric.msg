std_msgs/Header header                  # stamp ~ episode end; frame_id = "base_link"
unique_identifier_msgs/UUID goal_id
builtin_interfaces/Duration duration    # Duration of aggregation episode 
uint32 samples                          # Number of time synchronized samples per episode

# Tracking error
float64 tracking_rms_v
float64 tracking_rms_w
float64 saturation_frac_v
float64 saturation_frac_w

# Longitudinal Wheel Slip Statistics
float64 slip_mean                       # mean s over window
float64 slip_std                        # std(s) over window
float64 slip_p95                        # 95th percentile of |s|

# Control Energy
float64 control_energy

# Configuration Snapshot
float64 v_max                           # m/s used for saturation calc
float64 w_max                           # rad/s used for saturation calc
float64 epsilon_v                       # m/s used in slip denom
float64 epsilon_w                       # rad/s used in angular slip