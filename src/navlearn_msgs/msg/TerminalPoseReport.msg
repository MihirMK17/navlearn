# Terminal pose report — captured once, at the instant a navigation goal terminates.
#
# WHY THIS EXISTS
#
# Nav2 reports SUCCEEDED when its goal checker is satisfied, and the goal checker is
# evaluated against the localization *estimate*. Under localization perturbation the
# estimate can be converged on the goal while the robot is physically somewhere else.
# The retired campaign had no record of this: Table V binned results on the AMCL estimate
# rather than ground truth, and with a measured 0.94 m mean drift against a 0.28 m bin
# threshold the binning was not recoverable after the fact.
#
# This message is the fix. Every terminated goal carries both poses and the distance
# between them, so a reported success can be checked against physical reality.
#
# CONVENTIONS
#
# All poses are in the fixed frame (normally `map`). Every distance is metres, every
# angle radians. A scalar that could not be computed is -1.0, never 0.0 — zero is a
# legitimate and highly significant value here, so it must not double as "missing".
# Check the availability flags before using any scalar.

# --- availability -----------------------------------------------------------------
bool gt_available              # ground-truth transform resolved at termination
bool estimate_available        # a localization estimate was available at termination

# --- poses ------------------------------------------------------------------------
geometry_msgs/PoseStamped true_pose        # ground truth, from the simulator
geometry_msgs/PoseStamped estimated_pose   # what the robot believed, from AMCL

# --- the headline numbers ---------------------------------------------------------
float64 true_distance_to_goal_m       # |ground truth - goal|. THE number: how far the
                                      # robot actually finished from where it was sent.
float64 estimated_distance_to_goal_m  # |estimate - goal|. What Nav2 judged success on.
float64 localization_error_m          # |ground truth - estimate|. The gap between them;
                                      # a large value on a SUCCEEDED goal is a false success.
float64 true_yaw_error_rad            # |yaw(ground truth) - yaw(goal)|, wrapped to [0, pi]

# --- filter state -----------------------------------------------------------------
# Position covariance diagonal from /amcl_pose. A filter can be confidently wrong, so
# these are recorded alongside the error rather than as a substitute for it.
float64 covariance_xx                 # m^2
float64 covariance_yy                 # m^2
float64 covariance_yaw                # rad^2
bool filter_converged                 # covariance_xx + covariance_yy <= convergence_threshold_m2
float64 convergence_threshold_m2      # the threshold actually applied, recorded so that
                                      # analysis never has to assume which value was used

# --- staleness --------------------------------------------------------------------
# AMCL publishes only when it updates (update_min_d, update_min_a), and the robot is
# stationary at termination, so the newest estimate may predate the stop. Recording its
# age makes that visible instead of leaving it to be assumed away.
float64 estimate_age_sec              # termination time minus estimate stamp; -1.0 if unknown
