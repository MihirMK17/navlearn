std_msgs/Header header
unique_identifier_msgs/UUID goal_id

# Where the robot was teleported to. Yaw is load-bearing, not decorative: destination
# ambiguity is an entropy over (x, y, theta), so a destination recorded without theta
# cannot be scored against the map.
geometry_msgs/Pose kidnap_pose

# Did the teleport actually happen? A sampled-but-unapplied attempt leaves the robot
# where it was, and such a goal carries no perturbation and must not be pooled with
# kidnapped ones.
bool success

unique_identifier_msgs/UUID attempt_id

# Ground-truth pose of the robot immediately BEFORE the teleport, in header.frame_id.
#
# Realised displacement is (kidnap_pose - reference_pose) and is recoverable from
# nothing else. Without it, severity can only be expressed as the nominal commanded
# band, which is a bin label rather than a measurement — and regressing recovery on a
# bin handicaps distance against any competing predictor, biasing the comparison in
# favour of whatever it is being compared to.
geometry_msgs/Pose reference_pose

# False when the pose lookup failed at teleport time. Required because a default Pose is
# the origin, which is legitimate free space on the campaign map: an absent reference
# would otherwise yield a plausible displacement rather than an obviously missing one.
bool reference_available
