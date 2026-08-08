# bumperbot_localization

Localization bringup: local EKF (`robot_localization`) fusing odometry + IMU, the
global AMCL + map_server launch, an IMU republisher, and teaching implementations
(Kalman filter, odometry motion model) from the course lineage. The campaign-tuned
AMCL parameter sets live in `bumperbot_navigation/config/nav2_stack/localizers/`; this
package supplies the launch plumbing that loads whichever one the bringup selects.
