# navlearn_msgs

The typed data contract between every NavLearn node. Interfaces are frozen: the
collected campaign dataset serializes them into rosbags, so a field change is a
comparability break, not a refactor.

| interface | produced by | consumed by |
|---|---|---|
| `EpisodeEvent.msg` | episode_manager | control_metric, trajectory_metric, metrics_compiler, localization_metrics, offline recompute |
| `KidnapEvent.msg` | episode_manager | metrics_compiler, localization_metrics, offline recompute |
| `ControlMetric.msg` | control_metric | metrics_compiler |
| `TrajectoryMetric.msg` | trajectory_metric | metrics_compiler |
| `TerminalPoseReport.msg` | episode_manager (nested in `EpisodeEvent`) | metrics_compiler — carries the ground-truth pose at episode end, the paper's headline measurement |
| `srv/SetEntityPose.srv` | served by gz_set_pose_server | called by episode_manager for the kidnap teleport |
