# # Per-goal, authoritative metric emitted once a goal terminates (success/abort/cancel/timeout/preempt/reject).
# # All times are ROS time; durations are computed by the proxy.

# # ---------- Status enum (uint8 constants) ----------
# uint8 STATUS_UNKNOWN   = 0
# uint8 STATUS_SUCCEEDED = 1
# uint8 STATUS_ABORTED   = 2
# uint8 STATUS_CANCELED  = 3
# uint8 STATUS_PREEMPTED = 4
# uint8 STATUS_TIMEOUT   = 5
# uint8 STATUS_REJECTED  = 6

# # ---------- Identification ----------
# string robot_namespace                          # e.g., /robot1
# unique_identifier_msgs/UUID goal_id_downstream  # UUID assigned by the proxy’s action server
# unique_identifier_msgs/UUID goal_id_upstream    # UUID of upstream Nav2 handle (zero UUID if never accepted)

# # ---------- Poses ----------
# geometry_msgs/PoseStamped goal_pose             # pose as received by the proxy (frame_id should equal map_frame)
# geometry_msgs/PoseStamped start_pose            # map->base_link sampled at acceptance (or nearest available)

# # ---------- Timestamps (ROS time) ----------
# builtin_interfaces/Time stamp_received          # when proxy server received the goal
# builtin_interfaces/Time stamp_accepted          # when upstream accepted (zero time if REJECTED)
# builtin_interfaces/Time stamp_terminated        # when final result observed / classified

# # ---------- Derived duration ----------
# builtin_interfaces/Duration nav_time            # stamp_terminated - stamp_accepted (zero if never accepted)

# # ---------- Outcome ----------
# uint8  status                                   # one of STATUS_*
# string status_reason                            # free-form tags, e.g., "[upstream_rejected],[clock_paused]"

# # ---------- Extra telemetry (best-effort) ----------
# uint32 recoveries_count                         # last known count from feedback (0 if unknown)
# float32 distance_remaining_final                # meters; last feedback value (NaN if unknown)

# # ---------- Flags ----------
# bool start_pose_time_fallback                   # true if TF at accept-time failed and we used latest instead

uint8 GOAL_SUCCEEDED=1
uint8 GOAL_FAILED=2
uint8 GOAL_CANCELED=3

std_msgs/Header header
unique_identifier_msgs/UUID episode_id
builtin_interfaces/Duration nav_time
uint32 recoveries_count
uint8 goal_status                           # 1=SUCCEEDED, 2=FAILED, 3=CANCELED
geometry_msgs/PoseStamped start_pose
geometry_msgs/PoseStamped goal_pose