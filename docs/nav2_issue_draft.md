# Draft — file at https://github.com/ros-navigation/navigation2/issues (Mihir posts, not Claude)

Title: **SmacPlanner2D: unchecked worldToMap() causes planner_server SIGSEGV when the start pose is outside the costmap (Humble)**

## Bug report

- **Operating System:** Ubuntu 22.04
- **ROS2 Version:** Humble (binaries, `ros-humble-nav2-*` 1.1.18-1jammy)
- **Version or commit hash:** nav2 1.1.18 (current Humble release)
- **DDS implementation:** CycloneDDS

## Steps to reproduce

1. Nav2 with `nav2_smac_planner/SmacPlanner2D` as the global planner, AMCL localization.
2. Displace the robot (or otherwise drive the AMCL estimate) so the estimated pose leaves
   the global costmap bounds — in our case a kidnapped-robot experiment: the estimate
   walked off-map during the recovery attempt.
3. The next `ComputePathToPose` request segfaults `planner_server` (exit code -11); the
   lifecycle manager then reports `CRITICAL FAILURE: SERVER planner_server IS DOWN` and
   tears down the navigation stack.

## Expected behavior

The planner rejects the off-map pose (as newer distros do via
`nav2_core::StartOutsideMapBounds`) and the action aborts, leaving the server up.

## Actual behavior

SIGSEGV in `Costmap2D::getCost` on an out-of-bounds index. From the core dump:

```
#0  nav2_costmap_2d::Costmap2D::getCost (this=0x612456f41020, undex=4293863295)  costmap_2d.cpp:273
#1  nav2_smac_planner::GridCollisionChecker::inCollision                         collision_checker.cpp:176
#2  nav2_smac_planner::Node2D::isNodeValid                                       node_2d.cpp:56
#3  nav2_smac_planner::AStarAlgorithm<Node2D>::areInputsValid                    a_star.cpp:218
#4  nav2_smac_planner::AStarAlgorithm<Node2D>::createPath                        a_star.cpp:235
#5  nav2_smac_planner::SmacPlanner2D::createPlan                                 smac_planner_2d.cpp:255
#6  nav2_planner::PlannerServer::getPlan                                         planner_server.cpp:530
```

Locals in frame 5 (from the same core):

| local | value |
|---|---|
| costmap | `size_x_=500, size_y_=500, resolution_=0.05, origin_x_=-12.5, origin_y_=-12.5` |
| `start.pose.position` | `(2.6226, -15.7264)` — y is 3.2 m outside the map |
| `mx_start, my_start` | `4294967295, 4294965088` — never written |
| `mx_goal, my_goal` | `209, 234` |

## Root cause

`SmacPlanner2D::createPlan` (smac_planner_2d.cpp:214 and :218 in 1.1.18) discards the
return value of `Costmap2D::worldToMap`:

```cpp
unsigned int mx_start, my_start, mx_goal, my_goal;
costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start);
_a_star->setStart(mx_start, my_start, 0);

costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal);
_a_star->setGoal(mx_goal, my_goal, 0);
```

`worldToMap` returns `false` **without writing its output parameters** when the pose is
off-map, so `setStart` receives whatever was on the stack. `areInputsValid` then calls
`getCost` on that index (~4.29e9 against a 250 000-cell array) and the process dies.

An off-map pose estimate is a reachable state in normal operation — any sufficiently bad
localization failure produces one — and a planner crash there takes down the whole
navigation stack via the lifecycle manager, which is a much larger failure than the bad
request.

## Suggested fix

Check both `worldToMap` calls and throw (`nav2_core::PlannerException` in Humble;
matching the `StartOutsideMapBounds`/`GoalOutsideMapBounds` behaviour of newer distros).
`PlannerServer::computePlan` already catches `std::exception` and aborts the action
cleanly. We run this as a local patch:

```cpp
if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
  throw nav2_core::PlannerException(
    "Start pose (" + std::to_string(start.pose.position.x) + ", " +
    std::to_string(start.pose.position.y) + ") is outside the costmap bounds");
}
```

With the patch, the same off-map condition produces
`GridBased plugin failed to plan calculation to (...): "Start pose (...) is outside the
costmap bounds"`, the action aborts, behavior-tree recovery runs, and the server stays up
— observed twice across a 360-goal benchmark campaign that previously died on this crash.

Happy to submit a PR against the humble branch if useful.

<!--
Context for Mihir, not for the issue:
- Full core: /var/crash copy under results/leg2_ttr_rpp forensics + scratchpad crash/ unpack.
- Patched vendored copy: src/third_party/nav2_smac_planner (README there documents provenance).
- After filing: record issue number in src/third_party/README.md "Upstream" section and in
  docs/decisions section 15.
-->
