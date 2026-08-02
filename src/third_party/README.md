# third_party

Upstream packages vendored into this workspace because a defect in them blocks the
campaign and the fix cannot wait for a distro release. Everything here is a verbatim copy
of an upstream release with a named, minimal patch on top.

Rules for this directory:

- **Pin the origin.** Record the repository, tag and commit the copy was taken from.
- **Keep the diff minimal and auditable.** Copy the release unmodified, then patch. Do not
  reformat, do not tidy, do not drop files that are merely unused — a large diff makes it
  impossible to argue about what the patch actually changes.
- **Every patch carries a regression test** that fails without it.
- **Every patch is reported upstream.** Vendoring is how we keep running, not a fork.
- **Record it in the stack spec.** A result produced against a patched stack that does not
  say so is a result nobody can reproduce.

---

## nav2_smac_planner

| | |
|---|---|
| Origin | https://github.com/ros-navigation/navigation2 |
| Tag | `1.1.18` (newest Humble release; the version this workspace had installed) |
| Taken | 2026-08-01 |
| Replaces | `ros-humble-nav2-smac-planner` 1.1.18-1jammy |

### Why

`planner_server` took SIGSEGV during leg 2 of the Paper 1 campaign on 2026-08-01, 12
minutes into a 24-episode cell, and took the navigation stack down with it. From the core
dump (`/var/crash`, 192 MB, `Sat Aug 1 08:33:10 2026`):

```
#0  nav2_costmap_2d::Costmap2D::getCost (this=0x612456f41020, undex=4293863295)  costmap_2d.cpp:273
#1  nav2_smac_planner::GridCollisionChecker::inCollision                         collision_checker.cpp:176
#2  nav2_smac_planner::Node2D::isNodeValid                                       node_2d.cpp:56
#3  nav2_smac_planner::AStarAlgorithm<Node2D>::areInputsValid                    a_star.cpp:218
#4  nav2_smac_planner::AStarAlgorithm<Node2D>::createPath                        a_star.cpp:235
#5  nav2_smac_planner::SmacPlanner2D::createPlan                                 smac_planner_2d.cpp:255
```

Frame 5 locals:

| local | value |
|---|---|
| costmap | `size_x_=500 size_y_=500 resolution_=0.05 origin_x_=-12.5 origin_y_=-12.5` |
| `start.pose.position` | `(2.6226, -15.7264)` — y is 3.23 m outside the map |
| `mx_start, my_start` | `4294967295, 4294965088` — never written |
| `mx_goal, my_goal` | `209, 234` — in bounds |

`createPlan` called `worldToMap` and discarded its return value
(`smac_planner_2d.cpp:214` and `:218`). `worldToMap` returns false without writing its
output parameters when the pose is off-map, so `setStart` received uninitialised stack
values and A* indexed 4 293 863 295 into a 250 000-cell array.

A pose estimate outside the map is a **normal outcome of a kidnap**, which is why this only
ever fired in TTR cells: the TTC leg perturbs an estimate by a bounded offset from a valid
pose and drove the same planner through 360 goals without incident.

Later distros already check this and raise `nav2_core::StartOutsideMapBounds`. Humble does
not, and 1.1.18 is the newest Humble release, so there is nothing to upgrade to.

### The patch

`src/smac_planner_2d.cpp` — check both `worldToMap` calls and throw
`nav2_core::PlannerException` (the Humble-era exception; `StartOutsideMapBounds` does not
exist in this distro). `PlannerServer::computePlan` already catches `std::exception` and
aborts the action, so the behaviour tree runs its recovery and the episode continues.

`test/test_smac_2d.cpp` — `test_smac_2d_start_outside_map_bounds_is_refused_not_indexed`.
Fails by killing the test binary if the check is removed.

### Why this does not invalidate previously collected data

The check can only alter behaviour on a path that currently ends in an out-of-bounds read.
Any run that did not crash never reached it, so the patch is inert on all of leg 1. That
property is the reason this was preferred over changing what the TTR experiment measures.

### Upstream

To be filed against `ros-navigation/navigation2`. Not yet submitted.
