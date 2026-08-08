# bumperbot_navigation

Nav2 configuration as composable fragments, plus the launch that assembles them and
writes down what it assembled.

## The fragment system (`config/nav2_stack/`)

```
common/       controller_common, planner_common, bt_navigator, behavior_server,
              smoother_server        <- parity lives here, in exactly one file
controllers/  rpp.yaml  dwb.yaml  mppi.yaml
planners/     smac2d.yaml  navfn.yaml  thetastar.yaml
localizers/   amcl_tuned.yaml  amcl_default.yaml
ablations/    high_tolerance.yaml  fixed_bt.yaml  high_vx.yaml
```

ROS 2 merges parameter files in list order with later entries winning, so
`controller_server` receives `[common, controllers/<c>, ablations/<a>?]`. Goal checker
and progress checker exist in one file only: two controller arms cannot drift apart on
`xy_goal_tolerance` unless an ablation deliberately overrides it. That property is
enforced by `navlearn_analysis`' `validate_nav2_stack` gate (run as a test), not by
discipline.

Legal values for each slot come from the filesystem — the launch discovers fragments,
so there is no list in code to fall out of date.

## Provenance

`launch/stack_spec.py` (shared with the bringup by path — it is a launch-dir module)
resolves the selection and writes `~/.navlearn/current_stack_spec.json`: the exact
files, their hashes, and the writing PID. The benchmark harness refuses to run against
a missing or stale spec. "Which configuration produced this number?" is answerable for
every recorded run.
