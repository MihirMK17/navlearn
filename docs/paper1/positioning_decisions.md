# NavLearn Positioning Decisions

## Positioning Statement (Final)
"NavLearn is the first benchmarking framework to integrate localization robustness
metrics (TTC, TTR, ATE, RPE) with navigation efficiency metrics (path length, control
energy, safety) in a single reproducible pipeline. It enables config-driven comparison
of Nav2 algorithm profiles without recompilation."

## Confirmed Metric Additions (Priority Order)
1. Collision count + min clearance (LOW effort, HIGH O-1A value)
2. Full ATE/RPE (stubs → real) (MEDIUM effort, HIGH O-1A value)
3. SPL metric (VERY LOW effort, MEDIUM O-1A value)
4. Statistical depth in aggregation (LOW effort, HIGH O-1A value) ← DONE in Week 0

## navlearn_msgs Changes Needed
- EpisodeEvent.msg: +collision_count (uint32), +optimal_path_m (float64) ← DONE
- TrajectoryMetric.msg: +min_clearance_m (float64) ← DONE

## Conference Venue
- Primary: IEEE RA-L (rolling submission, 6-page limit, 3-4 month review cycle)
- Backup: IROS 2027 (venue TBD, deadline ~March 2027)

## Differentiation vs. Related Work (preliminary)
- BARN: fixed obstacle-density benchmark, no localization metrics, no comparative profile eval
- Arena-Bench: multi-robot, simulation-heavy, no per-episode localization quality measurement
- SLAM eval literature (EVO, rpg_trajectory_evaluation): trajectory only, no nav success/safety metrics
- NavLearn: combines all three pillars in one pipeline (localization quality + nav efficiency + safety)
