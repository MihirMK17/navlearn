# Paper 1 Outline — NavLearn: A Unified Benchmarking Framework for Mobile Robot Navigation

**Target venue**: IEEE Robotics and Automation Letters (RA-L)
**Target length**: 6 pages (RA-L standard)
**Status**: Outline only — populate after MK2 final benchmark run

---

## I. Introduction (~0.5 page)
- Opening hook: reproducibility gap in robot navigation evaluation
- NavLearn description + positioning statement
- 4 contribution bullets:
  1. Unified pipeline integrating localization robustness + navigation efficiency + safety
  2. Config-driven Nav2 profile comparison (no recompilation)
  3. TTC/TTR as first-class localization convergence metrics
  4. Open-source reproducible benchmark with seeded goals
- Paper organization

## II. Related Work (~1 page)
- Existing navigation benchmarks (BARN, Arena-Bench, Social Force benchmarks)
- SLAM/localization evaluation literature (EVO, rpg_trajectory_evaluation, SLAM Toolbox evals)
- Nav2 evaluation in the literature
- Gap: no unified pipeline combining all three metric pillars

## III. System Architecture (~1.5 pages)
- Config-first design philosophy
- 4-node metric pipeline (episode_manager → control_metric + trajectory_metric → metrics_compiler)
- navlearn_msgs typed data contract
- Localization eval module (TTC, TTR, ATE, RPE)
- Benchmarking harness (seeded goals, multi-run, CSV + JSON output)
- Diagram: Fig. 1 — NavLearn pipeline architecture (Mermaid → TikZ for paper)

## IV. Metrics Definition (~0.75 page)
- Navigation efficiency: nav_time, path_length, success_rate, SPL
- Control quality: tracking_rms_v, tracking_rms_w, saturation_frac, control_energy
- Localization robustness: TTC, TTR, ATE, RPE
- Safety: collision_count, min_clearance_m

## V. Experimental Setup (~0.5 page)
- World: small_house (Gazebo Fortress)
- Robot: Bumperbot (differential drive, RPLidar A1)
- Profiles: baseline vs. aggressive Nav2 configs
- Protocol: 10 runs × 5 seeded goals per profile

## VI. Results (~1 page)
- Table 1: Baseline vs. aggressive — nav/path/energy/success/SPL (mean ± std, p50, p95)
- Table 2: Safety — collision_count, min_clearance_m
- Table 3: Localization — ATE, RPE, TTC
- Table 4: Kidnap recovery — TTR (aggressive only)
- Discussion: trade-offs between profiles

## VII. Conclusion (~0.25 page)
- Summary of contributions
- Limitations (single world, 2D LiDAR, no dynamic obstacles)
- Future work (RL comparison, multi-robot, real-robot validation)

---

## Figures Needed
1. NavLearn pipeline architecture diagram
2. small_house world with goal overlay (screenshot)
3. Optional: representative trajectory plots (baseline vs. aggressive)
