# CLAUDE.md — NavLearn Project Intelligence

> Claude Code reads this file at the start of every session.
> This is the single source of truth for project context, O-1 strategy, and session workflow.
> Last updated: 2026-03-19 (post repo audit).

---

## Who Is Mihir

Mihir is a 26-year-old mechanical engineer (MS, robotics specialization, UCSD 2024)
working at Drov Technologies in Oklahoma City — a 25-person TPMS/trailer tech startup.
He is building toward an O-1A extraordinary ability visa petition. He works on NavLearn
~12-15 hrs/week: evenings (9:30-11:30 PM weekdays) and weekends.

**Key background:**
- MS research: multi-agent robotics lab under Dr. Jorge Cortes at UCSD (Crazyflie drones, VICON)
- Drov: de facto embedded systems lead — BLE firmware, QC/QA frameworks, test automation, 265+ trailers
- Baja SAE: 2nd place nationally as undergraduate team lead (Engine & Transmission sub-team)
- NavLearn: 52 commits, tagged v0.1.0 (MK1), working benchmarking pipeline with real results

## The O-1A Visa — Why Everything Exists

NavLearn is not a hobby project. It is the core vehicle for Mihir's O-1A extraordinary
ability visa petition. Every coding session, every publication, every community interaction
must trace back to: **"does this strengthen the O-1 case?"**

- **O-1A requires**: at least 3 of 8 USCIS evidentiary criteria
- **Target**: satisfy 4 criteria
- **Filing target**: Q1 2027 (March 2027), premium processing
- **Deadline flexibility**: If H-1B is selected, timeline relaxes. If not, June 2027 is hard.
- **O-1 approval rate**: ~94% (FY2025) — the filter is building the case, not approval after filing

**Read `docs/o1/` for full strategy, evidence tracker, recommendation letter guide, and milestones.**

## NavLearn — What It Is

An open-source benchmarking and decision-support framework for mobile robot navigation
stacks, built on ROS 2 Humble and Nav2. It runs standardized scenarios across algorithm
configurations, computes quantified metrics (safety, efficiency, robustness, compute),
and will eventually recommend optimal combinations based on user constraints.

**O-1 criteria it serves:**
- Criterion 5 (Original Contributions): NavLearn itself, if adopted by the community
- Criterion 6 (Scholarly Articles): Papers derived from NavLearn results

## Current Repo State (as of 2026-03-19 audit)

- **52 commits**, July 2025 → March 2026
- **Tagged release**: v0.1.0 = MK1 (Dec 18, 2025)
- **Active branch**: `feat/slam-localization-mk2` (18 modified, 8 untracked files)
- **ROS 2 Humble** on Ubuntu 22.04
- **Gazebo**: Fortress/Garden (ignition-transport11)

### What Works Today
- Full Gazebo sim (small_house, small_warehouse, empty worlds)
- AMCL localization + EKF odometry fusion
- Nav2 navigation (baseline and aggressive profiles)
- Complete benchmarking pipeline: seeded goals → 4 C++ metric nodes → CSV + JSON
- Multi-run harness (10 runs automated via `multi_run_harness.py`)
- Results aggregation (`aggregate_runs.py`)
- scan_sanitizer node
- navlearn_localization_eval nodes (TTC, TTR, ground truth) — functional but UNTRACKED

### MK1 Results (v0.1.0, Dec 2025)
- Aggressive Nav2 profile only, small_house world, 10 runs × 5 goals
- 100% success rate, mean nav time ~22.5s, mean path length ~6.5m
- No baseline comparison — only aggressive tested

### What's Partially Built
- MK2 localization eval (untracked, needs commit + CI integration)
- Kidnap scenario support (infrastructure ready, not exercised)
- ATE/RPE metrics (stubs exist, ground truth infra in localization_eval)
- Custom A*/Dijkstra planners and PD/Pure Pursuit controllers (plugin arch works, not benchmarked against Nav2 defaults)
- bumperbot_utils (lint errors, skipped in CI)

### What's Missing
- Baseline vs. aggressive comparative study (the obvious next benchmark)
- Collision/safety metrics (collision count, min clearance)
- Compute profiling (CPU/mem/latency)
- Statistical depth in aggregation (std dev, percentiles, outliers)
- Unit/integration tests (only ament_lint in CI)
- RL pipeline (no training code, env wrappers, or policy export)
- Bumperbot USD for Isaac Sim (only Turtlebot3 exists)
- Real-robot deployment docs

### Key Architecture Decisions (locked)
- Config-first: Nav2 behavior via YAML profiles, no recompile to switch
- 4-node metric pipeline: episode_manager, control_metric, trajectory_metric, metrics_compiler
- Seeded random goals for reproducibility
- CSV + JSON dual output
- navlearn_msgs as the typed data contract
- Ignition transport for ground truth (not ros-gz-bridge)
- SMAC Planner 2D (global) + RegulatedPurePursuit (controller) for both profiles
- C++17 throughout, gz_ros2_control as git submodule
- Architecture must be sensor-agnostic — current 2D LiDAR constraint is temporary

## Development Plan — Updated Critical Path

**Read `docs/o1/development-plan.md` for the full phased plan.**

### Immediate (this week)
1. Commit 18 modified + 8 untracked files on `feat/slam-localization-mk2`
2. Integrate `navlearn_localization_eval` into CI
3. Merge to main

### MK2 Sprint (next 2-3 sessions)
4. Run baseline vs. aggressive comparative benchmark (same 10×5 setup)
5. Enable kidnap scenario and run
6. Add std dev / percentiles / outlier detection to `aggregate_runs.py`

### Safety & Depth (1-2 sessions after MK2)
7. Add collision count + min clearance metrics
8. Wire ATE/RPE into trajectory_metric using localization_eval ground truth
9. Fix bumperbot_utils lint, remove CI skip

### Paper 1 Readiness (parallel)
10. With MK2 comparative results + safety metrics → draft Paper 1 outline
11. Set up Overleaf with IEEE template

### Deprioritized (not before Paper 1)
- RL integration (Phase 2 — classical comparison is publishable alone)
- Isaac Sim Bumperbot USD alignment
- Custom planner/controller benchmarking (unless they beat Nav2 defaults)
- Multi-robot support

## Technology Stack

| Layer | Choice | Notes |
|-------|--------|-------|
| ROS 2 | Humble Hawksbill | LTS, Ubuntu 22.04 |
| Navigation | Nav2 | Plugin architecture, BT-based |
| Simulation | Gazebo Fortress/Garden | ignition-transport11 |
| RL training (future) | Isaac Sim 4.5.0 + Isaac Lab | AWS EC2, ~$100/mo budget |
| RL framework (future) | Stable Baselines3 / RSL-RL | SB3 prototyping, RSL-RL for scale |
| Languages | C++17 (nodes, plugins) + Python (harness, analysis) |
| Build | colcon + CMake/ament |
| CI | GitHub Actions |
| Docs (future) | Sphinx + ReadTheDocs |
| Papers | Overleaf, IEEE format |

## Hardware

- **Bumperbot**: differential drive, RPLidar A1 (2D, 360°, 12m), MPU-6050 IMU, RPi 4
- **Dev machine**: Ubuntu 22.04, NVIDIA RTX 3050
- **Cloud**: AWS EC2 for Isaac Sim (future)
- **Design principle**: sensor-agnostic architecture. RPLidar A1 is current, not permanent.

## Session Workflow

### Starting Any Session
1. Read this CLAUDE.md (you're doing it now)
2. Check `SESSION_LOG.md` for the last handoff
3. Determine session type: **coding**, **writing**, **strategy**, or **evidence audit**
4. For coding: check current branch, unstaged changes, pick up from handoff
5. For O-1 strategy: read `docs/o1/strategy.md` and `docs/o1/criteria-evidence-tracker.md`

### Ending Any Session
1. Commit all changes with meaningful messages
2. Append to `SESSION_LOG.md`: date, what was done, decisions made, what's next
3. If context is long (25+ exchanges): generate compressed handoff, suggest new chat

### Code Quality Standards
- Every ROS 2 node: docstring with purpose, subscribers, publishers, parameters
- Every function: brief docstring
- YAML params: comments with valid ranges and defaults
- Python: PEP 8. C++: ROS 2 style guide.
- Commits: descriptive ("Add baseline profile benchmark run with kidnap scenario")
- PR-worthy README per package

## Session Type Quick Commands

- `/navlearn` or mention NavLearn → coding session, read this file + development plan
- `/o1` or "visa update" or "O-1 check-in" → strategy session, read docs/o1/
- `/paper` → paper writing session, load current draft state
- `/audit` → evidence audit, walk through all 8 criteria
- `/handoff` → generate session summary and update SESSION_LOG.md

## Key Strategic Principles

1. **Every hour counts** — ~750 total hours across 15 months. Waste nothing.
2. **NavLearn IS the O-1 case** — if NavLearn is strong, criteria 5+6 are satisfied naturally.
3. **Publish early** — an arXiv preprint counts. Don't wait for perfection.
4. **Build in public** — GitHub activity, ROS Discourse, community engagement = evidence.
5. **Dr. Cortes is gold** — give-first reconnection. Share NavLearn progress before asking for anything.
6. **Company transition is strategic, not urgent** — don't leave Drov without a robotics landing spot.
7. **Narrative coherence** — everything answers "how does this advance US robotics/autonomy?"
8. **Attorney early** — consultation Q3-Q4 2026, not at filing time.
9. **Surgical approach** — discuss first, then act. No wasted refactors.
10. **Engineered Cinema stays separate** — do NOT include in O-1 narrative unless autonomous drone nav is published.
