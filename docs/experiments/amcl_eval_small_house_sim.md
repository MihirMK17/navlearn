# AMCL Evaluation — Small House (Sim)

## 0) Snapshot
- Date:
- Repo tag/commit:
- Simulator: Gazebo
- Map: map.yaml / map.pgm (path: ...)
- Scenario: mk1_default.yaml (start + goals frozen)
- Nav2 profile: baseline / aggressive (path: ...)
- AMCL config: amcl.yaml (path: ...)
- Notes: (anything about sim settings, time sync, etc.)

## 1) Pass Criteria)
### Mission outcome
- Success rate: >= __%
- Avg nav time (nominal): <= __ s (optional)
- Recoveries per run: <= __ (optional)

### Quality
- amcl_pose cov_xx: (__ < cov_xx < __)
- amcl_pose cov_yy: (__ < cov_xx < __)
- amcl_pose cov_yawyaw: (__ < cov_xx < __)

### Stability
- map->odom jump events: 0 (jump > __ m or > __ deg)
- max jump magnitude: <= __

### Responsiveness
- TTC (time-to-converge) nominal: <= __ s
- TTC (bad init): <= __ s
- TTR (kidnap): <= __ s

## 2) AMCL Metrics
Mission Outcome:
- success, nav time, recoveries
Localization Quality:
- cov_xx, cov_yy, cov_yaw, trace_xy
Localiation Stability:
- jump_lin, jump_ang,  max map->odom jump_lin, max map->odom jump_ang , jump_count_lin, jump_count_ang
- jump_lin_amcl_pos, jump_ang_amcl_pos, max jump_lin_amcl_pos, max jump_ang_amcl_pos, jump_lin_amcl_pos_count, jump_ang_amcl_pos_count
Responsiveness:
- TTC, TTR
Input Validity:
- scan_rate stats, odom_rate stats, tf stats


## 3) Test Matrix (4 stress tests)
| Test ID | Name         | Initial Pose | Perturbation | Event            | Sensor Stress             | Trials |
|---------|--------------|--------------|--------------|------------------|---------------------------|--------|
| T1      | Nominal      | correct      | none         | none             | none                      | 5      |
| T2      | Bad Init     | offset       | +dx,+dy,+dyaw| none             | none                      | 5      |
| T3      | Kidnap       | correct      | none         | teleport mid-run | none                      | 5      |
| T4      | Sensor Stress| correct      | none         | none             | scan rate reduced / noise | 5      |

## 4) Results Summary (per test)
| Test ID | Success% | TTC_mean(s) | TTR_mean(s) | trace_xy_final_mean | yaw_var_final_mean | jump_count_mean | max_jump_mean |
|---------|----------|-------------|-------------|---------------------|--------------------|-----------------|---------------|
| T1      |          |             |             |                     |                    |                 |               |
| T2      |          |             |   N/A       |                     |                    |                 |               |
| T3      |          |   N/A       |             |                     |                    |                 |               |
| T4      |          |             |   N/A       |                     |                    |                 |               |

## 5) Per-trial log (raw)
| Test | Trial | Success | TTC(s) | TTR(s) | trace_xy_final | yaw_var_final | jump_count | max_jump | Notes |
|------|-------|---------|--------|--------|----------------|---------------|------------|----------|-------|
| T1   | 1     |         |        |        |                |               |            |          |       |

## 6) Failures observed (taxonomy)
- F1: slow convergence (TTC too high)
- F2: localization jump (map->odom discontinuity)
- F3: divergence after kidnap (no recovery)
- F4: scan/odom dropouts causing instability
- F5: nav failures due to localization (planner oscillation / recovery loop)

## 7) Changes made (one change at a time)
| Change ID | File      | Parameter(s) | Old -> New | Hypothesis | Result |
|-----------|-----------|--------------|------------|-----------|---------|
| C1        | amcl.yaml | ...          | ...        | ...       | ...     |

## 8) Conclusion
- What improved?
- What regressed?
- Next change to try (only 1–2 knobs)

