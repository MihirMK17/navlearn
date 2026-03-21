# Gap Analysis: NavLearn vs. Related Benchmarks

## Summary Comparison Table

| Feature | NavLearn | BARN | Arena-Bench | EVO/rpg_traj |
|---------|----------|------|-------------|--------------|
| Nav success metrics (SPL, nav_time) | ✓ | ✓ | ✓ | ✗ |
| Control quality (RMS tracking, energy) | ✓ | ✗ | ✗ | ✗ |
| Localization quality (ATE, RPE) | ✓ (MK2) | ✗ | ✗ | ✓ |
| TTC / TTR (convergence metrics) | ✓ | ✗ | ✗ | ✗ |
| Safety (collision count, clearance) | ✓ (MK2) | ✗ | ✓ | ✗ |
| Config-driven profile comparison | ✓ | ✗ | Partial | ✗ |
| Seeded reproducible goals | ✓ | ✓ | Partial | N/A |
| ROS 2 / Nav2 native | ✓ | ✗ | ✓ | ✗ |
| Single-robot | ✓ | ✓ | Multi | ✓ |
| Open source | ✓ | ✓ | ✓ | ✓ |

## Defensible Claims

(Verify each claim with actual papers before including in paper)

1. **"No existing benchmark integrates localization convergence metrics (TTC, TTR) with
   navigation efficiency metrics in a single pipeline."**
   - Must confirm BARN and Arena-Bench don't include TTC/TTR.

2. **"NavLearn is the first config-driven benchmark for Nav2 profile comparison without
   recompilation."**
   - Must confirm no Nav2-native benchmark paper exists.

3. **"Existing SLAM evaluation tools (EVO, rpg_trajectory_evaluation) measure trajectory
   accuracy but not navigation success, control quality, or safety."**
   - Well-established, easy to defend.

## Potential Overlaps to Frame Carefully

- **Arena-Bench** has collision metrics — NavLearn adds these in MK2. Frame as:
  "Arena-Bench targets multi-robot dynamic environments; NavLearn targets single-robot
  algorithm profile comparison with localization depth."
- **BARN** has reproducibility and difficulty scaling — NavLearn adds localization eval.
  Frame as complementary, different focus.

## Gaps to Populate (Week 2)

- [ ] Read BARN paper (Perille et al., ICRA 2022)
- [ ] Read Arena-Bench paper
- [ ] Read EVO paper (Grupp, 2017)
- [ ] Read rpg_trajectory_evaluation paper
- [ ] Search for Nav2-specific benchmark papers (2022-2025)
- [ ] Search for "convergence time localization" in mobile robotics context
