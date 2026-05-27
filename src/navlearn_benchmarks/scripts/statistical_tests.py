#!/usr/bin/env python3
"""Statistical significance tests for NavLearn TTC/TTR success rate comparisons.

Runs Fisher's exact test on pairwise success-rate comparisons across AMCL
configurations and perturbation levels. Reports two-sided p-values and
95% Wilson confidence intervals. Intended to support Paper 1 §6 claims.

Usage:
    python3 statistical_tests.py
    python3 statistical_tests.py --csv results/phase_comparison/ttc_ttr_aggregated.csv
    python3 statistical_tests.py --alpha 0.05
"""

import argparse
import csv
import math
import pathlib
import sys
from typing import Optional, Tuple

REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
DEFAULT_CSV = REPO_ROOT / "results/phase_comparison/ttc_ttr_aggregated.csv"

LEVELS = ["easy", "medium", "hard", "extreme"]


def wilson_ci(successes: int, total: int, alpha: float = 0.05) -> Tuple[float, float]:
    """Wilson score confidence interval for a proportion."""
    from scipy.stats import norm

    if total == 0:
        return (float("nan"), float("nan"))
    z = norm.ppf(1 - alpha / 2)
    p = successes / total
    denom = 1 + z**2 / total
    centre = (p + z**2 / (2 * total)) / denom
    margin = z * math.sqrt(p * (1 - p) / total + z**2 / (4 * total**2)) / denom
    return (max(0.0, centre - margin), min(1.0, centre + margin))


def fisher_test(s1: int, n1: int, s2: int, n2: int) -> Tuple[float, float]:
    """Two-sided Fisher's exact test: returns (odds_ratio, p_value)."""
    from scipy.stats import fisher_exact

    # contingency: [[s1, f1], [s2, f2]]
    table = [[s1, n1 - s1], [s2, n2 - s2]]
    odds_ratio, p_value = fisher_exact(table, alternative="two-sided")
    return odds_ratio, p_value


def load_csv(csv_path: pathlib.Path) -> dict:
    """Load aggregated CSV into data[phase][mode][level] = row."""
    data: dict = {}
    with csv_path.open() as f:
        for row in csv.DictReader(f):
            data.setdefault(row["phase"], {}).setdefault(row["mode"], {})[
                row["level"]
            ] = row
    return data


def fmt_ci(lo: float, hi: float) -> str:
    """Format a confidence interval as a percentage string."""
    if math.isnan(lo):
        return "[---]"
    return f"[{100*lo:.0f}%, {100*hi:.0f}%]"


def fmt_p(p: float, alpha: float) -> str:
    """Format p-value with significance marker."""
    if p < 0.001:
        return "p<0.001 ***"
    if p < 0.01:
        return f"p={p:.3f} **"
    if p < alpha:
        return f"p={p:.3f} *"
    return f"p={p:.3f} ns"


def run_comparison(
    label: str,
    phase_a: str,
    phase_b: str,
    mode: str,
    data: dict,
    alpha: float,
    only_level: Optional[str] = None,
) -> None:
    """Print pairwise Fisher test and CIs for one mode across all levels."""
    da = data.get(phase_a, {}).get(mode, {})
    db = data.get(phase_b, {}).get(mode, {})
    if not da or not db:
        print(f"  [skip] {label}: data missing for {phase_a} or {phase_b}")
        return

    print(f"\n  {label} [{mode.upper()}]")
    print(
        f"  {'Level':8s}  {'PhA%':>5s} 95%CI       {'PhB%':>5s} 95%CI       Δpp   {'':<16s}"
    )
    print(f"  {'-'*8}  {'-'*5} {'-'*12} {'-'*5} {'-'*12} {'-'*4}  {'-'*16}")

    for lv in (LEVELS if only_level is None else [only_level]):
        ra = da.get(lv, {})
        rb = db.get(lv, {})
        if not ra or not rb:
            continue
        try:
            sa, na = int(float(ra["successes"])), int(float(ra["total"]))
            sb, nb = int(float(rb["successes"])), int(float(rb["total"]))
        except (KeyError, ValueError):
            continue
        if na == 0 or nb == 0:
            continue

        pct_a = 100 * sa / na
        pct_b = 100 * sb / nb
        delta = pct_b - pct_a
        ci_a = wilson_ci(sa, na, alpha)
        ci_b = wilson_ci(sb, nb, alpha)
        _, p = fisher_test(sa, na, sb, nb)

        print(
            f"  {lv:8s}  {pct_a:5.0f} {fmt_ci(*ci_a):12s} "
            f"{pct_b:5.0f} {fmt_ci(*ci_b):12s} "
            f"{delta:+4.0f}pp  {fmt_p(p, alpha)}"
        )


def main() -> None:
    """Run all planned comparisons."""
    parser = argparse.ArgumentParser(
        description="NavLearn statistical significance tests"
    )
    parser.add_argument("--csv", type=pathlib.Path, default=DEFAULT_CSV)
    parser.add_argument(
        "--alpha", type=float, default=0.05, help="Significance level (default 0.05)"
    )
    args = parser.parse_args()

    if not args.csv.exists():
        print(
            f"ERROR: {args.csv} not found — run analyze_ttc_ttr.py first",
            file=sys.stderr,
        )
        sys.exit(1)

    data = load_csv(args.csv)
    print(f"NavLearn Statistical Tests  (alpha={args.alpha}, two-sided Fisher's exact)")
    print(f"Phases present: {sorted(data.keys())}")
    print("Legend: *** p<0.001  ** p<0.01  * p<alpha  ns = not significant")

    print("\n=== 1. Phase 1 vs Phase 2 (AMCL tuning effect) ===")
    for mode in ("ttc", "ttr"):
        run_comparison("Ph1 vs Ph2", "phase1", "phase2", mode, data, args.alpha)

    print("\n=== 2. Phase 1 vs Phase 2b (recovery_alpha_fast isolated) ===")
    run_comparison("Ph1 vs Ph2b", "phase1", "phase2b", "ttr", data, args.alpha)

    print("\n=== 3. Phase 2 vs Phase 2b (isolate alpha_fast, hold all else) ===")
    run_comparison("Ph2 vs Ph2b", "phase2", "phase2b", "ttr", data, args.alpha)

    print("\n=== 4. Key claim: Ph2 TTR extreme regression (48% → 16%) ===")
    run_comparison(
        "Ph1 vs Ph2 (extreme only)",
        "phase1",
        "phase2",
        "ttr",
        data,
        args.alpha,
        only_level="extreme",
    )

    print("\n=== 5. Key claim: Ph2b TTR extreme partial recovery (16% → 40%) ===")
    run_comparison(
        "Ph2 vs Ph2b (extreme only)",
        "phase2",
        "phase2b",
        "ttr",
        data,
        args.alpha,
        only_level="extreme",
    )

    print("\n=== 6. TTC ceiling: Ph1 vs Ph2 at extreme (both 52%) ===")
    run_comparison(
        "Ph1 vs Ph2 (TTC extreme)",
        "phase1",
        "phase2",
        "ttc",
        data,
        args.alpha,
        only_level="extreme",
    )
    print()


if __name__ == "__main__":
    main()
