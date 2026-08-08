#!/usr/bin/env python3
# Copyright 2026 Mihir Kulkarni
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the nested logistic-model comparison behind the secondary claim.

Why this is hand-rolled and therefore heavily tested
    statsmodels and scikit-learn are not installed on the campaign machine, so the
    logistic fit is written on scipy.optimize. A regression library earns trust from use;
    this one has to earn it from tests. Every number the paper's model-comparison table
    would print is checked here against a closed form, a textbook identity, or a dataset
    whose answer is known by construction.

What the claim needs it to get right
    The comparison is M0 intercept, M1 distance, M2 the candidate predictor, M3 both.
    Reported: AIC and BIC across all four, likelihood-ratio tests on nested pairs,
    McFadden pseudo-R-squared, and cross-validated AUC. If the fit is wrong, every one of
    those is wrong in the same direction and nothing downstream would notice.
"""

import math
import os
import sys

import pytest

np = pytest.importorskip("numpy", reason="numpy not available")
pytest.importorskip("scipy", reason="scipy not available")

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))

from nested_models import (  # noqa: E402
    compare_nested,
    cross_validated_auc,
    fit_logistic,
    roc_auc,
)


def _separable_free_data(n=400, seed=7):
    """A predictor with a real but non-separating effect on the outcome."""
    rng = np.random.default_rng(seed)
    x = rng.normal(size=n)
    p = 1.0 / (1.0 + np.exp(-(0.5 + 1.5 * x)))
    y = (rng.random(n) < p).astype(int)
    return x.reshape(-1, 1), y


# ------------------------------------------------------------------ the fit itself


def test_intercept_only_model_recovers_the_log_odds_of_the_base_rate():
    """Closed form: with no predictors the MLE intercept is log(k / (n - k)).

    Anything else means the optimiser is not finding the maximum, and every likelihood
    derived from it would be wrong by the same amount.
    """
    y = np.array([1] * 30 + [0] * 70)
    x = np.empty((100, 0))
    fit = fit_logistic(x, y)
    assert fit.coefficients[0] == pytest.approx(math.log(30 / 70), abs=1e-6)


def test_log_likelihood_of_the_intercept_model_matches_the_closed_form():
    """n*[p log p + (1-p) log(1-p)] for the base rate p."""
    y = np.array([1] * 30 + [0] * 70)
    fit = fit_logistic(np.empty((100, 0)), y)
    p = 0.3
    expected = 100 * (p * math.log(p) + (1 - p) * math.log(1 - p))
    assert fit.log_likelihood == pytest.approx(expected, abs=1e-6)


def test_perfect_predictor_is_detected_rather_than_silently_diverging():
    """Separable data has no finite MLE; coefficients run to infinity.

    Left alone the optimiser returns whatever it reached when it gave up, and the huge
    coefficient reads as an overwhelmingly strong effect. That is the classic way a
    logistic regression lies, and on a small campaign sample separation is a live risk.
    """
    x = np.array([[-3.0], [-2.0], [-1.0], [1.0], [2.0], [3.0]])
    y = np.array([0, 0, 0, 1, 1, 1])
    fit = fit_logistic(x, y)
    assert fit.separated is True


def test_slope_sign_follows_the_direction_of_the_effect():
    """A predictor that raises the outcome probability must get a positive coefficient."""
    x, y = _separable_free_data()
    fit = fit_logistic(x, y)
    assert fit.separated is False
    assert fit.coefficients[1] > 0.0


def test_recovers_a_known_coefficient_within_sampling_error():
    """Data generated with a slope of 1.5 must fit near 1.5 at n=4000."""
    rng = np.random.default_rng(11)
    x = rng.normal(size=4000)
    p = 1.0 / (1.0 + np.exp(-(0.5 + 1.5 * x)))
    y = (rng.random(4000) < p).astype(int)
    fit = fit_logistic(x.reshape(-1, 1), y)
    assert fit.coefficients[0] == pytest.approx(0.5, abs=0.15)
    assert fit.coefficients[1] == pytest.approx(1.5, abs=0.20)


def test_adding_a_predictor_cannot_lower_the_maximised_log_likelihood():
    """A nested model's fit can only improve; a drop means the optimiser is stopping early.

    Every likelihood-ratio test in the comparison assumes this, and a violation would
    produce a negative test statistic that no downstream check looks for.
    """
    rng = np.random.default_rng(3)
    x1 = rng.normal(size=300)
    noise = rng.normal(size=300)
    y = (rng.random(300) < 1 / (1 + np.exp(-(0.2 + 1.0 * x1)))).astype(int)
    small = fit_logistic(x1.reshape(-1, 1), y)
    large = fit_logistic(np.column_stack([x1, noise]), y)
    assert large.log_likelihood >= small.log_likelihood - 1e-8


# ------------------------------------------------------------ information criteria


def test_aic_and_bic_use_the_documented_formulas():
    """AIC = 2k - 2LL, BIC = k ln(n) - 2LL. Off-by-one in k flips model rankings."""
    x, y = _separable_free_data()
    fit = fit_logistic(x, y)
    k, n = 2, len(y)
    assert fit.n_parameters == k
    assert fit.aic == pytest.approx(2 * k - 2 * fit.log_likelihood, abs=1e-9)
    assert fit.bic == pytest.approx(k * math.log(n) - 2 * fit.log_likelihood, abs=1e-9)


def test_mcfadden_r2_is_zero_for_the_null_model_and_positive_for_a_real_one():
    """Pseudo-R-squared is 1 - LL/LL0; the null model must score exactly zero."""
    x, y = _separable_free_data()
    null = fit_logistic(np.empty((len(y), 0)), y)
    real = fit_logistic(x, y)
    assert null.mcfadden_r2(null.log_likelihood) == pytest.approx(0.0, abs=1e-12)
    assert 0.0 < real.mcfadden_r2(null.log_likelihood) < 1.0


# ---------------------------------------------------------------------- AUC


def test_auc_of_a_perfect_ranking_is_one_and_a_reversed_one_is_zero():
    """Endpoints pin the orientation; a flipped AUC would invert every conclusion."""
    y = np.array([0, 0, 1, 1])
    assert roc_auc(y, np.array([0.1, 0.2, 0.8, 0.9])) == pytest.approx(1.0)
    assert roc_auc(y, np.array([0.9, 0.8, 0.2, 0.1])) == pytest.approx(0.0)


def test_auc_of_an_uninformative_score_is_one_half():
    """Constant scores mean every pair is a tie, which is chance by definition."""
    y = np.array([0, 1, 0, 1])
    assert roc_auc(y, np.array([0.5, 0.5, 0.5, 0.5])) == pytest.approx(0.5)


def test_auc_equals_the_mann_whitney_statistic():
    """Textbook identity: AUC is the probability a positive outranks a negative.

    Computed here by brute force over all pairs, so the vectorised implementation is
    checked against the definition rather than against itself.
    """
    rng = np.random.default_rng(5)
    y = rng.integers(0, 2, size=60)
    scores = rng.normal(size=60)
    pos, neg = scores[y == 1], scores[y == 0]
    wins = sum((1.0 if a > b else 0.5 if a == b else 0.0) for a in pos for b in neg)
    assert roc_auc(y, scores) == pytest.approx(wins / (len(pos) * len(neg)), abs=1e-12)


def test_auc_is_undefined_when_one_class_is_absent():
    """All-successes or all-failures cannot rank anything; NaN, not a made-up 0.5."""
    assert math.isnan(roc_auc(np.array([1, 1, 1]), np.array([0.1, 0.5, 0.9])))


def test_cross_validated_auc_is_deterministic_for_a_fixed_seed():
    """A campaign number that changes between runs of the same data is not a result."""
    x, y = _separable_free_data()
    first = cross_validated_auc(x, y, folds=5, seed=42)
    second = cross_validated_auc(x, y, folds=5, seed=42)
    assert first == second


def test_cross_validated_auc_beats_chance_for_a_real_predictor():
    """Out-of-sample, not in-sample: the number the claim actually rests on."""
    x, y = _separable_free_data()
    assert cross_validated_auc(x, y, folds=5, seed=42) > 0.65


def test_cross_validated_auc_of_pure_noise_is_near_chance():
    """A predictor with no signal must not score well out of sample.

    In-sample fit always improves with any predictor; this is the check that
    distinguishes a real effect from an overfitted one.
    """
    rng = np.random.default_rng(9)
    y = rng.integers(0, 2, size=400)
    noise = rng.normal(size=400).reshape(-1, 1)
    assert cross_validated_auc(noise, y, folds=5, seed=1) == pytest.approx(0.5, abs=0.12)


# --------------------------------------------------------------- nested comparison


def test_likelihood_ratio_test_recovers_a_known_effect():
    """M1 over M0 with a genuine predictor must reject the null."""
    x, y = _separable_free_data(n=600)
    result = compare_nested(y, {"distance": x[:, 0]})
    lrt = result["tests"]["M0->M1"]
    assert lrt["df"] == 1
    assert lrt["p_value"] < 0.01


def test_likelihood_ratio_test_is_calibrated_under_the_null():
    """Over many null datasets, the test must reject at about its nominal rate.

    Asserting that one noise dataset yields p > 0.05 is not a test of anything: under the
    null, p is uniform, so such a check fails one time in twenty by construction. (It did,
    on the first seed tried, at p = 0.008 -- a genuine 1-in-130 draw, not a defect.) The
    property worth pinning is calibration: the false-positive rate at alpha, over many
    datasets, must land near alpha. A miscalibrated test is how a null finding gets
    reported as a real one, and it would not show up in any single-dataset check.

    The band is wide because 200 replicates put a binomial standard error of about 1.5
    points on a 5% rate; it is still narrow enough to catch a test that is broken rather
    than merely noisy.
    """
    p_values = []
    for seed in range(200):
        rng = np.random.default_rng(seed)
        y = rng.integers(0, 2, size=300)
        noise = rng.normal(size=300)
        p_values.append(compare_nested(y, {"distance": noise})["tests"]["M0->M1"]["p_value"])

    false_positive_rate = float(np.mean(np.array(p_values) < 0.05))
    assert 0.01 <= false_positive_rate <= 0.12, (
        f"false-positive rate {false_positive_rate:.3f} is not near the nominal 0.05")


def test_the_stronger_predictor_wins_on_aic():
    """The headline arrangement: predictor beats distance, and adding distance adds nothing.

    Built so the answer is known: the outcome is generated from `ambiguity` alone, and
    `distance` is pure noise. M2 must beat M1 on AIC, and M3 must not meaningfully beat
    M2 -- a delta-AIC under 2, which is the conventional 'no better' threshold and the one
    pre-registered for the claim.
    """
    rng = np.random.default_rng(17)
    n = 800
    ambiguity = rng.normal(size=n)
    distance = rng.normal(size=n)
    y = (rng.random(n) < 1 / (1 + np.exp(-(0.3 + 1.8 * ambiguity)))).astype(int)

    result = compare_nested(y, {"distance": distance, "ambiguity": ambiguity})
    aic = {k: v["aic"] for k, v in result["models"].items()}

    assert aic["M2"] < aic["M1"], "the generating predictor did not beat the noise one"
    assert aic["M3"] > aic["M2"] - 2.0, "adding a noise predictor appeared to help"
    assert result["verdict"]["ambiguity_subsumes_distance"] is True


def test_verdict_is_false_when_distance_carries_the_signal():
    """The mirror case must come out the other way, or the verdict means nothing."""
    rng = np.random.default_rng(19)
    n = 800
    distance = rng.normal(size=n)
    ambiguity = rng.normal(size=n)
    y = (rng.random(n) < 1 / (1 + np.exp(-(0.3 + 1.8 * distance)))).astype(int)

    result = compare_nested(y, {"distance": distance, "ambiguity": ambiguity})
    assert result["verdict"]["ambiguity_subsumes_distance"] is False


def test_comparison_reports_the_predictor_correlation_unconditionally():
    """Two correlated predictors muddy the comparison, so the correlation is never optional.

    Reporting it only when it looks harmless is how a confounded comparison reaches a
    table unchallenged.
    """
    rng = np.random.default_rng(23)
    base = rng.normal(size=300)
    result = compare_nested(
        rng.integers(0, 2, size=300),
        {"distance": base, "ambiguity": base * 0.9 + rng.normal(size=300) * 0.1})
    assert result["predictor_correlation"]["pearson"] > 0.9
    assert "spearman" in result["predictor_correlation"]


def test_separation_is_surfaced_in_the_comparison_not_buried_in_a_model():
    """If any model separated, the whole comparison is untrustworthy and must say so."""
    x = np.array([-3.0, -2.0, -1.0, 1.0, 2.0, 3.0])
    y = np.array([0, 0, 0, 1, 1, 1])
    result = compare_nested(y, {"distance": x, "ambiguity": np.zeros(6)})
    assert result["any_separated"] is True


def test_all_four_models_are_reported_even_when_one_is_useless():
    """The table is fixed at M0-M3; dropping a model post hoc is a researcher degree of freedom."""
    x, y = _separable_free_data()
    result = compare_nested(y, {"distance": x[:, 0], "ambiguity": np.zeros(len(y))})
    assert set(result["models"]) == {"M0", "M1", "M2", "M3"}
    for spec in result["models"].values():
        assert "aic" in spec and "bic" in spec and "log_likelihood" in spec
