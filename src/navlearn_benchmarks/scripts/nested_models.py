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

"""Nested logistic-model comparison for the Paper 1 secondary claim.

The question
    Kidnap severity is conventionally parameterised by displacement distance. The claim is
    that a map-derived property of the destination predicts recovery better. "Better" is
    made precise by four nested models on the per-goal binary recovery outcome:

        M0  intercept only
        M1  distance
        M2  the candidate predictor
        M3  both

    reported with AIC and BIC across all four, likelihood-ratio tests on nested pairs,
    McFadden pseudo-R-squared, cross-validated AUC, and -- unconditionally -- the
    correlation between the two predictors. The cleanest outcome the claim can have is
    M2 beating M1 while M3 does not beat M2: once the candidate is known, distance adds
    nothing.

Why the fit is hand-rolled
    statsmodels and scikit-learn are not installed on the campaign machine. A library
    earns trust from use; this earns it from `test_nested_models.py`, where every reported
    quantity is checked against a closed form, a textbook identity, or data whose answer
    is known by construction.

Two failure modes it refuses to hide
    Separation -- a predictor that splits the outcome perfectly has no finite maximum
    likelihood, and an optimiser left alone returns whatever it reached when it gave up.
    The enormous coefficient then reads as an overwhelming effect. On a campaign-sized
    sample this is a live risk, so it is detected and surfaced at the top of the report.
    Overfitting -- in-sample fit never worsens when a predictor is added, so AUC is also
    reported cross-validated, which is the number the claim actually rests on.
"""

import math
from dataclasses import dataclass, field

import numpy as np
from scipy import optimize
from scipy.stats import chi2, rankdata, spearmanr

# A coefficient past this magnitude is not an effect size, it is an optimiser running to
# infinity on separable data. Real log-odds effects in this domain are units, not tens.
SEPARATION_COEFFICIENT = 15.0

# Conventional threshold for "meaningfully better" on AIC. Pre-registered for the claim so
# it cannot be relaxed after seeing the table.
DELTA_AIC_THRESHOLD = 2.0


@dataclass
class LogisticFit:
    """A fitted logistic model and everything the comparison table needs from it."""

    coefficients: np.ndarray          # [intercept, slope_1, ...] in the original units
    log_likelihood: float
    n_observations: int
    predictor_names: list = field(default_factory=list)
    separated: bool = False
    converged: bool = True

    @property
    def n_parameters(self):
        """Intercept plus one per predictor."""
        return int(self.coefficients.size)

    @property
    def aic(self):
        """2k - 2LL."""
        return 2.0 * self.n_parameters - 2.0 * self.log_likelihood

    @property
    def bic(self):
        """k ln(n) - 2LL."""
        return self.n_parameters * math.log(self.n_observations) - 2.0 * self.log_likelihood

    def mcfadden_r2(self, null_log_likelihood):
        """1 - LL/LL0. Zero for the null model by construction."""
        if null_log_likelihood == 0.0:
            return float("nan")
        return 1.0 - (self.log_likelihood / null_log_likelihood)

    def as_dict(self, null_log_likelihood=None):
        """Flat summary for JSON output."""
        out = {
            "predictors": list(self.predictor_names),
            "coefficients": [float(c) for c in self.coefficients],
            "log_likelihood": float(self.log_likelihood),
            "n_parameters": self.n_parameters,
            "aic": float(self.aic),
            "bic": float(self.bic),
            "separated": bool(self.separated),
            "converged": bool(self.converged),
        }
        if null_log_likelihood is not None:
            out["mcfadden_r2"] = float(self.mcfadden_r2(null_log_likelihood))
        return out


def _log_likelihood_and_gradient(beta, x_design, y):
    """Negative log-likelihood and its gradient, in a numerically safe form."""
    z = x_design @ beta
    # log(1 + exp(z)) via logaddexp so a large |z| cannot overflow. A naive exp(z) here
    # turns a confidently-fitted observation into inf and the whole fit into nan.
    nll = float(np.sum(np.logaddexp(0.0, z) - y * z))
    p = 1.0 / (1.0 + np.exp(-np.clip(z, -700.0, 700.0)))
    grad = x_design.T @ (p - y)
    return nll, grad


def fit_logistic(x, y, max_iter=500):
    """Maximum-likelihood logistic fit. x is (n, k); k may be 0 for the null model.

    Predictors are standardised internally and the coefficients transformed back, so a
    campaign mixing metres with a unitless likelihood does not hand the optimiser a badly
    conditioned problem, while the reported coefficients stay in the units of the data.
    """
    y = np.asarray(y, dtype=np.float64).ravel()
    x = np.asarray(x, dtype=np.float64)
    if x.ndim == 1:
        x = x.reshape(-1, 1)
    n, k = x.shape[0], (x.shape[1] if x.ndim > 1 else 0)
    if y.size != n:
        raise ValueError(f"x has {n} rows but y has {y.size}")

    centre = x.mean(axis=0) if k else np.zeros(0)
    scale = x.std(axis=0) if k else np.zeros(0)
    # A constant column carries no information; leaving its scale at 1 keeps the column in
    # the design at coefficient zero rather than dividing by zero.
    safe_scale = np.where(scale > 0.0, scale, 1.0)
    x_std = (x - centre) / safe_scale if k else x

    design = np.column_stack([np.ones(n), x_std]) if k else np.ones((n, 1))
    beta0 = np.zeros(design.shape[1])

    result = optimize.minimize(
        _log_likelihood_and_gradient, beta0, args=(design, y),
        jac=True, method="L-BFGS-B", options={"maxiter": max_iter})

    beta_std = result.x
    log_likelihood = -float(result.fun)

    # Back to original units: b = b_std / s, and the intercept absorbs the centring.
    coefficients = np.empty(design.shape[1])
    if k:
        slopes = beta_std[1:] / safe_scale
        slopes = np.where(scale > 0.0, slopes, 0.0)
        coefficients[0] = beta_std[0] - float(np.sum(slopes * centre))
        coefficients[1:] = slopes
    else:
        coefficients[0] = beta_std[0]

    # Separation shows up as a coefficient running away, or as a likelihood that has
    # reached zero because every observation is fitted perfectly.
    separated = bool(
        np.any(np.abs(beta_std) > SEPARATION_COEFFICIENT) or log_likelihood > -1e-6)

    return LogisticFit(
        coefficients=coefficients,
        log_likelihood=log_likelihood,
        n_observations=n,
        separated=separated,
        converged=bool(result.success or result.status == 1),
    )


def predict_proba(fit, x):
    """Fitted success probabilities for new rows."""
    x = np.asarray(x, dtype=np.float64)
    if x.ndim == 1:
        x = x.reshape(-1, 1)
    z = fit.coefficients[0] + (x @ fit.coefficients[1:] if x.shape[1] else 0.0)
    return 1.0 / (1.0 + np.exp(-np.clip(z, -700.0, 700.0)))


def roc_auc(y, scores):
    """Area under the ROC curve, as the Mann-Whitney probability that a positive outranks
    a negative. Ties count a half. NaN when a class is absent, because nothing can be
    ranked and a stand-in 0.5 would read as a measured chance-level result.
    """
    y = np.asarray(y).ravel()
    scores = np.asarray(scores, dtype=np.float64).ravel()
    n_pos = int(np.sum(y == 1))
    n_neg = int(np.sum(y == 0))
    if n_pos == 0 or n_neg == 0:
        return float("nan")
    ranks = rankdata(scores)          # average ranks, which is what makes ties count 0.5
    return float((np.sum(ranks[y == 1]) - n_pos * (n_pos + 1) / 2.0) / (n_pos * n_neg))


def cross_validated_auc(x, y, folds=5, seed=0):
    """Out-of-sample AUC, pooled over stratified folds.

    In-sample fit never worsens when a predictor is added, so an in-sample AUC cannot
    distinguish a real effect from an overfitted one. Stratified so a fold cannot end up
    without one of the classes, which would make its predictions unrankable. Pooled rather
    than averaged per fold: with campaign-sized data a single fold's AUC is very noisy.
    """
    x = np.asarray(x, dtype=np.float64)
    if x.ndim == 1:
        x = x.reshape(-1, 1)
    y = np.asarray(y).ravel()
    n = y.size
    rng = np.random.default_rng(seed)

    assignment = np.empty(n, dtype=int)
    for label in (0, 1):
        idx = np.flatnonzero(y == label)
        rng.shuffle(idx)
        assignment[idx] = np.arange(idx.size) % folds

    predictions = np.full(n, np.nan)
    for fold in range(folds):
        test = assignment == fold
        train = ~test
        if not test.any() or len(np.unique(y[train])) < 2:
            continue
        fit = fit_logistic(x[train], y[train])
        predictions[test] = predict_proba(fit, x[test])

    scored = ~np.isnan(predictions)
    if scored.sum() == 0:
        return float("nan")
    return roc_auc(y[scored], predictions[scored])


def _likelihood_ratio_test(small, large):
    """Chi-square test of the added parameters. Statistic is 2*(LL_large - LL_small)."""
    statistic = 2.0 * (large.log_likelihood - small.log_likelihood)
    df = large.n_parameters - small.n_parameters
    if df <= 0:
        return {"statistic": float(statistic), "df": int(df), "p_value": float("nan")}
    # Clamped at zero: a negative statistic means the optimiser stopped early on the
    # larger model, which is a fit problem, not evidence against the added term.
    statistic = max(statistic, 0.0)
    return {"statistic": float(statistic), "df": int(df),
            "p_value": float(chi2.sf(statistic, df))}


def compare_nested(y, predictors, cv_folds=5, cv_seed=0):
    """Run the M0-M3 comparison.

    `predictors` is an ordered mapping. The first entry is the incumbent (distance); the
    second, if present, is the candidate. All four models are always reported when two
    predictors are given -- dropping one after seeing the table would be a researcher
    degree of freedom the pre-registration exists to remove.
    """
    y = np.asarray(y).ravel()
    names = list(predictors.keys())
    if not names:
        raise ValueError("at least one predictor is required")

    columns = {name: np.asarray(predictors[name], dtype=np.float64).ravel()
               for name in names}
    for name, values in columns.items():
        if values.size != y.size:
            raise ValueError(f"predictor {name!r} has {values.size} rows, y has {y.size}")

    incumbent = names[0]
    candidate = names[1] if len(names) > 1 else None

    models = {}
    null_fit = fit_logistic(np.empty((y.size, 0)), y)
    null_fit.predictor_names = []
    models["M0"] = null_fit

    m1 = fit_logistic(columns[incumbent].reshape(-1, 1), y)
    m1.predictor_names = [incumbent]
    models["M1"] = m1

    if candidate is not None:
        m2 = fit_logistic(columns[candidate].reshape(-1, 1), y)
        m2.predictor_names = [candidate]
        models["M2"] = m2

        both = np.column_stack([columns[incumbent], columns[candidate]])
        m3 = fit_logistic(both, y)
        m3.predictor_names = [incumbent, candidate]
        models["M3"] = m3

    ll0 = null_fit.log_likelihood
    report = {
        "n_observations": int(y.size),
        "n_recovered": int(np.sum(y == 1)),
        "incumbent": incumbent,
        "candidate": candidate,
        "delta_aic_threshold": DELTA_AIC_THRESHOLD,
        "models": {k: v.as_dict(ll0) for k, v in models.items()},
        "tests": {"M0->M1": _likelihood_ratio_test(models["M0"], models["M1"])},
        "any_separated": bool(any(m.separated for m in models.values())),
    }

    report["cross_validated_auc"] = {
        incumbent: cross_validated_auc(
            columns[incumbent].reshape(-1, 1), y, folds=cv_folds, seed=cv_seed),
    }

    if candidate is not None:
        report["tests"]["M0->M2"] = _likelihood_ratio_test(models["M0"], models["M2"])
        report["tests"]["M1->M3"] = _likelihood_ratio_test(models["M1"], models["M3"])
        report["tests"]["M2->M3"] = _likelihood_ratio_test(models["M2"], models["M3"])
        report["cross_validated_auc"][candidate] = cross_validated_auc(
            columns[candidate].reshape(-1, 1), y, folds=cv_folds, seed=cv_seed)

        # Reported whatever it says. Two highly correlated predictors cannot be told
        # apart by this comparison, and surfacing that only when it looks harmless is how
        # a confounded result reaches a table unchallenged.
        rho, rho_p = spearmanr(columns[incumbent], columns[candidate])
        with np.errstate(invalid="ignore"):
            pearson = float(np.corrcoef(columns[incumbent], columns[candidate])[0, 1])
        report["predictor_correlation"] = {
            "pearson": pearson,
            "spearman": float(rho),
            "spearman_p": float(rho_p),
        }

        aic = {k: v.aic for k, v in models.items()}
        beats_incumbent = aic["M2"] < aic["M1"] - DELTA_AIC_THRESHOLD
        adds_nothing = aic["M3"] > aic["M2"] - DELTA_AIC_THRESHOLD
        report["verdict"] = {
            "candidate_beats_incumbent": bool(beats_incumbent),
            "incumbent_adds_nothing_on_top": bool(adds_nothing),
            "ambiguity_subsumes_distance": bool(beats_incumbent and adds_nothing),
            "best_model_by_aic": min(aic, key=aic.get),
            "delta_aic_M1_minus_M2": float(aic["M1"] - aic["M2"]),
        }

    return report
