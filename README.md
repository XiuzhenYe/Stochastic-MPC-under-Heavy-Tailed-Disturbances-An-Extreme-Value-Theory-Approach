# Stochastic MPC under Heavy-Tailed Disturbances: An Extreme Value Theory Approach

Code accompanying the paper *"Stochastic MPC under Heavy-Tailed Disturbances:
An Extreme Value Theory Approach"* (X. Ye and W. Tang). This repository
reproduces all four figures in the manuscript.

## Summary

The paper develops a tube-based stochastic MPC (SMPC) formulation for linear
systems under heavy-tailed, regularly varying disturbances. Rather than
tightening the chance constraint using an assumed distribution (e.g.
Gaussian) or a moment bound, the tube error's tail is characterized directly
via Extreme Value Theory (EVT): a Generalized Pareto Distribution (GPD) is
fit to the peaks-over-threshold (POT) of Monte Carlo samples of the
closed-loop error, giving an asymptotically exact quantile estimate as the
target violation probability $\epsilon \to 0$. The paper further shows that
closed-loop dynamics induce temporal clustering of rare excursions across
the prediction horizon, characterized by Leadbetter's extremal index
$\theta \in (0,1]$, estimable in closed form from the closed-loop system
matrices. The resulting $\theta$-corrected constraint targets
$\epsilon_\star = -\ln(1-\epsilon)/(\theta N)$ instead of the raw per-step
$\epsilon$, bounding the probability of a rare-event episode over the
horizon rather than only the marginal per-step exceedance probability. The
approach is validated on a nonlinear unicycle navigating past an obstacle
under Student-$t$ disturbances.

## Install

```bash
pip install -r requirements.txt
```

`numpy`, `scipy`, `matplotlib`, and `cvxpy` (with its bundled OSQP solver)
cover everything below; the MPC solve (Figures 2 and 4) is the only part
that needs `scipy`/`cvxpy`.

## The four controllers

| Controller | Tightening |
|---|---|
| `baseline` (go-to-goal) | none |
| `gaussian` | assumes Gaussian tube error |
| `evt` | POT/GPD quantile, naive per-step $\epsilon$ |
| `evt_theta` | POT/GPD quantile, θ-corrected $\epsilon_\star = -\ln(1-\epsilon)/(\theta N)$ |

`evt` and `evt_theta` use the *same* GPD tail estimator and the *same*
tube-MPC solve at every step — the only difference is the target
probability level passed in, using this scenario's own validated θ. Any
difference in outcome between them is therefore attributable to the
clustering correction alone, not to a different estimator or solver.

## Install

```bash
pip install -r requirements.txt
```

## Reproduce the figures above without rerunning the simulation

```bash
python plot_figures.py
```

Reads `data/comparison_data.npz` (the exact data behind the figures above —
$M=300$ trials, fixed seeds) and renders `fig1_scenario_comparison.{png,pdf}`
and `fig2_theta_correction.{png,pdf}` into `figures/`.

## Rerun the full comparison from scratch (slow)

```bash
python main_compare_controllers.py   # overwrites data/comparison_data.npz
python plot_figures.py
```

This reruns all $M=300$ trials for all four controllers, re-estimating θ
for the scenario and refitting the GPD tail from fresh Monte Carlo samples
at every MPC re-solve.

## Contents

- `main_compare_controllers.py` — runs the $M=300$-trial comparison above.
- `main_linearize_smpc.py` — the linearize-and-resolve (SCP) tube-SMPC
  solver used inside the comparison.
- `main_Monte_Carlo.py` — standalone Monte Carlo safety-evaluation utility.
- `plot_figures.py` — renders the two figures above from saved data.
- `Helper_*.py` — system setup and disturbance sampling, DLQR gain,
  dynamics/obstacle linearization, nominal rollout, GPD/EVT quantile
  estimation (`pot_gpd_quantile`), **extremal-index estimation for this
  scenario** (`Helper_extremal_index.py` — computes this run's θ̂ and
  $\epsilon_\star$), the tightened-QP solve, and shared type/dataclass
  definitions.



## Figures

**Figure 1 — extremal index illustration.** A realization of the AR(1)
toy model showing exceedances clustering into episodes, and the
running-maximum probability compared to the i.i.d. and dependent-process
theoretical curves.

<img src="extremal_index_illustration.png" width="600" alt="Extremal index illustration: exceedance clustering and running-maximum probability">

**Figure 2 — scenario and naive-tightening comparison.** Representative
trajectories and minimum-clearance histograms for the baseline / Gaussian /
naive-EVT controllers on the unicycle obstacle-avoidance scenario.

<img src="fig1_scenario_comparison.png" width="600" alt="Scenario trajectories and minimum-clearance histograms for baseline, Gaussian, and naive-EVT controllers">

**Figure 3 — extremal index validation on the unicycle scenario.** An
illustrative trajectory with exceedance clusters marked, using the
closed-loop gain actually derived from the unicycle scenario.

<img src="theta_unicycle_validation.png" width="400" alt="Illustrative trajectory with exceedance clusters marked, unicycle-derived closed-loop gain">

**Figure 4 — effect of the θ-correction.** Empirical violation probability
for naive-EVT vs. θ-corrected-EVT against the target $\epsilon$, and
safety-margin distribution across all four controllers.

<img src="fig2_theta_correction.png" width="600" alt="Empirical violation probability for naive-EVT vs theta-corrected-EVT, and safety-margin distribution across all four controllers">

 
 
## Citation

If you use this code, please cite:

```
X. Ye and W. Tang, "Stochastic MPC under Heavy-Tailed Disturbances: An
Extreme Value Theory Approach," [venue/year].
```
