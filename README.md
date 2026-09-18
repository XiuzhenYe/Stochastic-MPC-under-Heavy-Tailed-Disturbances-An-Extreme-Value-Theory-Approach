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

## Figures

All scripts live at the repo root and are run directly with `python
<script>.py` — no `cd` or subfolder needed.

**Figure 1** — `python extremal_index_illustration.py`. Toy AR(1) model
$Z_k = \rho Z_{k-1} + \zeta_k$ with Pareto innovations, for which
$\theta = 1-\rho^\alpha$ in closed form: (a) a realization with exceedances
clustering into episodes, (b) the running-maximum probability compared to
the i.i.d. and dependent-process theoretical curves.

![Figure 1 - extremal index illustration](extremal_index_illustration.png)

*(Cross-check: `python estimate_theta_Y.py` independently validates the same
closed-form $\theta$ on a generic 2×2 rotation-scaling system against the
data-driven Ferro–Segers estimator; prints to the console, no figure.)*

**Figures 2 and 4** — `python plot_figures.py` (reads `comparison_data.npz`,
renders both). Figure 2: representative trajectories and minimum-clearance
histograms for the baseline / Gaussian / naive-EVT controllers on the
unicycle obstacle-avoidance scenario. Figure 4: empirical violation
probability for naive-EVT vs. $\theta$-corrected-EVT against the target
$\epsilon$, plus the safety-margin distribution across all four controllers.

![Figure 2 - scenario and controller comparison](fig1_scenario_comparison.png)

![Figure 4 - theta-correction effect on violation probability and safety margin](fig2_theta_correction.png)

**Figure 3** — `python validate_theta_unicycle.py`. Validates the
closed-form extremal index $\theta_Y$ (Proposition 1) against the
Ferro–Segers estimator, using the closed-loop gain $A_K$ actually derived
from the unicycle scenario above (frozen at the representative operating
point), rather than the generic system used for Figure 1's cross-check.
Prints the comparison to the console; the figure shows a single
illustrative trajectory with exceedance clusters marked. Takes roughly
1–2 minutes (several long simulated trajectories for the cross-check).

![Figure 3 - extremal index validation on the unicycle scenario](theta_unicycle_validation.png)

*(Risk demo: `python demo_theta_corrected_tightening.py` quantifies the
$\theta$-correction's effect on trajectory-level risk at this scenario's
own parameters — naive-vs-corrected episode-level violation probability
and expected consecutive-violation run length — saving
`theta_corrected_tightening_demo.png`.)*

## `main_compare_controllers.py`

This is the script that originally *generated* `comparison_data.npz` (the
data behind Figures 2 and 4) — it's included for transparency and full
reproducibility, but you don't need to run it to get the figures above,
since the data file is already provided.

Four controllers are compared over $M=300$ independent trials:

| Controller | Tightening |
|---|---|
| `baseline` (go-to-goal) | none |
| `gaussian` | assumes Gaussian tube error |
| `evt` | POT/GPD quantile, naive per-step $\epsilon$ |
| `evt_theta` | POT/GPD quantile, $\theta$-corrected $\epsilon_\star = -\ln(1-\epsilon)/(\theta N)$ |

`evt` and `evt_theta` use the identical GPD estimator and MPC solve; the
only difference is the target probability level, using the extremal index
$\theta$ estimated for this scenario. Any difference between them is
therefore attributable to the clustering correction alone.

To rerun the full comparison from scratch (slow — this is the "overnight"
run referenced in the code):

```bash
python main_compare_controllers.py   # overwrites comparison_data.npz
python plot_figures.py
```

Prints "OVERNIGHT settings ... this will take a long time" and checkpoints
progress to `checkpoint.npz` every 10 trials. Random seeds are fixed
throughout, so a full rerun reproduces the included results up to
solver/platform floating-point differences.

`main_linearize_smpc.py` is the linearize-and-resolve (SCP) tube-SMPC
solver used inside the comparison; `main_Monte_Carlo.py` is a standalone
Monte Carlo safety-evaluation utility. `Helper_*.py` cover system setup and
disturbance sampling, the DLQR gain, dynamics/obstacle linearization,
nominal rollout, GPD/EVT quantile estimation, extremal-index estimation,
the tightened-QP solve, and shared type/dataclass definitions.

## Citation

If you use this code, please cite:

```
X. Ye and W. Tang, "Stochastic MPC under Heavy-Tailed Disturbances: An
Extreme Value Theory Approach," [venue/year].
```
