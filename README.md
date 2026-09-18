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


## Citation

If you use this code, please cite:

```
X. Ye and W. Tang, "Stochastic MPC under Heavy-Tailed Disturbances: An
Extreme Value Theory Approach," [venue/year].
```
