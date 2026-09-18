"""
Validate the closed-form extremal index theta_Y (Proposition 1) against a
data-driven estimate obtained from simulated trajectories of the closed-loop
projection Y_k, using the Ferro-Segers (2003) intervals estimator.

Model
-----
    x_{k+1} = A_K x_k + w_k ,      w_k = zeta_k * b ,      Y_k = c^T x_k

A_K is a 2x2 rotation-scaling matrix with spectral radius rho and rotation
angle phi, so the impulse response g_j = c^T A_K^j b decays geometrically
while changing sign. zeta_k are i.i.d. Student-t innovations with alpha
degrees of freedom (regularly varying tail with index alpha), symmetric so
the tail-balance weights are p = q = 0.5.

Two independent computations of theta_Y are produced:
  1. Closed form (Proposition 1), from the model matrices alone, no data.
  2. Empirical estimate (Ferro-Segers intervals estimator), averaged over
     several long, independent simulated trajectories of Y_k, using no
     knowledge of the model.

Their agreement is the validation. Estimates are averaged over independent
replicate trajectories (rather than read off a single run) because the
intervals estimator is noisy at the high thresholds needed for convergence,
a single trajectory can look converged or not purely by chance.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------
# 1) Model setup
# ----------------------------------------------------------------------
rho = 0.7           # spectral radius of A_K
phi = 0.5           # rotation angle (rad)
alpha = 3.0         # disturbance tail index (Student-t degrees of freedom)
p, q = 0.5, 0.5     # tail-balance weights (symmetric disturbance)

n_steps = 2_000_000  # length of each simulated trajectory
n_reps = 15          # independent replicate trajectories, averaged
base_seed = 2000

Ak = rho * np.array([[np.cos(phi), -np.sin(phi)],
                      [np.sin(phi),  np.cos(phi)]])
b = np.array([1.0, 0.0])
c = np.array([1.0, 0.0])

# ----------------------------------------------------------------------
# 2) Closed-form theta_Y (Proposition 1)
# ----------------------------------------------------------------------
def closed_form_theta(Ak, b, c, alpha, p, q, n_lags=200):
    g = np.array([c @ np.linalg.matrix_power(Ak, j) @ b for j in range(n_lags)])
    g_plus = np.maximum(g, 0.0)
    g_minus = np.maximum(-g, 0.0)
    num = (g_plus.max() ** alpha) * p + (g_minus.max() ** alpha) * q
    den = p * np.sum(g_plus ** alpha) + q * np.sum(g_minus ** alpha)
    theta = num / den
    return theta, g_plus.max(), g_minus.max()

theta_Y, sup_gplus, sup_gminus = closed_form_theta(Ak, b, c, alpha, p, q)

# ----------------------------------------------------------------------
# 3) Simulate one closed-loop trajectory of Y_k
# ----------------------------------------------------------------------
def simulate_Y(n_steps, seed):
    rng = np.random.default_rng(seed)
    zeta = rng.standard_t(alpha, size=n_steps)
    x = np.zeros(2)
    Y = np.empty(n_steps)
    for k in range(n_steps):
        Y[k] = c @ x
        x = Ak @ x + zeta[k] * b
    return Y

# ----------------------------------------------------------------------
# 4) Ferro-Segers (2003) intervals estimator
# ----------------------------------------------------------------------
def ferro_segers_theta(Y, quantile):
    """Empirical extremal index of the sequence Y at the given threshold
    quantile, using the intervals estimator of Ferro and Segers (2003)."""
    u = np.quantile(Y, quantile)
    exceed_idx = np.flatnonzero(Y > u)
    N = exceed_idx.size
    if N < 2:
        return np.nan, N
    T = np.diff(exceed_idx).astype(float)  # inter-exceedance gaps

    if T.max() <= 2:
        num = 2.0 * (T.sum()) ** 2
        den = (N - 1) * np.sum(T ** 2)
    else:
        Tm1 = T - 1.0
        num = 2.0 * (Tm1.sum()) ** 2
        den = (N - 1) * np.sum(Tm1 * (T - 2.0))

    theta_hat = num / den if den > 0 else np.nan
    return min(1.0, theta_hat), N

# ----------------------------------------------------------------------
# 5) Sweep threshold quantiles, averaging over independent replicates
# ----------------------------------------------------------------------
quantiles = np.array([0.90, 0.95, 0.98, 0.99, 0.995, 0.998, 0.999, 0.9995, 0.9998])

theta_acc = np.zeros_like(quantiles)
n_acc = np.zeros_like(quantiles)
for rep in range(n_reps):
    Y = simulate_Y(n_steps, seed=base_seed + rep)
    for i, qtl in enumerate(quantiles):
        th, N = ferro_segers_theta(Y, qtl)
        theta_acc[i] += th
        n_acc[i] += N

theta_hats = theta_acc / n_reps
avg_n_exceed = n_acc / n_reps

# ----------------------------------------------------------------------
# 6) Report and plot
# ----------------------------------------------------------------------
print(f"Closed-form theta_Y (Proposition 1) = {theta_Y:.4f}")
print(f"  sup g_plus  = {sup_gplus:.4f}")
print(f"  sup g_minus = {sup_gminus:.4f}")
print()
print(f"{'quantile':>9} {'avg N_exceed':>13} {'theta_hat':>10}")
for qtl, N, th in zip(quantiles, avg_n_exceed, theta_hats):
    print(f"{qtl:9.4f} {N:13.0f} {th:10.4f}")

fig, ax = plt.subplots(figsize=(6, 4))
ax.plot(quantiles, theta_hats, "o-", color="tab:blue", label=r"$\hat\theta$ (Ferro-Segers)")
ax.axhline(theta_Y, color="tab:red", linestyle="--", label=r"$\theta_Y$ (closed form)")
ax.set_xlabel("threshold quantile")
ax.set_ylabel(r"$\theta$")
ax.legend()
fig.tight_layout()
fig.savefig("theta_estimation_convergence.png", dpi=200)
fig.savefig("theta_estimation_convergence.pdf")
print("\nSaved theta_estimation_convergence.png/.pdf")
