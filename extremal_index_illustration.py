"""
Illustrates Leadbetter's extremal index theta and the threshold sequence
mu_n(tau) used in Definition (extremal index) of the manuscript.

Model: Z_k = rho * Z_{k-1} + zeta_k, zeta_k i.i.d. standard Pareto(alpha).
This is the AR(1) special case of the closed-loop projection Z_k = sum_j g_j
zeta_{k-1-j} used in the paper, with g_j = rho^j. For this model the
extremal index has the closed form

    theta = 1 - rho**alpha,

which follows from the paper's own Proposition (extremal index): the
marginal tail scale is S_inf = sum_j rho^(j*alpha) = 1/(1 - rho**alpha)
(one-big-jump over all lags), while the running-maximum limit is governed
only by the single largest coefficient g_+ = g_0 = 1 (the running max is
realized at the moment the big jump itself occurs). Hence
theta = g_+**alpha / S_inf = 1 - rho**alpha.

The threshold mu_n(tau) is set empirically (as the (1 - tau/n) quantile of a
long reference simulation of the stationary process), rather than from the
asymptotic closed-form quantile formula, since the asymptotic formula is
only accurate for very large n once rho is not small; using the empirical
quantile keeps the figure correct at the moderate n used for the
illustration.

The figure has two panels:
  (a) a single realization of the dependent process with a threshold
      mu_n(tau) drawn at a fixed expected marginal exceedance count tau,
      showing that exceedances arrive in a small number of multi-step
      clusters;
  (b) the empirical/theoretical curve of P(max_{k<=n} Z_k <= mu_n(tau)) as
      a function of tau, compared against the i.i.d. reference e^{-tau}
      (theta = 1) and the dependent-process theory e^{-theta*tau},
      illustrating how theta discounts the running-maximum law relative
      to the naive i.i.d. count implied by mu_n(tau).

Run:
    python extremal_index_illustration.py
Output:
    extremal_index_illustration.png (saved next to this script)
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import gridspec

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

rng = np.random.default_rng(3)

alpha = 2.5      # tail index
rho = 0.85       # AR(1) persistence (plays the role of the closed-loop A_K)
n_series = 500   # length of the illustrated single-realization series
n_mc = 2000      # length of series used for the tau-curve Monte Carlo
n_reps = 4000    # replications for the Monte Carlo estimate of P(max <= mu_n(tau))
n_ref = 3000000  # long reference run used to get accurate empirical quantiles
                 # (bumped 10x from 300,000: at 300,000 the empirical quantile
                 # of mu_n(tau) was itself biased low at the extreme quantile
                 # levels this figure needs, e.g. q=0.9999 at tau=0.2, which
                 # systematically inflated the empirical P(max<=mu_n(tau)) curve
                 # above the theoretical e^{-theta*tau} curve; verified by a
                 # controlled test holding n_mc fixed and varying only n_ref)


def simulate_ar1_pareto(n, rho, alpha, rng, burn=500):
    """Simulate Y_k = rho*Y_{k-1} + zeta_k with zeta_k i.i.d. standard
    Pareto(alpha) (survival function P(zeta > x) = x^{-alpha}, x >= 1)."""
    zeta = (rng.uniform(size=n + burn)) ** (-1.0 / alpha)
    y = np.zeros(n + burn)
    for k in range(1, n + burn):
        y[k] = rho * y[k - 1] + zeta[k]
    return y[burn:]


theta_theory = 1.0 - rho ** alpha

# Long reference run gives accurate empirical marginal quantiles for mu_n(tau).
y_ref = simulate_ar1_pareto(n_ref, rho, alpha, rng)


def mu_n(n, tau):
    """Threshold such that the expected number of marginal exceedances in n
    steps is tau, i.e. n * P(Y > mu_n(tau)) = tau, evaluated empirically."""
    q = 1.0 - tau / n
    return np.quantile(y_ref, q)


# ---------- Panel (a): clustering in a single realization ----------
y = simulate_ar1_pareto(n_series, rho, alpha, rng)

tau0 = 8.0
mu0 = mu_n(n_series, tau0)

# ---------- Panel (b): P(max <= mu_n(tau)) vs tau ----------
taus = np.linspace(0.2, 6, 25)
p_iid_theory = np.exp(-taus)
p_dep_theory = np.exp(-theta_theory * taus)

maxvals = np.array([simulate_ar1_pareto(n_mc, rho, alpha, rng).max() for _ in range(n_reps)])
p_dep_emp = np.array([np.mean(maxvals <= mu_n(n_mc, tau)) for tau in taus])

# ---------- Figure ----------
fig = plt.figure(figsize=(10, 4.0))
gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1.25], wspace=0.35)

ax1 = fig.add_subplot(gs[0])
ax1.plot(y, color="steelblue", lw=0.9)
ax1.axhline(mu0, color="crimson", ls="--", lw=1.3, label=r"$\mu_n(\tau)$")
exceed = y > mu0
ax1.scatter(np.where(exceed)[0], y[exceed], color="crimson", s=28, zorder=5)
n_clusters_a = int(np.sum(np.diff(np.r_[0, exceed.astype(int), 0]) == 1))
ax1.set_ylabel("$Z_k$")
ax1.set_xlabel("Time step $k$")
ax1.legend(loc="upper left", fontsize=8, frameon=False)

ax2 = fig.add_subplot(gs[1])
ax2.plot(taus, p_iid_theory, "k--", label=r"$e^{-\tau}$")
ax2.plot(taus, p_dep_theory, color="crimson", lw=2, label=r"$e^{-\theta\tau}$")
ax2.plot(taus, p_dep_emp, "o", color="crimson", ms=4, mfc="white",
         label=r"$\mathbb{P}(\max_{k\leq n} Z_k \leq \mu_n(\tau))$")
ax2.set_xlabel(r"$\tau$")
ax2.set_ylabel(r"$\mathbb{P}(\max_{k\leq n} Z_k \leq \mu_n(\tau))$")
ax2.set_xlabel(r"$\tau$")
ax2.legend(fontsize=8, frameon=False)

fig.tight_layout()
fig.savefig(os.path.join(SCRIPT_DIR, "extremal_index_illustration.png"), dpi=200)
print("theta_theory =", theta_theory, "mu0 =", mu0)
print("panel (a): %d exceedances, %d clusters" % (exceed.sum(), n_clusters_a))
