# Figure 3 — Extremal index validation on the unicycle scenario

Validates the closed-form extremal index $\theta_Y$ (Proposition 1) against
the data-driven Ferro–Segers intervals estimator, using the closed-loop
gain $A_K$ actually derived from the unicycle obstacle-avoidance scenario
in [`../fig2_fig4_tube_smpc_pipeline/`](../fig2_fig4_tube_smpc_pipeline/)
— frozen at the representative operating point where the robot passes the
obstacle at exactly the tightened safety distance, heading tangentially
around it — rather than validated only on the generic toy system in
[`../fig1_extremal_index_illustration/`](../fig1_extremal_index_illustration/).

```
e_{k+1} = A_K e_k + w_k,   w_k = zeta_k * b,   Y_k = c^T e_k
```

`validate_theta_unicycle.py` computes $\theta_Y$ in closed form from
$A_K$, $b$, $c$, and the disturbance's tail parameters, then cross-checks
it against the Ferro–Segers estimator averaged over several long,
independent simulated trajectories — their agreement (printed to the
console) is the validation. The saved figure shows a single illustrative
trajectory with the exceedance clusters marked, matching the style of the
Figure 1 illustration.  

 

## Run

```bash
python validate_theta_unicycle.py 
```

`validate_theta_unicycle.py` runs several independent long simulated
trajectories for the Ferro–Segers cross-check. 
