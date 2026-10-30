# Scientific and Code Audit Findings

> Historical audit dated 2026-10-29. Current Xing source reconstruction and
> v1/v2 fix dispositions are in `sources/xing_paper/FIDELITY_REQUIREMENTS.md`;
> current comparative conclusions are in `FINAL_COMPARISON_REPORT.md`.

Date: 2026-10-29. Scope: current `experiments/reiter20` implementations,
stored comparison artifacts, and `ACCEPTANCE_REPORT.md`. Paths below are
relative to this directory; line references describe the audited snapshot.

## Prioritized Findings and Disposition

1. **P1: Approximate road constraints do not guarantee the physical margin.**
   `paper_comparison/model.py:132` approximates corner projection; the test at
   `tests/test_paper_model.py:292` checks the approximate formula, not exact
   Cartesian projection. The width-4.9 circles curve case records physical
   clearance **0.194172584 m**, below the requested 0.2 m, despite model residual
   about 1.47e-8 (`results/width_sweep/half_width_4.9/paper6_circles_sqp_curve.json:61`).
   Independent evaluation at saved state 49 gives 0.194248231 m while the
   approximate constraint gives 0.200000018 m. Saved sweep corner-offset errors
   reach about 0.07305 m.
   **Disclosure fixed; model limitation remains.** The report now states the
   shortfall. Physical-margin claims require exact projection or a validated
   error buffer and new validation. No sampled road exit is a narrower,
   supported observation, not a continuous-time or real-driving safety guarantee.

2. **P2: CiLQR acceptance certifies feasibility, not stationarity.**
   `paper_comparison/cilqr.py:181` stops on feasibility and
   `paper_comparison/cilqr.py:189` derives status from feasibility alone. Inner
   iteration-budget or line-search termination need not establish convergence.
   **Disclosure fixed; measurement limitation remains.** The report now calls
   these feasibility-accepted trajectories. Distinct convergence/termination
   diagnostics and an optimality test are still needed for stationary-solution
   claims or equal-optimality comparisons with SQP.

3. **P2: Historical Reiter settings are not its effective configuration.**
   `scripts/run_paper_comparison.py:274` stores shared `Settings`, while Reiter
   uses separate options constructed at `scripts/run_paper_comparison.py:114`.
   Its acceleration-square weight is **1345.6**, versus **1** for the paper
   model; paper jerk bounds in historical Reiter metadata are inactive.
   **Documentation supplemented:** `results/effective_method_configurations.json`
   now records effective configurations without rewriting historical cases.
   Cross-method model, cost, and constraint differences remain; use the own-method
   geometry and 6/9-state ablations for controlled attribution.

4. **P3: Numerical precision and archive wording needed correction.**
   Correct three-decimal progress differences are **55.965 / 63.004 m** against
   Reiter Lifted/ellipse and **55.249 / 62.512 m** against own circles. Paired
   maxima round to **9.48e-7**, **2.91e-6**, and **7.42e-10** for physical state,
   first jerk, and first curvature rate respectively.
   **Documentation fixed:** corrected values or explicitly approximate rounding;
   executed states/controls distinguished from unsaved accepted horizon predictions;
   N=80 warm-phase counts disclose 15 zero-control reinitializations per
   representation. No historical measurements were corrected by this audit.

## Verified Evidence and Remaining Boundaries

- Counts match: 32 formal cases / 3,840 executed cycles; width sweep 36 cases /
  4,105 cycles, with 34 complete. Including proxies: 76 cases / 8,455 cycles.
  These are configurations and scenarios, not independent paper baselines.
- Scaling matches: 1,512 calls, 1,482 accepted, 741 jointly accepted pairs;
  timing reductions 18.83% / 19.79% / 19.64%. Failures remain in the measurements.
  Variable counts are nominal multiple-shooting counts, not memory measurements
  or evidence of dimensionality explosion.
- All 34 audited source entries match recorded provenance, including historical
  runner snapshots. Scaling and six width-sweep source hashes also match.
- The aligned-rectangle grid oracle and reported fractions are consistent.
  Zero unsafe-accepted points on that finite grid is not an all-angle or global
  geometric proof, and grid fractions are not driving-success probabilities.
- Xing remains an explicitly unverified inspired proxy, not the original
  baseline. Cartesian obstacle geometry is used across the compared methods;
  these results do not isolate a pure-Frenet versus joint-frame advantage.
- Reiter terminal obstacle omissions and unequal wall-time scopes are disclosed
  limitations, not newly discovered hidden defects. Do not compare cross-method
  wall times as an acceleration ratio or infer general superiority.

## Independent Verification Status

The audit independently recomputed stored-data counts, timing means, differences,
scaling statistics, source hashes, and selected exact physical geometry. It did
not rerun simulations, solvers, timed benchmarks, or pytest. The inspected log
records **168 passed, 147 warnings** (`logs/comparison_final_tests.log:97`);
this is historical test evidence, not independent test reproduction.

The follow-up checked the revised report and configuration supplement. This
write operation creates only `AUDIT_FINDINGS.md`; it changes no models, results,
or existing documentation. Corrected disclosures do not resolve the remaining
model limitations or complete faithful own/Reiter/Xing reproduction.
