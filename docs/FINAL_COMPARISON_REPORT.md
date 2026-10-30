# Final Experimental Report: This Paper, Reiter [20], and Xing [26]

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent.
- Origin Date: 2026-10-30, Asia/Singapore.
- Verification Status: VERIFIED, limited to the independent equation reconstruction, bus adaptation, and sampled evaluation stated in this report.
- Version Label: final-three-paper-bus-v2.
- This report supersedes the 2026-10-29 report's current-state judgement that "the full Xing text was missing and only a proxy was available".
- The manuscript body has not been modified; it does not claim to have used the original authors' deployed code for the three papers or to have reproduced the original vehicle performance.

## 1. Conclusions First

**The methods of the three papers have been independently reconstructed and compared in a common scenario on the same machine.
The results support some of the design advantages of this paper, but do not support the claim that this paper is overall superior to Reiter and Xing.**

1. The hybrid geometry of this paper completes overtaking in the narrow road tested, while the reconstructed ellipse/circles of Reiter and this paper's own circles ablation both follow the obstacle. An ablation within the same model of this paper that replaces only the collision geometry, together with an independent geometric grid audit, supports the claim that the hybrid reduces conservatism relative to these collision approximations.
2. **The newly added Xing keep-left corridor reconstruction also completes the narrow-road overtake, with slightly greater progress than this paper.**
   The inference that "only this paper can overtake in the narrow road", or that it is "superior to all joint-frame prior art", must therefore be withdrawn.
   The abort of the old Xing-inspired proxy can no longer be cited as evidence of Xing's method performance.
3. The same-model six/nine-state ablation supports the overhead benefit of removing redundant states, with the native average solve time reduced by about 20%. This is not a measured speed-up ratio relative to Xing. This paper is also not the lowest-dimensional: Reiter Direct has five states, this paper has six, and the Xing joint state is seven-dimensional.
4. The SQP version of this paper has a lower cost in the current implementation, but the local Python AL-CiLQR has no real-time advantage. Solve times measured under different backends, costs, input dynamics, and diagnostic scopes cannot be used to demonstrate pure algorithmic speed-up.
5. The full Xing text has now been checked. The numerical weights, the complete vehicle-body corridor construction, and the upstream decisions that the original text does not publish still require adaptation. This is disclosed together with the reconstruction boundaries of Reiter and this paper; the adapted results are not called an author reproduction table.

## 2. Comparison Content and Common Conditions

This round re-executed the original eight formal configurations and added one Xing seven-state corridor configuration and one yield strategy ablation:

| Method family | Configuration | Count |
| --- | --- | ---: |
| Reiter equation reconstruction | Direct/Lifted × ellipse/circles | 4 |
| Equation reconstruction and ablations of this paper | six-state hybrid SQP, nine-state hybrid SQP, six-state circles SQP, six-state hybrid AL-CiLQR | 4 |
| Xing equation reconstruction | seven-state joint-frame QP, keep-left main configuration | 1 |
| Xing upstream strategy ablation | same planner, changed to always yield | 1 |

That is three method families and ten configurations; yield is not another independent algorithm. The old proxy is kept only as history and is not mixed into the formal charts of this round.

The common vehicle is a bus of length 6.50 m, width 2.30 m, and wheelbase 4.32 m, with a 1.00 m rear overhang;
N=40, dt=0.1 s, 4 s prediction, 12 s closed loop; initial speed 12 m/s, target 15 m/s,
obstacle rear axle `s=35+8t,n=0`. Straight-road curvature 0, curve curvature 1/60 m^-1.
Wide road half-width 8.5 m, lateral target 5.5 m; narrow road half-width 4.6 m, lateral target 2.6 m.
The three method families share this environment and target, but they are not optimization problems with the same physical states, costs, and constraints.

All planning calls run sequentially, with the numerical library thread count set to 1. All forty cases of the main comparison completed 120 steps, for a total of **4,800 execution cycles**, and the independent true-rectangle evaluation found no collision or boundary violation at the 0.02 s sampling points.
These are deterministic synthetic scenarios and cannot be interpreted as a statistical success rate or a continuous-time safety proof.

## 3. What the Xing Text Actually Implements

The full text was provided by the user on 2026-10-30 as
`../../Vehicle_Motion_Planning_With_Joint_Cartesian-Frent_MPC.pdf`,
SHA-256 `197607c3606b992f2111bd32beeb9a59351031116403383185b0f35ff78c4825`.

The new implementation `paper_comparison/xing2022.py` uses:

- Seven-dimensional state `[V,X,Y,psi,S,D,phi]`, input `[delta,a]`.
- The exact discretization of the constant-input model of Eq. (6), with a stable continuous extension for zero steering angle.
- The affine dynamics of Eq. (7), together with the full coordinate coupling of Eq. (17)/(19);
  the relative heading row is derived as `phi=psi-theta_ref(S)`, and is not mistaken for the Cartesian heading.
- The prior S-dependent corridor of Eq. (12)--(15), the input and input-increment bounds of Figure 3,
  the future-state tracking of Eq. (20), and the adjacent input-increment cost.
- Per-iteration updates of the prior, the dynamics, the coordinate coupling, and the corridor, using OSQP; the rollout backtracking line search of the old proxy and the Reiter collision geometry are not reused.

The original text does not publish the complete vehicle-body corridor inflation implementation. This work uses the Frenet envelope of the bus rectangle at the prior pose for road shrinkage and obstacle expansion, and for circular roads it considers the radial extremum inside the boundary.
The main configuration provides the keep-left decision and yield when the left side is blocked; another configuration always yields.
The full upstream dynamic programming, arbitrary multi-obstacle topology, and the ahead overtake-wall branch are not reproduced.
The hard-constraint version permitted by the original text is selected, with epsilon=0.

The numerical Q/R values are missing from the original text; this work uses
`Q=dt*diag(1000,0,0,0,1,500,1000)` and `R=diag(2e6,1)/dt`.
Q is positive semidefinite and the Cartesian position is not tracked separately; this is an adaptation recorded publicly,
not a claim of identical tuning to the authors. A bus lateral acceleration bound and a common execution check are added in addition.
The lateral acceleration constraint and the final residual are checked at the start and the end of every constant-input interval;
states, inputs, and key diagnostics must be finite.

Stopping requires both a mean prior position change <=0.2 m and a maximum residual of the nonlinear model <=1e-4;
the latter is an additional strict acceptance check, and its rejection must not be called directly a failure of the original paper itself.
The complete text audit and all choices are in
`sources/xing_paper/FIDELITY_REQUIREMENTS.md` and
`sources/xing_paper/IMPLEMENTATION_NOTES.md`.

The audit found that one v1 accepted prediction reached 5.0084346 m/s² at the end of an interval,
exceeding the adapted limit of 5, while the start-of-interval check did not detect it; the executed trajectories contain no such violation.
This report uses the **v2 that was rerun in full** after correcting the end-of-interval check and the non-finite rejection.
The old source code, results, and report snapshot are preserved; v1 has not been rewritten into the corrected result.
The overall normalization of the Xing objective function and the dual residual in the original objective units are also explicitly recorded;
the nominal QP tolerance cannot be interpreted as the same optimality accuracy as SQP.
Adding 0.2 m in the Frenet frame is also not automatically equivalent to 0.2 m of Euclidean vehicle-body clearance on the curve;
the physical results still follow the independent true-rectangle evaluation.

## 4. Planning Performance

The values in the table are the Frenet s after 12 s, in units of metres. All configurations completed in the four scenarios;
"following" is not a solve failure.

| Configuration | Wide straight | Wide curve | Narrow straight | Narrow curve |
| --- | ---: | ---: | ---: | ---: |
| Reiter Direct / ellipse | 176.594 | 190.591 | 121.377 | 120.817 |
| Reiter Lifted / ellipse | 176.594 | 190.591 | 121.377 | 120.817 |
| Reiter Direct / circles | 176.594 | 190.591 | 122.489 | 121.710 |
| Reiter Lifted / circles | 176.594 | 190.591 | 122.489 | 121.710 |
| This paper 6 / hybrid SQP | 177.127 | 191.096 | 177.342 | 183.820 |
| This paper 9 / hybrid SQP | 177.127 | 191.096 | 177.342 | 183.820 |
| This paper 6 / circles SQP | 177.127 | 191.096 | 122.093 | 121.308 |
| This paper 6 / hybrid AL-CiLQR | 177.145 | 191.102 | 177.340 | 183.846 |
| Xing 7 / keep-left corridor | 177.266 | 192.050 | 177.408 | 184.075 |
| Xing 7 / yield strategy ablation | 122.930 | 122.403 | 122.909 | 122.769 |

On the wide road, everything except the deliberate yield ablation completes the overtake. On the narrow road, the hybrid of this paper and Xing keep-left
complete the overtake, while Reiter, the circles ablation of this paper, and the deliberate yield ablation all follow the obstacle.
The Xing yield row cannot be used to show that this paper is superior to Xing; it demonstrates the effect of the upstream decision.

Compared with Xing keep-left, the hybrid SQP of this paper has lower progress by
0.139, 0.954, 0.066, and 0.254 m in the four scenarios. These very small differences are affected by costs, control variables, and local solutions;
**there is no evidence that the planning quality of this paper is better than Xing's, and it is also not claimed on this basis that Xing is generally superior to this paper.**

The narrow-road progress difference between this paper and Reiter is about 55--63 m, and it is also present in this paper's own geometric ablation.
The original Reiter force cost is equivalent to `1345.6*a^2`, while this paper uses `1*a^2`;
an existing single-weight sensitivity study changed Reiter to `1*a^2` and circles still followed the obstacle,
but the direction of the small wide-road progress difference reversed. The difference above should therefore not all be attributed to the coordinate representation.

### Narrow-road True Vehicle-Body Clearance

| Configuration | Minimum obstacle clearance: straight/curve m | Minimum road clearance: straight/curve m |
| --- | --- | --- |
| This paper 6 / hybrid SQP | 0.625 / 0.616 | 0.430 / 0.258 |
| Xing 7 / keep-left corridor | 0.539 / 0.396 | 0.495 / 0.478 |

This paper has a larger obstacle clearance in these two cases and Xing has a larger road clearance;
what is reflected is the allocation in trajectory space, and a single minimum distance must not be interpreted as the method being overall safer.

## 5. Efficiency and State Dimensionality

In the table below, the acados column is the native `time_tot` with the first call removed; the Xing/CiLQR column is the planning wall time of the local
Python implementation. These are not the same timing basis and must not be divided to form a method speed-up ratio.

| Configuration and timing basis | Wide straight ms | Wide curve ms | Narrow straight ms | Narrow curve ms |
| --- | ---: | ---: | ---: | ---: |
| Reiter Direct / ellipse, native acados | 0.793 | 0.889 | 5.255 | 7.247 |
| Reiter Lifted / ellipse, native acados | 0.923 | 0.991 | 7.658 | 13.272 |
| This paper 6 / hybrid SQP, native acados | 1.847 | 2.139 | 3.108 | 4.849 |
| This paper 9 / hybrid SQP, native acados | 2.278 | 2.591 | 3.725 | 5.851 |
| Xing 7 / keep-left, Python wall | 20.710 | 27.349 | 19.729 | 26.086 |
| Xing 7 / yield, Python wall | 60.974 | 125.826 | 36.415 | 103.216 |
| This paper AL-CiLQR, Python wall | 179.309 | 204.819 | 254.343 | 278.865 |

In the new main comparison, all Xing keep-left warm-start calls are under 100 ms; the yield configuration is slower,
and the fraction of warm starts exceeding 100 ms on the wide-road curve is 54.62%. The timing behaviour of the local implementation is not the on-vehicle
performance of the authors, and a tail-latency guarantee cannot be derived from the mean of the same scenario alone.
The average narrow-road cost of the AL-CiLQR of this paper is about 254/279 ms, with no verified real-time advantage;
its status=0 still only indicates that feasibility passed, with no independent stationarity certificate.

At N=40, the nominal uneliminated multiple-shooting variable counts are as follows:

| Representation | State dimension | `nx*(N+1)+nu*N` |
| --- | ---: | ---: |
| Reiter Direct | 5 | 285 |
| This paper | 6 | 326 |
| Xing joint | 7 | 367 |
| Reiter Lifted | 8 | 408 |
| This paper lifted ablation | 9 | 449 |

This count for this paper is 11.17% lower than Xing's, 20.10% lower than Reiter Lifted,
but 14.39% higher than Reiter Direct. The physical states and algebraic variables contained in each method differ;
this is only a count, not an efficiency experiment controlling for other factors, and not the dimension after condensing or in memory.
AL-CiLQR does not explicitly optimize these multiple-shooting state variables, so this formula must not be described as its true optimization dimension.

The evidence that controls for other factors comes from the existing **same-model six/nine-state replay ablation**:

| N | Six-state native mean ms | Nine-state native mean ms | Reduction |
| --- | ---: | ---: | ---: |
| 20 | 2.459 | 2.922 | 18.83% |
| 40 | 8.594 | 10.295 | 19.79% |
| 80 | 29.309 | 35.065 | 19.64% |

This replay set contains 1,512 calls in total, including the duration of failed calls, with 1,482 accepted; at N=80 there are cold-start and
warm-start failures as well as re-initialization after failure. The maximum physical state difference of the 741 pairs accepted on both sides is
9.48e-7, supporting that they preserve essentially the same physical solution. The raw archive is in `results/dimension_scaling/`.
The recommendation is therefore to use "measured redundant-state overhead" and to delete "dimensionality explosion".

### Correction 2026-10-30 — the six/nine-state overhead percentages

The reduction percentages reported in the tables above were arithmetically
incorrect; the six-state and nine-state mean solve times they were derived
from were correct.

| N | Six-state mean ms | Nine-state mean ms | Old column | Correct |
| --- | ---: | ---: | ---: | ---: |
| 20 | 2.459 | 2.922 | 15.85% | **18.83%** |
| 40 | 8.594 | 10.295 | 16.52% | **19.79%** |
| 80 | 29.309 | 35.065 | 16.42% | **19.64%** |

Recomputed from `results/dimension_scaling/paired_controls.csv`:

- warm-start calls only, 240 paired calls per horizon: **18.83% / 19.79% / 19.64%**;
- all 252 paired calls per horizon, cold and warm: **18.59% / 19.60% / 19.67%**;
- calls accepted on both sides only: 18.59% / 19.60% / 19.60%.

The conclusion is unchanged in direction and slightly stronger in magnitude: the
redundant states of the lifted nine-state formulation cost about 20% of the
mean native solve time, which is linear in character and not an exponential
growth of the state dimension.

An earlier archived run (`results/dimension_scaling_pre_quiet/`) was taken while
other processes were competing for the CPU; it reported 13.67% / 17.50% /
19.87% and was replaced by the current archive, which carries an idle-check and
an overlap guard in `provenance.json`.



## 6. Geometric Conservatism and Road-Width Sensitivity

In the existing solver-independent same-direction rectangle grid audit, the proportion of safe positions with true clearance >=0.2 m
that were conservatively rejected by hybrid/circles/ellipse was **4.52% / 10.48% / 25.92%** respectively.
None of the three accepted an unsafe point on this finite grid. This supports hybrid being less conservative than these two shape approximations,
but does not support a continuous guarantee over all poses, and the same proportion for the Xing corridor has not been measured.
Xing can use a configuration-space corridor closer to a rectangle; the ellipse numbers must not be attributed to Xing.

The newly added Xing keep-left six-width, two-road sweep is merged with the previous three-configuration sweep as follows:

| Half-width m | This paper hybrid straight/curve | This paper circles straight/curve | Reiter Lifted circles straight/curve | Xing keep-left straight/curve |
| --- | --- | --- | --- | --- |
| 4.0 | following / following | following / following | following / following | abort at step 12 / abort at step 4 |
| 4.3 | overtake / abort at step 9 | following / following | following / following | overtake / abort at step 28 |
| 4.6 | overtake / overtake | following / following | following / following | overtake / overtake |
| 4.9 | overtake / overtake | overtake / overtake | overtake / abort at step 16 | overtake / overtake |
| 5.2 | overtake / overtake | overtake / overtake | overtake / overtake | overtake / overtake |
| 5.5 | overtake / overtake | overtake / overtake | overtake / overtake | overtake / overtake |

The lateral reference is taken synchronously as `half_width-2`; this is a joint sensitivity of width and target position, not a single-factor experiment.
Of the 12 new Xing cases, 9 completed, executing 1,124 cycles, with no sampled collision/boundary violation before stopping:

- At 4.0 m on the straight road, the mean prior change after 50 rounds was still about 0.659 m, not converged.
- At 4.0 m on the curve, the mean change after 50 rounds was about 0.0103 m, but the maximum residual of the exact model was about 0.0107,
  so it was rejected by the additional nonlinear feasibility check; it cannot be said that the position-change criterion of the original paper also failed.
- At 4.3 m on the curve, the QP returned primal infeasible inaccurate at iteration 3 of one cycle, and the run stopped.

These are results under the current hard corridor, bus adaptation, initialization, and acceptance rules; they do not prove that the original Xing method
necessarily fails on these roads. In particular, the original text permits soft constraints and external decisions, and this experiment did not use an extra strategy
specifically to rescue a failing case. The hybrid of this paper also aborts on its own, so it cannot be claimed to always succeed.

The historical road-projection audit found that the circles of this paper on the 4.9 m curve have a physical road clearance of only 0.194173 m
despite a nominal margin of 0.2 m. The existing exact road sensitivity version raises the corresponding case to
0.200000029 m, but other cases still have small inter-node shortfalls and convergence failures.
This round does not silently substitute this paper's original model; this limitation is retained.

## 7. Verification, Results, and Re-run

The full test suite reports **254 passed, 147 warnings**; of these, 82 are new independent Xing tests and audit regressions.
Log: `logs/final_with_xing_tests_v2.log`. The warnings are mainly pre-existing NumPy/CasADi and acados
compatibility/deprecation warnings; solve-failure tests also keep the expected native error output.

`scripts/verify_final_comparison.py` has checked the main comparison and the new width sweep over
**52 cases and 5,924 execution cycles**, verifying the summaries, the acceptance criteria, the source code, and the paper SHA,
and has independently replayed all **2,084 Xing execution cycles** with scipy DOP853, recomputing the 0.02 s
true vehicle-body clearance. The maximum state error of the 960-cycle replay of the main comparison is about 5.7e-14.
The manifest is `results/final_three_paper_manifest_v2.json`.
It also checked that the predicted states and inputs of all accepted Xing solutions are finite, as well as the lateral acceleration at both ends of
every interval of the full prediction horizon; the maximum archived value is **4.999999882 m/s²**.
No nonlinear stationarity was certified, and no continuous collision proof was performed over all predictions.

Main files:

- Main comparison raw results: `results/final_three_paper_wide_v2/` and
  `results/final_three_paper_narrow_v2/`; both have full-precision summary CSV/JSON and provenance.
- Main comparison plots: `results/final_three_paper_plots_v2/trajectories.png`,
  `trajectories_cartesian.png`, `performance.png`; full-precision merged table `comparison.csv`.
- Width sweep: `results/xing_paper_width_sweep_v2/`;
  charts merged with the historical sweep: `results/final_width_comparison_plots_v2/width_sweep.png`, `width_sweep.csv`.
- Six/nine-dimensional ablation: `results/dimension_scaling/dimension_scaling.png`.
- Geometry audit: `results/geometry_conservatism/geometry_conservatism.png`.
- The `*_predictions.json` of each Xing case saves all accepted predictions, the controls, and the recomputed corridor;
  within a case, the per-iteration prior errors and the QP status are also saved. The other methods continue to use the execution state/control and failed-prediction archives.
- Supplementary information on the effective configurations of the original Reiter is in `results/effective_method_configurations.json`;
  the weights of this paper in the common Settings of a case must not be taken as the actual Reiter weights.
- The first pilot of `results/xing_paper_wide_v1/` encountered a NumPy
  bool error when serializing the yield result; it is recorded as a Python archiving error with the log and the exact source snapshot retained;
  this report uses the subsequent complete re-run and does not count it as an algorithmic failure.

Re-running should use a new output directory:

```bash
cd experiments/reiter20
bash scripts/run_final_comparison.sh --steps 120 --output results/recheck_wide
bash scripts/run_final_comparison.sh --steps 120 \
  --half-width 4.6 --lateral 2.6 --output results/recheck_narrow
.venv/bin/python scripts/run_xing_width_sweep.py --output results/recheck_xing_width

.venv/bin/python scripts/plot_paper_comparison.py \
  --inputs results/recheck_wide results/recheck_narrow \
  --merge-by-width --cartesian --output results/recheck_plots
```

All methods involve adaptation of weights, constraints, and boundaries; the reconstruction notes on the classes missing from the original Reiter work,
the terminal constraint differences, the Eq.34e correction of this paper, and the road-projection limitations are still in the historical
`ACCEPTANCE_REPORT.md`. These reproduction boundaries must not be deleted merely because all three sides can now be run.

## 8. Final Answers to the Reviewer Comments

It is now possible to respond: a baseline of equation reconstructions of the original Reiter and Xing has been added, evaluated in a common bus scenario,
and the two design effects are separated by state-dimensionality and geometry ablations using the same physical model.

The following must be acknowledged at the same time:

- The joint-frame idea is not originated by this paper and cannot in itself be claimed as a contribution distinct from the two earlier works.
- "Leaner state" holds only relative to certain joint/lifted representations; Reiter Direct is lower-dimensional.
- "Dimensionality explosion" is unfounded and should be replaced with a quantified redundant-state overhead.
- The advantage of the hybrid geometry applies to some shape approximations and to the tight spaces measured here, and the Xing corridor can also overtake;
  no general advantage of this paper over all joint-frame methods has been demonstrated.
- The local Python AL-CiLQR has no real-time advantage, and the SQP ablation timings must not be reported as CiLQR results.

It is recommended to focus the contributions on **the hybrid vehicle-body constraints for long vehicles, the less conservative planning results under
specific spatial conditions, and the state-organization overhead measured through controlled experiments**. This matches the actual outcome of this round of
experiments better than "first joint coordinates, broadly faster, and generally stronger on narrow roads".

## 9. Supplement: Verification Results Against Table 2 of the Reiter Text (2026-10-30)

In preparation for the revised manuscript, the content of the original Reiter text (`sources/Reiter2023a.pdf`) corresponding to the related-work
statements of this paper was checked item by item, and the results are recorded below for citation in the main text.

1. In **Table 2 (comparison of CF representations)**, the problems column of "Lifted ODE Frenet" (8 states) is written as
   **"redundant states"**, while that column is empty for "Direct Elimination Frenet" (5 states).
   That is, the observation that "the lifted representation produces redundant states" comes from Reiter's own comparison table.
   The six-state representation of this paper lies between these two.
2. In **Table 4 (parameters)**, there is only one road-side parameter, "road bounds" (road half-width),
   and its constraint acts on the reference point, not on the full vehicle contour. Therefore, in the original manuscript of this paper,
   the statement "both [26] and [20] fail to explicitly model the complete vehicle contour in
   road constraint formulations" holds for [20], but the formulation should be more precise.
3. **Table 5 and the main text** report the computation time and progress for each obstacle geometry form (ellipse, covering circle, separating hyperplane),
   and point out that the separating hyperplane form is worse in both time and progress.
   The reconstruction of this paper covers only the ellipse and the covering circle; the separating hyperplane form is not implemented.
4. **Section 2.3** states that the obstacle shape transformation in Frenet coordinates cannot guarantee a safety-conservative over-approximation under SQP,
   which is one of the core arguments of [20] and is consistent with the direction of this paper's criticism of "Frenet obstacle over-approximation".

These verification results allow the related work to be rewritten as sourced and measurable statements, replacing the originally unfounded
"dimensionality explosion" phrasing. A draft of the rewritten passage is in
`../../论文修改稿_评审意见2和3.md`.

## 10. Supplement: Verification Results Against the State Definitions of the Xing Text (2026-10-30)

Checked `Vehicle_Motion_Planning_With_Joint_Cartesian-Frent_MPC.pdf`:

- The original Eq. (2) defines the Cartesian state `p_c = [V, X, Y, psi]ᵀ` (including inertia),
  Eq. (3) defines the control `u = [delta, a]ᵀ`;
  the joint state of Eq. (21) is `[p_cᵀ, p_fᵀ]ᵀ`, with the Frenet part `p_f = [S, D, phi]ᵀ`
  (see Eq. (11)). The joint state is therefore 7-dimensional, consistent with the
  `[V, X, Y, psi, S, D, phi]` of this reconstruction.
- Eq. (9) gives a deterministic coupling (9a–9c) from the Frenet states to the Cartesian states,
  showing that `S` in the joint state can be recovered from `X, Y, psi`, and is therefore deterministic duplicate information.
  This is the same phenomenon as Reiter's original Table 2 listing the problem of the lifted representation as "redundant states".
- Therefore, in the original manuscript phrase "redundant coupling of Cartesian and Frenet states causes state-space
  dimensionality explosion", the first half (redundant coupling) is substantiated in both original texts,
  while the second half (explosion) is not substantiated in either original text. When rewriting, the first half should be kept, the second half removed,
  and the citation changed to this paper's own controlled ablation data.

## 11. Supplement: N_disc Contour Discretization Accuracy Sweep (2026-10-30)

A new `scripts/run_ndisc_sweep.py` directly quantifies the quantity on which the paper's Table 2/3 depend:
it compares the lateral offset of the corners given by the road boundary constraint model with the exact lateral offset of the
corners computed independently on a circular reference line, and computes the RMSE of each corner. The optimizer is not involved, so it reflects only the representation error.

Results (`results/ndisc_sweep/ndisc_sweep.csv`):

| Reference line | N_disc | RL | RR | FR | FL |
| --- | ---: | ---: | ---: | ---: | ---: |
| Straight road | arbitrary | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| Curve 1/60, integrated Δθ | 1 | 0.0192 | 0.2317 | 0.1991 | 0.0431 |
| Curve 1/60, integrated Δθ | 2 | 0.0202 | 0.1079 | 0.1060 | 0.0455 |
| Curve 1/60, integrated Δθ | 5 | 0.0211 | 0.0539 | 0.0900 | 0.0472 |
| Curve 1/60, integrated Δθ | 10 | 0.0214 | 0.0522 | 0.0968 | 0.0478 |
| Curve 1/60, integrated Δθ | 20 | 0.0216 | 0.0561 | 0.1021 | 0.0481 |
| Curve 1/60, frozen Δθ | arbitrary | 0.0192 | 0.2317 | 0.1991 | 0.0431 |

Observations:

1. On the straight road the error is zero and independent of N_disc, confirming the order of magnitude reported in the paper's Table 2.
2. On the curve, the error of the integrated Δθ decreases as the number of segments increases (the leading corner RR drops from 0.2317 m to the order of 0.052 m).
3. **The error of the frozen Δθ is completely independent of N_disc** — increasing the number of segments cannot improve it, because the error source is
   the heading deviation along the vehicle body not being integrated, rather than the integration accuracy. This directly supports the difference of contribution 1
   of this paper relative to [16], and also shows that the "N_disc dependence" property belongs only to the integrated version.
4. The error of the leading corner FR no longer decreases monotonically once N_disc >= 5, and rises slightly. This is the same class of phenomenon as the
   "RL/RR error slightly increasing from 5 to 10 in N_disc" described in reviewer comment 6.6, and indicates that this reconstruction shares the origin with the paper.
   **This phenomenon does not yet have a complete explanation**, and no conclusion should be drawn from it before the main text explains it clearly.

Units are metres. The curvature of the reference line, the sequence of heading deviations, and the sampling counts are recorded in the script arguments and in
`results/ndisc_sweep/ndisc_sweep.json`.

## 12. Supplement: A Cross-Validation of the Vehicle Parameters and Formulas (2026-10-30)

The parameters of the paper's Table 1 are used to check the printed formulas against the values actually used:

| Quantity | Computed from the parameters | Value in the paper/implementation | Conclusion |
| --- | --- | --- | --- |
| Minimum turning radius | l_wb / tan(δ_max) = 4.32 / tan(29.32°) = 7.692 m | Table 1 records 7.69 m | Consistent |
| Curvature upper bound 1/R_min | 0.1300 m⁻¹ | Eq. (34c) upper bound 0.13 m⁻¹ | Consistent |
| Curvature-rate upper bound (small-angle form) | δ̇_max / l_wb = 24.55°/s / 4.32 m = 0.0992 | Value used in the paper 0.099 | Consistent |
| Curvature-rate upper bound (as printed) | v · δ̇_max / l_wb, 1.190 when v = 12 m/s | Value used in the paper 0.099 | Inconsistent |

This independently confirms the judgement of reviewer comment 6.6 numerically: the form that actually takes effect is the small-angle form
δ̇_max / l_wb, whereas the printed v·δ̇_max/l_wb matches neither the dimensions nor the numbers.
The implementation of this round's reconstruction (`paper_comparison/model.py`) uses exactly
`steering_rate = l_wb · κ̇ / (1 + (l_wb · κ)²)`, which is consistent with the small-angle form.

Therefore the printed expressions of Eq. (34c) and (34e) need to be corrected; this is listed as an item to fix in the revision.
