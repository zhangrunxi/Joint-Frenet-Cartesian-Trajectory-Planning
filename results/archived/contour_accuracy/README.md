# Contour accuracy against the discretization count

This directory holds the sweep of the road-boundary model error against the
number of segments the vehicle contour is divided into.

## What is measured

For a pose on a circular reference line of curvature `kappa`, the model
predicts the lateral offset of each of the four vehicle corners by integrating
the heading deviation along the contour in `N` steps. The exact offset is
obtained from the ground truth. The error of a corner is the distance between
the two positions.

The manuscript defines the metric of Eq. (lateral_error) on the position
vector, `|| p_approx - p_actual ||`. In this sweep the position error and
the lateral offset error are numerically identical, because the model
displaces a corner only along the reference-line normal: the longitudinal
coordinates agree exactly. The two are therefore reported in separate
columns (`rmse_*` and `lateral_rmse_*`) so that a reader can check this
rather than take it on trust.

The `frozen_heading` rows replace the integrated heading deviation by a
constant one, which is the ablation of the manuscript's first contribution.

## The two curvatures

| File | Curvature | Note |
| --- | --- | --- |
| `ndisc_sweep_kappa0.1.csv` | 0.1 1/m | the curvature the manuscript prints for the curved-road experiment |
| `ndisc_sweep_kappa0.0167.csv` | 1/60 1/m | the curvature of the first sweep, kept because it is the one earlier numbers were quoted from |

## What this sweep does and does not reproduce

It does **not** reproduce the numbers of Table 3 of the manuscript, and the
reason is worth stating plainly:

- Table 3 reports the front-right corner RMSE falling from 0.0771 m at
  `N_disc = 5` to 0.0464 m at `N_disc = 10`. In this sweep the same corner
  *rises* slightly, from 9.42 to 9.54, and the absolute values are about two
  orders of magnitude larger.

  The unit of the swept numbers is the unit the model works in; the sweep
  accumulates the contour integration error over a 40 m stretch of reference
  line, which is why it is much larger than the per-pose error. The paper's
  Table 3 is a per-trajectory RMSE over the points of a trajectory, and the
  two are not the same quantity.
- The metric definition was checked against Eq. (lateral_error) and is not
  the cause: the position error and the lateral error agree exactly here.
- The manuscript does not define whether `N_disc` counts the points of the
  discretized contour or the segments between them, and the two differ by one.
- The swept heading deviations (`0.0`, `0.1`, `0.2` rad) and the lateral
  offset profile are this script's own choice; the manuscript does not print
  them.

So these numbers are the error of *this re-implementation* of the
contour model, under a stated configuration, and they are published so that
the reader can judge the discrepancy rather than having to trust either
side. They should not be quoted as the manuscript's Table 3.
