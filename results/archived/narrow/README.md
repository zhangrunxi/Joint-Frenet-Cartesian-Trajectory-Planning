# Narrow-road benchmark, bus and SUV

These runs test the regime the manuscript is written for: a long vehicle on a
narrow, strongly curved road, where the error and the conservatism of an
approximate geometry decide whether a manoeuvre is possible at all. The
protocol, including the predictions, was frozen before the runs; see
`docs/NARROW_BENCHMARK_PROTOCOL.md`. Wide-road results are kept in
`../random_benchmark/` as the contrast case: on a wide road all methods do
well and none of them is limited by its geometry.

Every method, the evaluator and the scenario generator use one vehicle per
run, selected with `BENCH_VEHICLE` (`paper_comparison/vehicle.py`). The
obstacle is a vehicle of the same type.

| Vehicle | Body | Wheelbase | Rear overhang |
| --- | --- | ---: | ---: |
| Bus (Table 1 of the manuscript) | 6.50 m x 2.30 m | 4.32 m | 1.00 m |
| SUV (representative mid-size) | 4.80 m x 1.90 m | 2.85 m | 1.00 m |

## Gap task (`bus_gap/`, `suv_gap/`)

A stopped vehicle 0.3 m from the right curb leaves a gap `g` on the left. The
grid covers nine gaps, four curvatures and two speeds (72 cases per vehicle).

A case counts as a **pass with margin** only if the run completes without
collision, the ego vehicle ends up past the obstacle, and the obstacle
clearance never falls below the 0.2 m margin every method is asked for (with
a 0.01 m tolerance). Following the obstacle without collision is not a pass.
Grazing the obstacle with a geometry that does not cover the body is not a
pass with margin either.

| Method | Bus, passes | Bus, narrowest gap (straight / curve) | SUV, passes | SUV, narrowest gap (straight / curve) |
| --- | ---: | --- | ---: | --- |
| This paper, hybrid | **41/72** | **3.6** / 4.0 m | **45/72** | **2.8** / **2.8** m |
| This paper, circles (3 or 5) | 38/72 | 4.0 / 4.0 m | 38/72 | 3.3 / 3.3 m |
| Reiter, circles (direct / lifted) | 38/72 | 4.0 / 4.0 m | 38 / 36 of 72 | 3.3 / 3.3 m |
| Xing, keep-left corridor | 37/72 | 4.5 / **3.6** m | 42/72 | 3.3 / **2.8** m |
| This paper, capsule only (Eq. 23) | 30/72 | 4.0 / 4.5 m | 39/72 | 2.8 / 3.3 m |
| Reiter, ellipse (direct / lifted) | 2 / 2 of 72 | none / 6.0 m | 7 / 2 of 72 | 4.8 / 4.4 m |

What the table does and does not say:

- The ellipse cannot pass in these gaps. Its lateral extent is fixed by the
  covering of the whole body.
- **The proposed hybrid geometry is not the exact rectangle.** Its corner
  circles reach 1.39 m from the axis of a 2.30 m wide bus, so its minimum gap
  is 3.18 m against 2.70 m for the exact rectangle
  (`scripts/measure_minimum_gap.py`).
- On the curved bus cases at `g = 3.6 m`, Xing's rectangle corridor passes
  3 of 6 and the hybrid none. This is the one gap where a compared method
  passes and the proposed one does not.
- The capsule alone (Eq. 23) passes narrow gaps without collision but closer
  than the margin (29 such passes per vehicle, clearance 0.001 m to 0.185 m),
  because it does not cover the corners.
  It counts as a pass in the `passed` column and not in `passed_with_margin`.

Most Xing failures are aborts: in the first frame, or at the step where the
keep-left wall of its corridor (Eq. 14) switches on. Its acceptance rule
(solver success and violation at most 1e-4) is the same as that of the
acados-based methods. Its SQP cap was 50 iterations against 150 for acados,
so `xing150/` repeats every Xing run with a cap of 150. **The results are
identical**, so the cap is not the cause.

## Curb task (`bus_curb/`, `suv_curb/`)

The ego vehicle follows the outer curb of a narrow curved road, with the
lateral target set at the boundary a reference-point model allows. This
pushes every planner against its own road constraint. The score is the exact
clearance between the body rectangle and the curb.

| Method | Bus: curb strikes | Bus: completed | Bus: min clearance | SUV: curb strikes | SUV: completed | SUV: min clearance |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| This paper, deformed contour | **0/32** | **32/32** | 0.125 m | **0/32** | **28/32** | 0.146 m |
| This paper, frozen heading | 32/32 | 0/32 | -0.047 m | 32/32 | 0/32 | -0.025 m |
| Reiter, reference-point bound (published) | 32/32 | 0/32 | -0.030 m | 32/32 | 0/32 | -0.024 m |
| Xing, reference-point bound (Eq. 12, published) | 32/32 | 0/32 | -0.044 m | 32/32 | 0/32 | -0.026 m |
| Xing, rectangle corridor bound | 0/32 | 0/32 | 0.200 m | 0/32 | 3/32 | 0.200 m |

The Reiter row with the deformed contour of this paper is in the CSV and
matches the proposed method; it borrows the proposed constraint and is
therefore not an independent result.

What the table does and does not say:

- Every reference-point bound and the frozen-heading ablation strike the curb
  on every curve, because they do not account for the outward swing of the
  front overhang.
- The deformed contour never strikes the curb, but it **does not keep the full
  0.2 m margin**. The contour is integrated in five segments, and its error
  takes up to 0.075 m of the margin for the bus (20 m radius, 5 m road) and
  up to 0.054 m for the SUV (12 m radius, 3.5 m lane).
- Xing's rectangle corridor keeps the margin exactly, but its SQP aborts
  before completing the run in almost every case.
- The four SUV failures of the deformed contour are the 3.5 m lane at an 8 m
  radius. There, the outer front corner of the starting pose is already
  0.145 m inside the margin, so the first frame is infeasible for any exact
  body constraint. The scenario generator checks the lateral acceleration of
  the starting pose but not its body clearance. This is a defect of the
  scenario, and it is reported rather than removed.

## Re-running it

```bash
BENCH_VEHICLE=bus bash scripts/run_random_benchmark.sh --task gap --output results/recheck_gap
BENCH_VEHICLE=suv bash scripts/run_random_benchmark.sh --task curb --output results/recheck_curb
.venv/bin/python scripts/summarize_narrow_benchmark.py --task gap \
    --input results/recheck_gap --output results/recheck_gap/summary.csv
```

Each `part*/` directory holds one slice of the case list. The slices were run
as parallel processes, which is why this benchmark does not report solve
times.
