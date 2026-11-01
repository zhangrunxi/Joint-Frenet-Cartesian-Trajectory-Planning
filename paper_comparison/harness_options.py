"""Harness options shared by every planner of the comparison.

First-frame initial guess (``BENCH_INITIAL_GUESS``)
---------------------------------------------------

``rollout`` (the frozen-protocol default) leaves each planner its own cold
start: the proposed model and the Xing reconstruction integrate the initial
state forward at zero input, which drives straight through an obstacle ahead,
and the Reiter planner starts from the acados default iterate.

``ramp`` gives every planner the same guess: constant speed along the
reference line, with the lateral offset moving linearly from the initial
offset to the lateral target over the first half of the horizon. It carries
the side decision that an upstream behaviour layer (the manuscript's state
lattice) would supply, and that the Xing corridor already has built in.

Selected with ``BENCH_INITIAL_GUESS``; recorded in every protocol.json.
"""

import os

INITIAL_GUESS = os.environ.get("BENCH_INITIAL_GUESS", "rollout")
if INITIAL_GUESS not in ("rollout", "ramp"):
    raise ValueError(f"BENCH_INITIAL_GUESS must be rollout or ramp, got {INITIAL_GUESS}")


def lateral_ramp(k, n_intervals, start, target):
    """Lateral offset of the ramp guess at node ``k``."""
    fraction = min(1.0, k / max(1.0, n_intervals / 2))
    return start + (target - start) * fraction


# Contour discretisation of the proposed road constraint (Section 4.2). The
# frozen protocol uses 5; ``BENCH_N_DISC`` runs the sensitivity study.
N_DISC = int(os.environ.get("BENCH_N_DISC", "5"))


# Acceptance of a solve. The frozen protocol accepts only a converged solve
# (status 0). ``BENCH_ACCEPT_MAXITER=1`` also accepts an iterate that hit the
# iteration cap (status 2) while meeting the same 1e-4 violation tolerance,
# which is what a real-time MPC would execute. It applies to every method.
ACCEPT_MAXITER = os.environ.get("BENCH_ACCEPT_MAXITER", "0") == "1"


def accepted_status(status):
    return status == 0 or (ACCEPT_MAXITER and status == 2)


# Implementation details of the Xing reconstruction that the paper does not
# fix. ``BENCH_XING_ACCEPT_INACCURATE=1`` keeps an OSQP "solved inaccurate"
# iterate instead of aborting the SQP; the nonlinear residual check still
# decides acceptance. ``BENCH_XING_FREEZE_CORRIDOR=1`` decides the keep-left
# wall once per frame from the warm start instead of at every SQP iteration,
# which removes the corridor switching between iterations.
XING_ACCEPT_INACCURATE = os.environ.get("BENCH_XING_ACCEPT_INACCURATE", "0") == "1"
XING_FREEZE_CORRIDOR = os.environ.get("BENCH_XING_FREEZE_CORRIDOR", "0") == "1"
