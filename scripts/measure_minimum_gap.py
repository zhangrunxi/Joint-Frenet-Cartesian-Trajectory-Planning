"""Minimum passable gap of each obstacle geometry, from its own constraint.

Two identical buses are aligned on a straight road. For each longitudinal
offset in [-12, 12] m the smallest lateral centre distance at which the
geometry's constraint is satisfied is found by bisection; the worst offset is
reported. The gap the ego bus needs between the obstacle's left edge and the
road boundary is that distance plus the road margin minus nothing else:
g_min = d + 0.2 (half widths cancel, see NARROW_BENCHMARK_PROTOCOL.md).

Usage:
    python scripts/measure_minimum_gap.py
"""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'scripts')]
import numpy as np, casadi as ca
from paper_comparison.model import hybrid_constraints, capsule_distance, bus_vehicle, gym_pose
from paper_comparison.vehicle import GEOMETRY
from reiter2023.geometry import covering_circles, ellipse_constraint
v=bus_vehicle()
P=ca.MX.sym('p',3); Q=ca.MX.sym('q',3)
def f(expr): return ca.Function('f',[P,Q],[ca.mmin(expr)])
geo={
 'hybrid(25 pairs)': f(hybrid_constraints(P,Q,0.2)),
 'capsule Eq23': f(capsule_distance(P,Q,0.2)),
 'ellipse (Reiter)': f(ellipse_constraint(gym_pose(P),gym_pose(Q),v,v,0.2)),
}
def circ(n):
    aa,ra=covering_circles(gym_pose(P),v,n); bb,rb=covering_circles(gym_pose(Q),v,n)
    return f(ca.vertcat(*[ca.sqrt(ca.sumsqr(a-b)+1e-16)-ra-rb-0.2 for a in aa for b in bb]))
geo['3 circles']=circ(3); geo['5 circles']=circ(5)
def rect(p,q):  # exact aligned rectangles, rear axle origin, overhang 1.0
    dx=abs(q[0]-p[0]); dy=abs(q[1]-p[1]); return np.hypot(max(dx-GEOMETRY.length,0),max(dy-GEOMETRY.width,0))-0.2
dxs=np.linspace(-12,12,481)
print(GEOMETRY)
for name,fn in list(geo.items())+[('exact rectangle',None)]:
    need=0
    for dx in dxs:
        lo,hi=0,10
        for _ in range(40):
            m=(lo+hi)/2
            val = rect([0,0,0],[dx,m,0]) if fn is None else float(fn([0,0,0],[dx,m,0]))
            lo,hi=(lo,m) if val>=0 else (m,hi)
        need=max(need,hi)
    print(f'{name:18s} lateral centre distance {need:.3f} m -> g_min {need+0.2:.2f} m')
