"""Summarize the CURRENT (latest-setting) benchmark results.

Latest setting (all methods): shared ramp initial guess, iteration-cap acceptance
when violation <= 1e-4, node-0 fix in the proposed SQP, N_DISC=5 (gap/overtake)
or 10 (curb), Xing accept-inaccurate + frozen corridor.
Sources: results/v2/* ; methods re-run after the Xing safety fix / Reiter N_DISC
fix in results/rerun_fixed/* override the same method in v2.

Usage: python3 scripts/summarize_current_results.py [--root results]
(public repo: --root results/current)
"""
import argparse
import glob
import json
from collections import defaultdict

V2, RR = "results/v2", "results/rerun_fixed"
SUV_CURB_INVALID = frozenset({0, 1, 8, 9})  # exact body already < 0.2 m from curb at t=0


def jsonl(path):
    return [json.loads(l) for l in open(path) if l.startswith("{")]


def load(name):
    rows, cases = [], {}
    for p in sorted(glob.glob(f"{V2}/{name}/part*/")):
        cases.update({c["case_id"]: c for c in json.load(open(p + "cases.json"))})
        rows += jsonl(p + "partial_results.jsonl")
    over = defaultdict(list)
    for p in sorted(glob.glob(f"{RR}/{name}/*_part*/")):
        for r in jsonl(p + "partial_results.jsonl"):
            over[r["method"]].append(r)
    rows = [r for r in rows if r["method"] not in over] + [r for v in over.values() for r in v]
    return rows, cases, sorted(over)


def gap_ok(r):
    c = r.get("min_obstacle_clearance_m")
    return bool(r.get("success") and r.get("passed_obstacle") and c is not None and c >= 0.19)


def table_pass(name):
    rows, cases, over = load(name)
    if not rows:
        return
    by = defaultdict(list)
    for r in rows:
        by[r["method"]].append(r)
    print(f"\n## {name}  (rerun_fixed override: {', '.join(over) or '-'})")
    print("| method | pass | straight | curve | collision/exit |")
    print("| --- | ---: | ---: | ---: | ---: |")
    for m, rs in sorted(by.items(), key=lambda kv: -sum(map(gap_ok, kv[1]))):
        st = [r for r in rs if cases[r["case_id"]]["kappa"] == 0]
        cu = [r for r in rs if cases[r["case_id"]]["kappa"] != 0]
        f = lambda xs: f"{sum(map(gap_ok, xs))}/{len(xs)}"
        col = sum(bool(r.get("collision_or_road_exit")) for r in rs)
        print(f"| {m} | {f(rs)} | {f(st)} | {f(cu)} | {col} |")


def table_curb(name, invalid=frozenset()):
    rows, _, over = load(name) if not name.startswith("ablation") else (
        jsonl(f"{V2}/{name}/partial_results.jsonl"), None, [])
    if not rows:
        return
    rows = [r for r in rows if r["case_id"] not in invalid]
    by = defaultdict(list)
    for r in rows:
        by[r["method"]].append(r)
    print(f"\n## {name}  (excluded invalid cases: {sorted(invalid) or '-'}; override: {', '.join(over) or '-'})")
    print("| method | completed | strikes (<0) | <0.19 m | min clearance m |")
    print("| --- | ---: | ---: | ---: | ---: |")
    for m, rs in by.items():
        cl = [r["min_road_clearance_m"] for r in rs if r.get("min_road_clearance_m") is not None]
        mn = f"{min(cl):.3f}" if cl else "-"
        print(f"| {m} | {sum(bool(r['success']) for r in rs)}/{len(rs)} | "
              f"{sum(c < 0 for c in cl)} | {sum(c < 0.19 for c in cl)} | {mn} |")


def table_general(name):
    rows = jsonl(f"{V2}/{name}/partial_results.jsonl")
    by = defaultdict(list)
    for r in rows:
        by[r["method"]].append(r)
    print(f"\n## {name}")
    print("| method | completed | collision/exit | median wall ms |")
    print("| --- | ---: | ---: | ---: |")
    for m, rs in sorted(by.items(), key=lambda kv: -sum(bool(r["success"]) for r in kv[1])):
        t = sorted(r["mean_solve_wall_ms"] for r in rs if r.get("mean_solve_wall_ms"))
        med = f"{t[len(t) // 2]:.2f}" if t else "-"
        print(f"| {m} | {sum(bool(r['success']) for r in rs)}/{len(rs)} | "
              f"{sum(bool(r.get('collision_or_road_exit')) for r in rs)} | {med} |")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="results", help="directory holding v2/ and rerun_fixed/")
    root = ap.parse_args().root
    V2, RR = f"{root}/v2", f"{root}/rerun_fixed"
    for veh in ("bus", "suv"):
        table_pass(f"{veh}_gap2")
        table_pass(f"{veh}_overtake2")
        table_curb(f"{veh}_curb", SUV_CURB_INVALID if veh == "suv" else frozenset())
    table_general("ablation_general")
    table_curb("ablation_bus_curb")
    table_curb("ablation_suv_curb", SUV_CURB_INVALID)
    for m in ("serial9_paper6_hybrid_sqp", "serial9_paper9_hybrid_sqp"):
        table_general(m)
