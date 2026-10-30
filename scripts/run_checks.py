"""Record original-paper blockers separately from current portable checks."""

import json
import os
import platform
import subprocess
import sys
from pathlib import Path
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
GYM = ROOT / "vendor/vehicle_gym"
ORIGINAL = ROOT / "vendor/obstacle_avoidance"


def git(repo, *args):
    return subprocess.check_output(["git", "-C", str(repo), *args], text=True).strip()


def run(name, command, cwd, timeout=300):
    env = dict(
        os.environ,
        OPENBLAS_NUM_THREADS="1",
        OMP_NUM_THREADS="1",
        VECLIB_MAXIMUM_THREADS="1",
        MPLBACKEND="Agg",
    )
    start = perf_counter()
    log = ROOT / "logs" / f"{name}.log"
    with log.open("w") as stream:
        stream.write(f"cwd: {cwd}\ncommand: {command!r}\n")
        stream.flush()
        try:
            result = subprocess.run(
                command,
                check=False,
                cwd=cwd,
                env=env,
                stdout=stream,
                stderr=subprocess.STDOUT,
                timeout=timeout,
            )
            code, timed_out = result.returncode, False
        except subprocess.TimeoutExpired:
            code, timed_out = None, True
    receipt = {
        "command": command,
        "cwd": str(cwd),
        "exit_code": code,
        "timed_out": timed_out,
        "elapsed_s": perf_counter() - start,
        "log": str(log.relative_to(ROOT)),
    }
    print(name, json.dumps(receipt), flush=True)
    return receipt


def main():
    for directory in ["logs", "results/original", "results/portable"]:
        (ROOT / directory).mkdir(parents=True, exist_ok=True)
    tracked_paths = git(GYM, "ls-tree", "-r", "--name-only", "HEAD").splitlines()
    legacy_module = "trajectory_planner_acados_20222"
    history = git(GYM, "log", "--all", "--format=%H", "--", f"*{legacy_module}*")
    report = {
        "platform": platform.platform(),
        "python": sys.version,
        "repositories": {
            "obstacle_avoidance": {
                "url": git(ORIGINAL, "remote", "get-url", "origin"),
                "commit": git(ORIGINAL, "rev-parse", "HEAD"),
            },
            "vehicle_gym": {
                "url": git(GYM, "remote", "get-url", "origin"),
                "commit": git(GYM, "rev-parse", "HEAD"),
            },
        },
        "source_audit": {
            "required_original_planner_module": legacy_module,
            "matching_current_paths": [p for p in tracked_paths if legacy_module in p],
            "matching_history_commits": history.splitlines(),
            "scope": "Fetched refs only; not proof that unpublished code does not exist.",
        },
        "checks": {},
        "paper20_reproduced": False,
        "portable_is_paper20_baseline": False,
        "author_algorithm_ported": False,
    }
    checks = report["checks"]
    checks["original_script"] = run(
        "original_script",
        [sys.executable, str(ORIGINAL / "comparisons_short.py")],
        ROOT / "results/original",
    )
    checks["portable_tests"] = run(
        "portable_tests",
        [sys.executable, "-m", "pytest", "-q", str(GYM / "tests/unit")],
        ROOT,
    )
    checks["portable_demo"] = run(
        "portable_demo",
        [
            sys.executable,
            "-m",
            "vehiclegym.demo",
            "--output",
            str(ROOT / "results/portable"),
        ],
        ROOT,
    )
    report["portability_status"] = (
        "portable_backend_only"
        if all(checks[c]["exit_code"] == 0 for c in ["portable_tests", "portable_demo"])
        else "portable_checks_incomplete_or_failed"
    )
    (ROOT / "results/feasibility.json").write_text(json.dumps(report, indent=2) + "\n")
    if (
        checks["portable_tests"]["exit_code"] != 0
        or checks["portable_demo"]["exit_code"] != 0
    ):
        return 1
    # A successful portable demo must not turn a failed paper reproduction green.
    return 0 if checks["original_script"]["exit_code"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
