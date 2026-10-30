"""Source-verified Xing corridor in the same six historical width/target pairs."""

import argparse
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        raise FileExistsError("Use a new output directory")
    args.output.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env.update(
        ACADOS_SOURCE_DIR=str(ROOT / "vendor/acados"),
        DYLD_LIBRARY_PATH=str(ROOT / "vendor/acados/lib"),
        OPENBLAS_NUM_THREADS="1",
        OMP_NUM_THREADS="1",
        VECLIB_MAXIMUM_THREADS="1",
    )
    for width in ("4.0", "4.3", "4.6", "4.9", "5.2", "5.5"):
        command = [
            sys.executable,
            str(ROOT / "scripts/run_final_comparison.py"),
            "--steps",
            "120",
            "--methods",
            "xing_joint_corridor",
            "--half-width",
            width,
            "--lateral",
            str(float(width) - 2),
            "--output",
            str(args.output / f"half_width_{width}"),
        ]
        subprocess.run(command, env=env, check=True, cwd=ROOT)


if __name__ == "__main__":
    main()
