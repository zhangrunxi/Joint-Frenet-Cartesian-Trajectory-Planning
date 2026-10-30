#!/bin/bash
# Copy the publishable sources from the working experiment directory into this
# repository, so the released code never drifts from the code that produced the
# archived results. Run from the openrepo root.
#
# The experiment directory keeps the paper PDFs outside the published tree and
# refers to them with a fallback path; this repository has no paper PDFs, so the
# three references are rewritten to tolerate their absence after every sync.
set -euo pipefail

SOURCE="${1:?usage: sync_from_workspace.sh <experiment-dir>}"
TARGET="$(cd "$(dirname "$0")/.." && pwd)"

if [ ! -d "$SOURCE/paper_comparison" ]; then
  echo "not an experiment directory: $SOURCE" >&2
  exit 1
fi

for item in paper_comparison reiter2023; do
  rm -rf "$TARGET/$item"
  cp -R "$SOURCE/$item" "$TARGET/$item"
done
rm -rf "$TARGET/tests" "$TARGET/scripts"
cp -R "$SOURCE/tests" "$TARGET/tests"
cp -R "$SOURCE/scripts" "$TARGET/scripts"
chmod +x "$TARGET/scripts"/*.sh
cp "$SOURCE/requirements-comparison.lock" "$TARGET/"

# The documentation in docs/ is the English text of the public repository and
# is the single source of truth for it. The working experiment directory keeps
# the Chinese originals under docs/zh/, which are refreshed from the source
# directory here; the English files are never overwritten by a sync.
mkdir -p "$TARGET/docs/zh"
for doc in FINAL_COMPARISON_REPORT.md ACCEPTANCE_REPORT.md AUDIT_FINDINGS.md RECONSTRUCTION.md RANDOM_BENCHMARK_PROTOCOL.md; do
  [ -f "$SOURCE/$doc" ] && cp "$SOURCE/$doc" "$TARGET/docs/zh/$doc"
done

# The paper PDFs are copyrighted and are not redistributed. Make the three
# references to them optional so the runners work without the PDFs.
TARGET="$TARGET" "$TARGET/.venv/bin/python" - <<'PY'
import os
from pathlib import Path

target = Path(os.environ["TARGET"])

final = target / "scripts" / "run_final_comparison.py"
text = final.read_text()
text = text.replace(
    '        "xing_paper_sha256": hashlib.sha256(paper.read_bytes()).hexdigest(),',
    '        "xing_paper_sha256": (\n'
    '            hashlib.sha256(paper.read_bytes()).hexdigest() if paper.exists() else None\n'
    '        ),\n'
    '        "xing_paper_available": paper.exists(),',
)
final.write_text(text)

verify = target / "scripts" / "verify_final_comparison.py"
text = verify.read_text()
text = text.replace(
    "        assert sha(paper) == provenance[\"xing_paper_sha256\"]",
    "        if provenance.get(\"xing_paper_sha256\"):\n"
    "            assert paper.exists(), \"Recorded paper digest but no local PDF\"\n"
    "            assert sha(paper) == provenance[\"xing_paper_sha256\"]",
)
verify.write_text(text)

paper_comp = target / "scripts" / "run_paper_comparison.py"
text = paper_comp.read_text()
old = (
    '        "paper_sha256": hashlib.sha256(\n'
    '            (\n'
    '                ROOT.parents[1] / "Joint_Frenet_Cartesian_Trajectory_Planning.pdf"\n'
    '            ).read_bytes()\n'
    '        ).hexdigest(),'
)
new = (
    '        # The manuscript PDF is not redistributed with the experiment code.\n'
    '        "paper_sha256": None,'
)
text = text.replace(old, new)
paper_comp.write_text(text)
PY

find "$TARGET" -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null || true
echo "synced from $SOURCE"
