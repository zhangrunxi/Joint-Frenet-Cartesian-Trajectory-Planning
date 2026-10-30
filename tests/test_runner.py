import importlib.util
import sys
from pathlib import Path

MODULE = Path(__file__).resolve().parents[1] / "scripts/run_checks.py"
spec = importlib.util.spec_from_file_location("run_checks", MODULE)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def prepare(monkeypatch, tmp_path):
    (tmp_path / "logs").mkdir()
    monkeypatch.setattr(runner, "ROOT", tmp_path)


def test_records_success_and_output(monkeypatch, tmp_path):
    prepare(monkeypatch, tmp_path)
    result = runner.run("success", [sys.executable, "-c", "print('ok')"], tmp_path)
    assert result["exit_code"] == 0
    assert result["timed_out"] is False
    assert "ok" in (tmp_path / result["log"]).read_text()


def test_preserves_failure_exit_code(monkeypatch, tmp_path):
    prepare(monkeypatch, tmp_path)
    result = runner.run(
        "failure", [sys.executable, "-c", "raise SystemExit(7)"], tmp_path
    )
    assert result["exit_code"] == 7
    assert result["timed_out"] is False


def test_timeout_is_not_success(monkeypatch, tmp_path):
    prepare(monkeypatch, tmp_path)
    result = runner.run(
        "timeout",
        [sys.executable, "-c", "import time; time.sleep(10)"],
        tmp_path,
        timeout=0.1,
    )
    assert result["exit_code"] is None
    assert result["timed_out"] is True
