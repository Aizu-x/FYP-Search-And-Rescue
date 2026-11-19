"""Run scheduler/adapter tests separately from the upstream tests namespace."""
from pathlib import Path
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


if __name__ == "__main__":
    scratch = ROOT / "verification/local/tmp"
    scratch.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, TEMP=str(scratch), TMP=str(scratch), TMPDIR=str(scratch))
    command = [sys.executable, "-B", "-m", "unittest", "discover", "-s", "tests", "-v"]
    result = subprocess.run(command, cwd=ROOT / "DAG_scheduler", env=environment,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=240)
    (ROOT / "verification/local/dag_tests.txt").write_text(result.stdout, encoding="utf-8")
    print(result.stdout)
    raise SystemExit(result.returncode)
