"""Own one native process and stop it when the worker's stdin lifeline closes.

Production ORB execution is Linux-only. Windows support here is for direct-child
test fixtures; it is not a Windows/WSL ORB launcher. No shell commands are used.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from dag_scheduler.engine import atomic_json


def write_status(path, report):
    """Retry a short file-sharing conflict while preserving atomic publication.

    A Windows reader can temporarily prevent replacing an open status file.
    Persistent permission failures still propagate; they never imply success.
    """
    deadline = time.monotonic() + 1.0
    while True:
        try:
            atomic_json(path, report)
            return
        except PermissionError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.01)


def supervise(request):
    cancelled = threading.Event()

    def watch_lifeline():
        try:
            while os.read(0, 1):
                pass
        finally:
            cancelled.set()

    threading.Thread(target=watch_lifeline, daemon=True).start()
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, lambda *_: cancelled.set())
    folder = Path(request["directory"])
    status_path = folder / "process_status.json"
    report = {"request_id": request["request_id"], "state": "STARTING",
              "termination_confirmed": False, "return_code": None,
              "command": request["command"]}
    write_status(status_path, report)
    process = None
    cause = None
    started = time.monotonic()
    try:
        with (folder / "run.log").open("wb") as log:
            if cancelled.is_set():
                cause = "CANCELLED"
            else:
                process = subprocess.Popen(request["command"], cwd=folder,
                    stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=(os.name == "posix"))
                report.update(state="RUNNING", pid=process.pid)
                write_status(status_path, report)
                while process.poll() is None:
                    if cancelled.is_set():
                        cause = "CANCELLED"
                        break
                    if time.monotonic() >= request["deadline_monotonic"]:
                        cause = "TIMED_OUT"
                        break
                    time.sleep(0.025)
                if cause:
                    if os.name == "posix":
                        try:
                            os.killpg(process.pid, signal.SIGTERM)
                        except ProcessLookupError:
                            pass
                    elif process.poll() is None:
                        process.terminate()
                    try:
                        process.wait(timeout=1.5)
                    except subprocess.TimeoutExpired:
                        if os.name == "posix":
                            os.killpg(process.pid, signal.SIGKILL)
                        else:
                            process.kill()
                        process.wait(timeout=2)
                    # Clear any remaining members of the owned Linux process group.
                    if os.name == "posix":
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                else:
                    process.wait()
                report["return_code"] = process.returncode
        report["termination_confirmed"] = process is None or process.poll() is not None
        report["state"] = cause or ("COMPLETED" if process.returncode == 0 else "FAILED")
    except BaseException as error:
        report.update(state="FAILED", error=f"{type(error).__name__}: {error}")
        if process is not None and process.poll() is None:
            try:
                if os.name == "posix":
                    os.killpg(process.pid, signal.SIGKILL)
                else:
                    process.kill()
                process.wait(timeout=2)
            except (OSError, subprocess.TimeoutExpired):
                pass
        report["termination_confirmed"] = process is None or process.poll() is not None
    report["duration_s"] = time.monotonic() - started
    write_status(status_path, report)
    return 0 if report["state"] == "COMPLETED" else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("request", type=Path)
    arguments = parser.parse_args()
    raise SystemExit(supervise(json.loads(arguments.request.read_text(encoding="utf-8"))))
