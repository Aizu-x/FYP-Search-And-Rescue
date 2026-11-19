from pathlib import Path
import sys
import time

from orb_adapter import execute_command


def long_native(inputs, context):
    return execute_command([sys.executable, "-c", "import time; time.sleep(60)"],
                           Path(context.directory) / "native", context)


def short_recording(inputs, context):
    return {"recording_id": "fixture"}


def empty_exports(inputs, context):
    from orb_io.read_exports import read_exports
    folder = Path(context.directory)
    (folder / "poses.txt").write_text("", encoding="utf-8")
    (folder / "points.xyz").write_text("1 2 3\n", encoding="utf-8")
    return read_exports(folder / "poses.txt", folder / "points.xyz")
