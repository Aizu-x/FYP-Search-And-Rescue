"""Verify and install ORB_assets.zip from this project's GitHub release."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import stat
import zipfile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("orb_installer", ROOT / "DAG_scheduler/install_orb_handoff.py")
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)


def install(archive, root=ROOT):
    expected = json.loads((ROOT / "assets/ORB_ASSETS.json").read_text(encoding="utf-8"))
    if archive.stat().st_size != expected["bytes"] or installer.sha(archive) != expected["sha256"]:
        raise ValueError("ORB asset ZIP differs from this release's verified archive")
    with zipfile.ZipFile(archive) as package:
        entries = [p for p in package.infolist() if not p.is_dir()]
        if len(entries) != expected["files"] or len({p.filename for p in entries}) != len(entries):
            raise ValueError("Unexpected or duplicate archive members")
        targets = []
        for entry in entries:
            if not entry.filename.startswith("ORB_handoff/") or stat.S_ISLNK(entry.external_attr >> 16):
                raise ValueError("Unexpected ORB archive path or symbolic link")
            targets.append((entry, installer.destination(root, entry.filename)))
        for entry, target in targets:
            installer.write_checked(target, package.read(entry))
    print("ORB assets installed and verified. No SLAM program was executed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    args = parser.parse_args()
    try:
        install(args.archive.resolve())
    except (OSError, ValueError, zipfile.BadZipFile) as error:
        parser.exit(1, f"Installation failed: {error}\n")
