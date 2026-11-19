"""Install only ORB assets from the three supplied ZIPs; never execute their code."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import stat
import zipfile


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def destination(root, name):
    relative = PurePosixPath(name)
    if relative.is_absolute() or ".." in relative.parts or "\\" in name or ":" in name:
        raise ValueError(f"Unsafe ZIP path: {name}")
    result = (root / str(relative)).resolve()
    if not result.is_relative_to(root.resolve()):
        raise ValueError(f"ZIP path escapes destination: {name}")
    return result


def write_checked(path, data):
    if path.exists():
        if not path.is_file() or sha(path) != hashlib.sha256(data).hexdigest():
            raise ValueError(f"Existing file differs; preserving it: {path}")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(data)


def install(package, video1, video2, root):
    root = Path(root).resolve()
    with zipfile.ZipFile(package) as archive:
        matches = [i.filename for i in archive.infolist() if i.filename.endswith("/HANDOFF_FILES.json")]
        if len(matches) != 1:
            raise ValueError("Expected one HANDOFF_FILES.json")
        prefix = matches[0][:-len("HANDOFF_FILES.json")]
        descriptor = json.loads(archive.read(matches[0]))
        refs = json.loads(archive.read(prefix + "ORB_SLAM3/evidence/COLLECTION_MANIFEST.json"))
        manifest = {row["path"]: row for row in refs["files"]}
        # Check both full archive hashes before writing any files.
        for path, expected in zip((video1, video2), descriptor["companion_archives"]):
            if Path(path).stat().st_size != expected["bytes"] or sha(path) != expected["sha256"]:
                raise ValueError(f"Companion archive failed the supplied hash check: {path}")
        names = set()
        selected = []
        for item in archive.infolist():
            if item.is_dir() or not item.filename.startswith(prefix + "ORB_SLAM3/"):
                continue
            name = item.filename[len(prefix):]
            if name in names or stat.S_ISLNK(item.external_attr >> 16):
                raise ValueError(f"Duplicate or symbolic-link entry: {name}")
            names.add(name)
            selected.append((item, destination(root, name)))
        for item, target in selected:
            write_checked(target, archive.read(item))
        write_checked(root / "HANDOFF_FILES.json", archive.read(matches[0]))
    # The expected original manifest provides exact membership and per-file hashes.
    for number, video in enumerate((video1, video2), 1):
        prefix = f"datasets/ENV1/video{number}_first20s/"
        expected_names = {name for name in manifest if name.startswith(prefix)}
        with zipfile.ZipFile(video) as archive:
            names = [item.filename for item in archive.infolist() if not item.is_dir()]
            if len(expected_names) != 401 or len(names) != len(set(names)) or set(names) != expected_names:
                raise ValueError(f"Video {number}: expected exactly 400 PNGs and rgb.txt")
            for name in names:
                data = archive.read(name)
                expected = manifest[name]
                if len(data) != expected["bytes"] or hashlib.sha256(data).hexdigest() != expected["sha256"]:
                    raise ValueError(f"Image/manifest differs from original: {name}")
                write_checked(destination(root / "ORB_SLAM3", name), data)
    report = {"status": "installed", "source_archives_changed": False,
              "frames": 800, "frame_files_hash_verified": 802,
              "orb_root": str(root / "ORB_SLAM3"), "slam_executed": False,
              "main_archive_sha256": sha(package)}
    (root / "installation_check.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", type=Path, required=True)
    parser.add_argument("--video1", type=Path, required=True)
    parser.add_argument("--video2", type=Path, required=True)
    parser.add_argument("--destination", type=Path, default=Path(__file__).resolve().parent.parent / "ORB_handoff")
    args = parser.parse_args()
    try:
        print(json.dumps(install(args.package, args.video1, args.video2, args.destination), indent=2))
        return 0
    except (OSError, ValueError, KeyError, zipfile.BadZipFile) as error:
        print(f"Installation failed: {error}. Existing different files are not overwritten.")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
