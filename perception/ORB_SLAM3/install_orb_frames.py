#!/usr/bin/env python3
"""Install the two verified frame ZIPs beside this handoff's ORB source.

Python 3.10+, standard library only. Reads original image bytes; never changes
the supplied archives or overwrites a different existing file. Does not run SLAM.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import sys
import zipfile
from preflight import sha256_file

ROOT = Path(__file__).resolve().parent


def checked_destination(destination, name):
    rel = PurePosixPath(name)
    if rel.is_absolute() or '..' in rel.parts or '\\' in name:
        raise ValueError(f'Invalid archive path: {name}')
    path = destination / name
    if not path.resolve().is_relative_to(destination):
        raise ValueError(f'Path escapes the destination: {name}')
    return path


def install_frames(video1, video2, destination=ROOT, manifest=ROOT / 'evidence/COLLECTION_MANIFEST.json'):
    destination = Path(destination).expanduser().resolve()
    refs = {r['path']: r for r in json.loads(Path(manifest).read_text())['files']}
    jobs = [('video1', Path(video1)), ('video2', Path(video2))]
    # Validate BOTH archives and all existing destination files before writing.
    for video, source in jobs:
        prefix = f'datasets/ENV1/{video}_first20s/'
        expected = {name for name in refs if name.startswith(prefix)}
        if len(expected) != 401 or sum(n.endswith('.png') for n in expected) != 400:
            raise ValueError(f'{video}: reference manifest does not describe 400 PNGs and rgb.txt')
        with zipfile.ZipFile(source) as z:
            names = [i.filename for i in z.infolist() if not i.is_dir()]
            if len(names) != len(set(names)) or set(names) != expected:
                raise ValueError(f'{source}: expected exactly the original {video} dataset files')
            for name in names:
                target = checked_destination(destination, name)
                data = z.read(name)  # Includes ZIP CRC verification.
                ref = refs[name]
                if len(data) != ref['bytes'] or hashlib.sha256(data).hexdigest() != ref['sha256']:
                    raise ValueError(f'Archive file differs from original manifest: {name}')
                if target.exists() and (not target.is_file() or sha256_file(target) != ref['sha256']):
                    raise ValueError(f'Refusing to overwrite a different existing file: {target}')
    installed, reused = 0, 0
    for _, source in jobs:
        with zipfile.ZipFile(source) as z:
            for info in z.infolist():
                if info.is_dir():
                    continue
                target = checked_destination(destination, info.filename)
                ref = refs[info.filename]
                if target.exists():
                    if not target.is_file() or sha256_file(target) != ref['sha256']:
                        raise ValueError(f'Existing file changed during installation: {target}')
                    reused += 1
                    continue
                data = z.read(info)
                if len(data) != ref['bytes'] or hashlib.sha256(data).hexdigest() != ref['sha256']:
                    raise ValueError(f'Archive changed during installation: {source}')
                target.parent.mkdir(parents=True, exist_ok=True)
                created = False
                try:
                    with target.open('xb') as stream:
                        created = True
                        stream.write(data)
                except BaseException:
                    if created:
                        target.unlink(missing_ok=True)
                    raise
                installed += 1
    return {'status': 'completed', 'operation': 'install_original_frames_not_run_slam',
            'destination': str(destination), 'total_frames': 800,
            'total_files': 802, 'installed_files': installed, 'reused_identical_files': reused}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video1', type=Path, required=True)
    parser.add_argument('--video2', type=Path, required=True)
    parser.add_argument('--destination', type=Path, default=ROOT,
                        help='ORB_SLAM3 directory under which datasets/ will be installed')
    args = parser.parse_args()
    try:
        result = install_frames(args.video1, args.video2, args.destination)
    except (OSError, ValueError, KeyError, zipfile.BadZipFile) as exc:
        print(json.dumps({'status': 'failed', 'error': str(exc),
                          'note': 'Source ZIPs unchanged. Any partial installation needs another input preflight.'}), file=sys.stderr)
        return 1
    print(json.dumps(result))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('{"status":"cancelled","note":"Source ZIPs unchanged; rerun installation and preflight."}', file=sys.stderr)
        sys.exit(130)
