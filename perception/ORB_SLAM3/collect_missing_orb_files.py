"""Read-only source collection on the original WSL laptop; never runs SLAM.

Copies existing source, vocabulary, settings, full 400-frame datasets and logs
to a NEW ZIP. It cannot reconstruct files that are missing on that laptop.
No source is edited; no package, service, or robot command is run.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile


def probe(command, cwd=None):
    try:
        p = subprocess.run(command, cwd=cwd, capture_output=True, text=True, timeout=20)
        return {'command': command, 'returncode': p.returncode, 'stdout': p.stdout, 'stderr': p.stderr}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'command': command, 'error': str(exc)}


def manifest_sequence(folder):
    manifest = folder/'rgb.txt'
    if not manifest.is_file():
        raise FileNotFoundError(manifest)
    paths, rows = [manifest], []
    for number, line in enumerate(manifest.read_text().splitlines(), 1):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) != 2:
            raise ValueError(f'{manifest}:{number}: expected timestamp and relative image path')
        timestamp = float(parts[0])
        relative = Path(parts[1])
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Sequence paths must stay within their dataset directory')
        p = folder/relative
        if not p.is_file():
            raise FileNotFoundError(p)
        paths.append(p)
        rows.append({'timestamp_s': timestamp, 'relative_path': relative.as_posix()})
    if len(rows) != 400:
        raise ValueError(f'{folder}: expected the documented 400-frame subset, found {len(rows)}')
    return paths, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path.home()/'FYP_SLAM')
    parser.add_argument('--output', type=Path, default=Path.home()/'FYP_ORB_SLAM3_Complete_Source_And_Examples.zip')
    args = parser.parse_args()
    root = args.root.expanduser().resolve()
    repo = root/'ORB_SLAM3'
    if args.output.exists():
        raise FileExistsError(f'Refusing to overwrite {args.output}')
    required = ['include/System.h', 'src/System.cc', 'Examples/Monocular/mono_tum.cc',
                'Examples/Monocular/Tello_ENV1_provisional.yaml', 'CMakeLists.txt', 'build.sh']
    missing = [str(repo/p) for p in required if not (repo/p).is_file()]
    voc = [p for p in (repo/'Vocabulary/ORBvoc.txt', repo/'Vocabulary/ORBvoc.txt.tar.gz') if p.is_file()]
    if not voc:
        missing.append(str(repo/'Vocabulary/ORBvoc.txt (or ORBvoc.txt.tar.gz)'))
    if missing:
        raise FileNotFoundError('Missing required original files:\n'+'\n'.join(missing))
    files = {}
    excluded = {'.git', 'build', '.venv', '__pycache__', 'CMakeFiles'}
    for current, dirs, names in os.walk(repo):
        dirs[:] = [d for d in dirs if d not in excluded]
        for name in names:
            p = Path(current)/name
            if p.is_file() and not name.endswith(('.pyc', 'Zone.Identifier')):
                files['ORB_SLAM3/'+p.relative_to(repo).as_posix()] = p
    datasets = {}
    for video in ('video1_first20s', 'video2_first20s'):
        folder = root/'datasets/ENV1'/video
        paths, rows = manifest_sequence(folder)
        for p in paths:
            files['datasets/ENV1/'+video+'/'+p.relative_to(folder).as_posix()] = p
        datasets[video] = rows
    for run in ('video1_3D_export', 'video2_3D_export', 'video2_shutdown_fixed', 'video2_stability_test'):
        folder = root/'results/ENV1'/run
        for name in ('run.log', 'KeyFrameTrajectory.txt', 'MapPoints.xyz'):
            p = folder/name
            if p.is_file():
                files[f'results/ENV1/{run}/{name}'] = p
    probes = [probe(['git', 'rev-parse', 'HEAD'], repo), probe(['git', 'status', '--short'], repo),
              probe(['cmake', '--version']), probe(['c++', '--version']),
              probe(['pkg-config', '--modversion', 'opencv4']), probe(['pkg-config', '--modversion', 'pangolin'])]
    executable = repo/'Examples/Monocular/mono_tum'
    if executable.is_file():
        probes.append(probe(['ldd', str(executable)]))
    manifest = {'created_utc': datetime.now(timezone.utc).isoformat(), 'sources_modified': False,
                'collection_only_not_execution_test': True, 'probes': probes, 'dataset_rows': datasets,
                'files': []}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    partial = args.output.with_suffix(args.output.suffix+'.partial')
    if partial.exists():
        raise FileExistsError(partial)
    with zipfile.ZipFile(partial, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for name, path in sorted(files.items()):
            data = path.read_bytes()
            z.writestr(name, data)
            manifest['files'].append({'path': name, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()})
        z.writestr('COLLECTION_MANIFEST.json', json.dumps(manifest, indent=2))
    with zipfile.ZipFile(partial) as z:
        if z.testzip() is not None:
            raise RuntimeError('Archive CRC check failed')
    # Hard-link creation is exclusive, preventing a late overwrite of a prior output.
    os.link(partial, args.output)
    partial.unlink()
    print(f'Collected {len(files)} original files: {args.output}')
    print('No source files changed. SLAM execution was not tested by this collector.')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'STOP: {exc}', file=sys.stderr)
        sys.exit(1)
