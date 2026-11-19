#!/usr/bin/env python3
"""Copy BOTH real 400-frame Tello sequences into small verified ZIPs.

Run on the original Ubuntu/WSL laptop. Python 3.10+; standard library only.
Copies actual PNG bytes, including when source paths are symlinks. Does not run
SLAM, modify source images, or overwrite an existing output directory.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import struct
import sys
import zipfile


def digest(data):
    return hashlib.sha256(data).hexdigest()


def collect_frames(root, reference=None):
    frames, manifests = [], []
    for name in ('video1_first20s', 'video2_first20s'):
        dataset = root / 'datasets/ENV1' / name
        manifest = dataset / 'rgb.txt'
        data = manifest.read_bytes()
        lines = data.decode('utf-8').splitlines()
        if len(lines) < 4 or not all(s.startswith('#') for s in lines[:3]):
            raise ValueError(f'{manifest}: expected three initial comment lines')
        rows = [s.split() for s in lines[3:] if s.strip()]
        if len(rows) != 400:
            raise ValueError(f'{name}: expected 400 frames, found {len(rows)} rows')
        manifest_arc = f'datasets/ENV1/{name}/rgb.txt'
        manifests.append({'path': manifest_arc, 'bytes': len(data), 'sha256': digest(data), 'data': data})
        names = set()
        for index, fields in enumerate(rows):
            if len(fields) != 2:
                raise ValueError(f'{manifest}: malformed row {index}')
            timestamp, relative = float(fields[0]), PurePosixPath(fields[1])
            if not math.isfinite(timestamp) or abs(timestamp - index / 20) > 1e-6:
                raise ValueError(f'{name}: expected timestamps index/20 starting at zero')
            if relative.is_absolute() or '..' in relative.parts or '\\' in fields[1] or relative.suffix.lower() != '.png':
                raise ValueError(f'{name}: invalid relative PNG path: {fields[1]}')
            if fields[1] in names:
                raise ValueError(f'{name}: repeated PNG filename: {fields[1]}')
            names.add(fields[1])
            path = dataset / fields[1]
            raw = path.read_bytes()  # Follows source symlinks, packages actual bytes.
            if len(raw) < 24 or raw[:8] != b'\x89PNG\r\n\x1a\n' or raw[12:16] != b'IHDR':
                raise ValueError(f'{path}: not a PNG with an IHDR header')
            if struct.unpack('>II', raw[16:24]) != (960, 720):
                raise ValueError(f'{path}: expected 960x720 pixels')
            frames.append({'path': f'datasets/ENV1/{name}/{fields[1]}', 'local_path': str(path),
                           'timestamp_s': timestamp, 'input_frame_index': index,
                           'bytes': len(raw), 'sha256': digest(raw)})
    if reference is not None:
        for record in manifests + frames:
            expected = reference.get(record['path'])
            if not expected or (expected['sha256'], expected['bytes']) != (record['sha256'], record['bytes']):
                raise ValueError(f'{record["path"]}: differs from original collection manifest')
    return frames, manifests


def make_parts(frames, manifests, output, max_bytes):
    # Leave 1 MiB per part for manifests, ZIP directory records and metadata.
    budget = max_bytes - 1024 * 1024
    groups, group, size = [], [], 0
    for record in frames:
        if record['bytes'] > budget:
            raise ValueError(f'Single image is larger than the requested part size: {record["path"]}')
        if group and size + record['bytes'] > budget:
            groups.append(group)
            group, size = [], 0
        group.append(record)
        size += record['bytes']
    if group:
        groups.append(group)
    output.mkdir(parents=True, exist_ok=False)
    index = {'status': 'incomplete', 'source_images_modified': False, 'frames': len(frames),
             'image_format': 'PNG 960x720', 'timestamp_units': 'sequence-relative seconds',
             'fps': 20, 'expected_parts': len(groups), 'parts': [],
             'instructions': 'Extract every part into the SAME directory. Repeated rgb.txt files are byte-identical. '
                             'A part is not a complete SLAM sequence. Run preflight after extraction.'}
    index_path = output / 'PACKAGE_INDEX.json'
    index_path.write_text(json.dumps(index, indent=2) + '\n')
    for part_no, records in enumerate(groups, 1):
        name = f'ORB_Frames_Part_{part_no:02d}_of_{len(groups):02d}.zip'
        target = output / name
        public_records = [{k: v for k, v in r.items() if k != 'local_path'} for r in records]
        with zipfile.ZipFile(target, 'x', compression=zipfile.ZIP_STORED, allowZip64=True) as z:
            for manifest in manifests:
                z.writestr(manifest['path'], manifest['data'])
            for record in records:
                data = Path(record['local_path']).read_bytes()
                if len(data) != record['bytes'] or digest(data) != record['sha256']:
                    raise RuntimeError(f'Source changed during packaging: {record["path"]}')
                z.writestr(record['path'], data)
            z.writestr(f'FRAME_PART_{part_no:02d}.json', json.dumps(
                {'part': part_no, 'total_parts': len(groups), 'frames': public_records}, indent=2) + '\n')
        with zipfile.ZipFile(target) as z:
            if z.testzip() is not None:
                raise RuntimeError(f'ZIP verification failed: {target}')
            if sum(n.endswith('.png') for n in z.namelist()) != len(records):
                raise RuntimeError(f'PNG count mismatch: {target}')
        if target.stat().st_size > max_bytes:
            raise RuntimeError(f'ZIP exceeds requested maximum: {target}')
        index['parts'].append({'filename': name, 'png_count': len(records),
                               'bytes': target.stat().st_size, 'sha256': digest(target.read_bytes())})
        index_path.write_text(json.dumps(index, indent=2) + '\n')
        print(f'Verified {name}: {len(records)} actual PNGs, {target.stat().st_size} bytes', flush=True)
    index['status'] = 'complete'
    index_path.write_text(json.dumps(index, indent=2) + '\n')
    return index


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path.home() / 'FYP_SLAM')
    parser.add_argument('--output', type=Path, required=True, help='New directory; must not already exist')
    parser.add_argument('--max-mib', type=int, default=45, help='Maximum ZIP size; default 45 MiB')
    parser.add_argument('--manifest', type=Path, help='Optional original COLLECTION_MANIFEST.json for hash checking')
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f'Refusing to overwrite existing output directory: {args.output}')
    if not 3 <= args.max_mib <= 100:
        raise ValueError('--max-mib must be between 3 and 100')
    reference = None
    if args.manifest:
        reference = {r['path']: r for r in json.loads(args.manifest.read_text())['files']}
    print('Checking all 800 original frames before creating any ZIP...', flush=True)
    frames, manifests = collect_frames(args.root.expanduser(), reference)
    result = make_parts(frames, manifests, args.output.expanduser(), args.max_mib * 1024 * 1024)
    print(f'SUCCESS: {result["frames"]} PNG frames in {len(result["parts"])} verified ZIP parts.')
    print(f'Upload every ZIP in: {args.output.expanduser().resolve()}')
    print('The original source files were not modified.')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('CANCELLED: source images unchanged. Any output marked incomplete is not ready.', file=sys.stderr)
        sys.exit(130)
    except (OSError, ValueError, KeyError, RuntimeError, zipfile.BadZipFile) as exc:
        print(f'STOP: {exc}\nSource images unchanged. Any output marked incomplete is not ready.', file=sys.stderr)
        sys.exit(1)
