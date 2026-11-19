"""Check the supplied Tello monocular inputs without starting ORB-SLAM3.

Python standard library only. Checks PNG headers and optional original hashes;
does not decode images, estimate calibration, build C++, or test tracking.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import struct
import sys

ROOT = Path(__file__).resolve().parent


def sha256_file(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read_rgb_list(path):
    lines = Path(path).read_text().splitlines()
    if len(lines) < 4 or not all(s.startswith('#') for s in lines[:3]):
        raise ValueError('rgb.txt needs exactly three initial comment lines for this mono_tum loader')
    rows = []
    names = set()
    for line_no, line in enumerate(lines[3:], 4):
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) != 2:
            raise ValueError(f'{path}:{line_no}: expected timestamp and relative image filename')
        stamp = float(fields[0])
        rel = PurePosixPath(fields[1])
        if not math.isfinite(stamp) or stamp < 0:
            raise ValueError(f'{path}:{line_no}: invalid timestamp')
        if rel.is_absolute() or '..' in rel.parts or '\\' in fields[1]:
            raise ValueError(f'{path}:{line_no}: invalid relative filename')
        if rows and stamp <= rows[-1]['timestamp_s']:
            raise ValueError('rgb.txt timestamps must increase strictly')
        if fields[1] in names:
            raise ValueError('rgb.txt contains a repeated image filename')
        names.add(fields[1])
        rows.append({'input_frame_index': len(rows), 'timestamp_s': stamp,
                     'relative_path': fields[1]})
    if not rows:
        raise ValueError('rgb.txt has no image rows')
    return rows


def png_dimensions(path):
    with Path(path).open('rb') as stream:
        header = stream.read(24)
    if len(header) != 24 or header[:8] != b'\x89PNG\r\n\x1a\n' or header[12:16] != b'IHDR':
        raise ValueError(f'{path}: not a PNG with an IHDR header')
    return struct.unpack('>II', header[16:24])


def manifest_index(path):
    if path is None:
        return None
    data = json.loads(Path(path).read_text())
    return {r['path']: r for r in data['files']}


def check_dataset(dataset, references=None):
    dataset = Path(dataset)
    rows = read_rgb_list(dataset / 'rgb.txt')
    prefix = 'datasets/ENV1/' + dataset.name
    errors, missing, present = [], [], []
    if len(rows) != 400:
        errors.append(f'Expected 400 frames for this supplied sequence; found {len(rows)}')
    if any(abs(r['timestamp_s'] - r['input_frame_index'] / 20) > 1e-6 for r in rows):
        errors.append('Expected sequence-relative 20 FPS timestamps starting at zero')
    checked_paths = [('rgb.txt', dataset / 'rgb.txt')]
    for row in rows:
        path = dataset / row['relative_path']
        if not path.is_file():
            missing.append(row['relative_path'])
            continue
        try:
            if png_dimensions(path) != (960, 720):
                errors.append(f'{row["relative_path"]}: expected 960x720 pixels')
        except (OSError, ValueError) as exc:
            errors.append(str(exc))
        present.append(dict(row))
        checked_paths.append((row['relative_path'], path))
    hash_records = {}
    if references is not None:
        for rel, path in checked_paths:
            ref = references.get(prefix + '/' + rel)
            digest = sha256_file(path)
            match = bool(ref and ref['sha256'] == digest and ref['bytes'] == path.stat().st_size)
            hash_records[rel] = {'sha256': digest, 'matches_collection_manifest': match}
            if not match:
                errors.append(f'{rel}: missing reference or mismatch against collection manifest')
    if missing:
        errors.append(f'{len(missing)} referenced frame files are missing')
    for row in present:
        row.update(hash_records.get(row['relative_path'], {}))
    return {'dataset': prefix, 'manifest_rows': len(rows), 'available_frames': len(present),
            'missing_frames': len(missing), 'missing_paths': missing,
            'available_frame_details': present, 'errors': errors,
            'status': 'inputs_valid' if not errors else 'failed',
            'rgb_list_hash_check': hash_records.get('rgb.txt')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, action='append', required=True,
                        help='May be repeated for Video 1 and Video 2')
    parser.add_argument('--collection-manifest', type=Path)
    parser.add_argument('--vocabulary', type=Path, default=ROOT / 'source/ORB_SLAM3/Vocabulary/ORBvoc.txt')
    parser.add_argument('--settings', type=Path, default=ROOT / 'source/ORB_SLAM3/Examples/Monocular/Tello_ENV1_provisional.yaml')
    parser.add_argument('--binary', type=Path, help='Optionally check a locally compiled mono_tum exists; not executed')
    parser.add_argument('--output', type=Path, help='Optional JSON report, must be a new file')
    args = parser.parse_args()
    if args.output and args.output.exists():
        print('Output exists; choose a new file', file=sys.stderr)
        return 2
    report = {'operation': 'validate_inputs_not_run_slam', 'status': 'failed',
              'build_or_tracking_tested': False, 'datasets': [], 'errors': []}
    try:
        refs = manifest_index(args.collection_manifest)
        for dataset in args.dataset:
            try:
                result = check_dataset(dataset, refs)
                report['datasets'].append(result)
                report['errors'].extend(f'{dataset.name}: {e}' for e in result['errors'])
            except (OSError, ValueError) as exc:
                report['errors'].append(f'{dataset}: {exc}')
        for name, path, key in (
            ('vocabulary', args.vocabulary, 'ORB_SLAM3/Vocabulary/ORBvoc.txt'),
            ('settings', args.settings, 'ORB_SLAM3/Examples/Monocular/Tello_ENV1_provisional.yaml'),
            ('binary', args.binary, None),
        ):
            if path is None:
                continue
            if not path.is_file() or path.stat().st_size == 0:
                report['errors'].append(f'{name} is missing or empty: {path}')
            elif refs is not None and key:
                ref = refs.get(key)
                if not ref or sha256_file(path) != ref['sha256'] or path.stat().st_size != ref['bytes']:
                    report['errors'].append(f'{name} does not match the supplied collection manifest')
        report['status'] = 'inputs_valid' if not report['errors'] else 'failed'
    except (OSError, ValueError, KeyError) as exc:
        report['errors'].append(f'{type(exc).__name__}: {exc}')
    if args.output:
        try:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open('x') as stream:
                json.dump(report, stream, indent=2, allow_nan=False)
                stream.write('\n')
        except OSError as exc:
            print(json.dumps({'status': 'failed', 'error': str(exc)}), file=sys.stderr)
            return 1
    print(json.dumps({'status': report['status'], 'operation': report['operation'],
                      'available_frames': sum(x['available_frames'] for x in report['datasets']),
                      'missing_frames': sum(x['missing_frames'] for x in report['datasets']),
                      'errors': report['errors']}))
    return 0 if report['status'] == 'inputs_valid' else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('{"status":"cancelled","operation":"validate_inputs_not_run_slam"}', file=sys.stderr)
        sys.exit(130)
