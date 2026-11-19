"""Read existing ORB-SLAM3 text exports; this does not run SLAM."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from preflight import read_rgb_list, manifest_index, sha256_file


def read_rows(path, columns):
    result = []
    for line_number, line in enumerate(Path(path).read_text().splitlines(), 1):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        values = list(map(float, line.split()))
        if len(values) != columns or not all(math.isfinite(v) for v in values):
            raise ValueError(f'{path}:{line_number}: expected {columns} finite numbers')
        result.append(values)
    if not result:
        raise ValueError(f'{path}: empty export; no usable result')
    return result


def read_exports(poses_path, points_path, rgb_list=None, collection_manifest=None):
    poses = read_rows(poses_path, 8)
    points = read_rows(points_path, 3)
    if any(b[0] <= a[0] for a, b in zip(poses, poses[1:])):
        raise ValueError('Keyframe timestamps must increase strictly for this single-sequence reader')
    for row in poses:
        norm = math.sqrt(sum(v*v for v in row[4:]))
        if abs(norm-1) > 0.01:
            raise ValueError('Quaternion is not approximately unit length')
    image_rows = read_rgb_list(rgb_list) if rgb_list else None
    refs = manifest_index(collection_manifest)
    keyframes = []
    for i, p in enumerate(poses):
        frame = {'export_row_index': i, 'timestamp_s': p[0],
                 'translation_slam_units': p[1:4], 'quaternion_xyzw': p[4:],
                 'estimated_frame_index_at_20fps': round(p[0]*20),
                 'frame_index_verified_against_rgb_txt': False}
        if image_rows is not None:
            matches = [r for r in image_rows if abs(r['timestamp_s'] - p[0]) <= 0.0000005]
            if len(matches) != 1:
                raise ValueError(f'Keyframe timestamp {p[0]} has no unique match in rgb.txt')
            row = matches[0]
            path = Path(rgb_list).parent / row['relative_path']
            frame.update({'input_frame_index': row['input_frame_index'],
                          'frame_index_verified_against_rgb_txt': True,
                          'image_relative_path': row['relative_path'],
                          'image_available': path.is_file()})
            if path.is_file():
                frame['image_sha256'] = sha256_file(path)
                if refs is not None:
                    ref = refs.get('datasets/ENV1/' + Path(rgb_list).parent.name + '/' + row['relative_path'])
                    frame['image_hash_matches_collection_manifest'] = bool(
                        ref and ref['sha256'] == frame['image_sha256'] and ref['bytes'] == path.stat().st_size)
                    if not frame['image_hash_matches_collection_manifest']:
                        raise ValueError(f'Image differs from original collection manifest: {path}')
        keyframes.append(frame)
    if rgb_list and refs is not None:
        ref = refs.get('datasets/ENV1/' + Path(rgb_list).parent.name + '/rgb.txt')
        if not ref or ref['sha256'] != sha256_file(rgb_list):
            raise ValueError('rgb.txt differs from original collection manifest')
    return {
        'operation': 'parse_existing_exports_not_run_slam',
        'status': 'completed',
        'coordinate_units': 'arbitrary_monocular_slam_units',
        'metric_calibrated': False,
        'pose_columns': ['timestamp_s', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'],
        'pose_convention': 'T_wc camera-to-world; confirmed in supplied custom System.cc SaveKeyFrameTrajectoryTUM',
        'export_map_scope': 'current Atlas map only; confirmed in hash-matched Atlas.cc',
        'world_axis_alignment': 'not registered to arena, gravity, ERFNet or UGV',
        'keyframe_id_available': False, 'point_id_available': False,
        'keyframe_count': len(poses), 'map_point_count': len(points),
        'poses_sha256': hashlib.sha256(Path(poses_path).read_bytes()).hexdigest(),
        'points_sha256': hashlib.sha256(Path(points_path).read_bytes()).hexdigest(),
        'rgb_list_sha256': sha256_file(rgb_list) if rgb_list else None,
        'keyframes': keyframes,
        'map_points_xyz': points,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--poses', type=Path, required=True)
    parser.add_argument('--points', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rgb-list', type=Path, help='Optional original manifest for exact keyframe-image association')
    parser.add_argument('--collection-manifest', type=Path, help='Optional hash verification for available images and rgb.txt')
    args = parser.parse_args()
    if args.output.exists():
        print('Output exists; choose a new file', file=sys.stderr)
        return 2
    started = time.perf_counter()
    try:
        if args.collection_manifest and not args.rgb_list:
            raise ValueError('--collection-manifest requires --rgb-list')
        result = read_exports(args.poses, args.points, args.rgb_list, args.collection_manifest)
        result['parse_seconds'] = time.perf_counter()-started
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x') as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
            stream.write('\n')
    except (OSError, ValueError, KeyError) as exc:
        print(json.dumps({'status': 'failed', 'error_type': type(exc).__name__, 'error': str(exc)}), file=sys.stderr)
        return 1
    print(json.dumps({k: result[k] for k in ('status', 'operation', 'keyframe_count', 'map_point_count', 'parse_seconds')}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
