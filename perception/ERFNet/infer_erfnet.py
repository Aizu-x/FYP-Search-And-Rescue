"""Portable file inference using the supplied, unchanged ERFNet/refinement model.

New handoff entry point (2026-11-07). It adds explicit completion metadata and
original-image numerical masks. It implements no SLAM fusion or path planner.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import signal
import sys
import time
from datetime import datetime, timezone

import numpy as np
import torch
from PIL import Image
from erfnet_tools.common import load_model
from neural_inference import infer
from projection import project_probabilities
from grid_export import aggregate_semantics
from refine_semantics import refine

ROOT = Path(__file__).resolve().parent


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def save_labels(path, labels):
    Image.fromarray(np.asarray(labels, dtype=np.uint8)).save(path)


def process_image(model, image_path, out, stem, rotation=0, scene=None,
                  input_root=None, settings=None):
    started = time.perf_counter()
    image_path = Path(image_path)
    p, geometry = infer(model, image_path, size=640, quarter_turns=rotation)
    x, y, w, h = (geometry[k] for k in ('x', 'y', 'width', 'height'))
    raw_content = p[:, y:y+h, x:x+w].argmax(0).astype(np.uint8)
    original = Image.fromarray(raw_content).resize(tuple(geometry['original_size']),
                                                 Image.Resampling.NEAREST)
    original_name = f'{stem}_raw_labels_original.png'
    original.save(out / original_name)
    record = {
        'sample_id': stem, 'source_image': str(image_path),
        'source_sha256': sha256(image_path), 'capture_timestamp': None,
        'capture_timestamp_note': 'Not provided; processing time is not camera capture time.',
        'geometry': geometry, 'original_mask': original_name,
        'mask_frame': 'EXIF-normalized source image; x right, y down; top-left origin',
        'original_mask_shape_hw': [original.height, original.width],
        'class_ids': {'0': 'free_space', '1': 'blocked', '2': 'obstacle', '255': 'ignore'},
        'original_mask_values': np.unique(np.asarray(original)).tolist(),
        'method': 'raw ERFNet',
    }
    if scene is not None:
        warped, support = project_probabilities(p, geometry, scene['homography_original_to_800'])
        with Image.open(input_root / scene['valid_roi']) as im:
            roi = np.asarray(im) > 0
        with Image.open(input_root / scene['topdown_photo']) as im:
            rgb = np.asarray(im.convert('RGB'))
        if roi.shape != support.shape or rgb.shape != (800, 800, 3):
            raise ValueError('Saved scene ROI/photo dimensions must match the 800x800 projection')
        support &= roi
        if not support.any():
            raise ValueError('No valid arena support; a usable map cannot be produced')
        t = time.perf_counter()
        labels, operations = refine(rgb, warped, support, settings)
        refine_seconds = time.perf_counter() - t
        raw = warped.argmax(0).astype(np.uint8)
        raw[~support] = 255
        palette = np.zeros((256, 3), np.uint8)
        palette[0], palette[1], palette[2], palette[255] = (0, 200, 0), (255, 165, 0), (255, 0, 0), (210, 210, 210)
        for tag, mask in (('raw', raw), ('refined', labels)):
            save_labels(out / f'{stem}_{tag}_labels_topdown.png', mask)
            Image.fromarray(palette[mask]).save(out / f'{stem}_{tag}_colours_topdown.png')
        photo_grid, _ = aggregate_semantics(labels)
        planning_labels = np.rot90(labels, scene['planning_rotation_ccw_quarter_turns']).copy()
        planning_grid, _ = aggregate_semantics(planning_labels)
        np.save(out / f'{stem}_occupancy_photo_frame.npy', photo_grid, allow_pickle=False)
        np.save(out / f'{stem}_occupancy_planning_frame.npy', planning_grid, allow_pickle=False)
        record.update({
            'method': 'ERFNet with automatic appearance-guided semantic-map refinement',
            'annotation_assisted_roi': True, 'manual_interior_labels_used': False,
            'homography_original_to_800': scene['homography_original_to_800'],
            'refined_topdown_mask': f'{stem}_refined_labels_topdown.png',
            'refined_topdown_sha256': sha256(out / f'{stem}_refined_labels_topdown.png'),
            'refinement_seconds': refine_seconds, 'refinement_operations': operations,
            'grid': {
                'photo_frame_file': f'{stem}_occupancy_photo_frame.npy',
                'planning_frame_file': f'{stem}_occupancy_planning_frame.npy',
                'shape_rc': [60, 60], 'dtype': 'uint8',
                'cell_meanings': {'0': 'free', '1': 'occupied', '255': 'unknown_nontraversable'},
                'nominal_resolution_m_per_cell': 0.05, 'metric_scale_measured': False,
                'array_origin': 'top-left', 'row_direction': 'down', 'column_direction': 'right',
                'nominal_xy_origin': 'bottom-left of the selected grid frame',
                'nominal_xy_axes': 'x right, y up; not registered to SLAM or robot odometry',
                'cell_center_formula': 'x=(col+0.5)*0.05; y=(60-row-0.5)*0.05',
                'planning_rotation_ccw_quarter_turns': scene['planning_rotation_ccw_quarter_turns'],
                'robot_footprint_included': False, 'map_margin_cells': 0,
                'dynamic_obstacle_updates_implemented': False,
            },
        })
    record['processing_seconds'] = time.perf_counter() - started
    record['processing_finished_utc'] = utc_now()
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument('--image', type=Path, help='Raw segmentation of any single image')
    choice.add_argument('--scene', choices=['ENV1', 'ENV2', 'ENV3', 'all'],
                        help='Replay the supplied calibrated-by-annotation saved scene(s)')
    parser.add_argument('--inputs', type=Path, default=ROOT / 'inputs')
    parser.add_argument('--checkpoint', type=Path, default=ROOT / 'models/erfnet_continuation_selected.pth')
    parser.add_argument('--settings', type=Path, default=ROOT / 'config/refinement_settings.json')
    parser.add_argument('--output', type=Path, required=True, help='New directory; existing outputs are never reused')
    parser.add_argument('--threads', type=int, default=2)
    args = parser.parse_args()
    if args.output.exists():
        print('Output directory already exists; choose a new --output', file=sys.stderr)
        return 2
    if args.threads < 1:
        parser.error('--threads must be positive')
    started = time.perf_counter()
    report = {'status': 'running', 'started_utc': utc_now(), 'samples': [],
              'execution_ready': False, 'scope': 'offline perception and optional semantic grid export'}
    def cancel(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, cancel)
    code = 0
    try:
        args.output.mkdir(parents=True, exist_ok=False)
        write_json(args.output / 'run_metadata.json', report)
        torch.set_num_threads(args.threads)
        t = time.perf_counter()
        model, meta = load_model(args.checkpoint)
        model.eval()
        report['model_load_seconds'] = time.perf_counter() - t
        report['checkpoint'] = meta
        report['threads'] = args.threads
        if args.image:
            report['samples'].append(process_image(model, args.image, args.output, args.image.stem))
        else:
            config = json.loads((args.inputs / 'scene_geometry.json').read_text())
            settings = json.loads(args.settings.read_text())
            scenes = [s for s in config['environments'] if args.scene in ('all', s['environment'])]
            if not scenes:
                raise ValueError('No selected scene in scene_geometry.json')
            for scene in scenes:
                report['samples'].append(process_image(
                    model, args.inputs / scene['source_photo'], args.output, scene['environment'],
                    scene['input_rotation_ccw_quarter_turns'], scene, args.inputs, settings))
                write_json(args.output / 'run_metadata.json', report)
                print(scene['environment'] + ' processed', flush=True)
        report['status'] = 'completed'
    except KeyboardInterrupt:
        report['status'] = 'cancelled'
        code = 130
    except Exception as exc:
        report.update(status='failed', error_type=type(exc).__name__, error=str(exc))
        print(f'{type(exc).__name__}: {exc}', file=sys.stderr)
        code = 1
    finally:
        report['wall_seconds'] = time.perf_counter() - started
        report['finished_utc'] = utc_now()
        if args.output.is_dir():
            write_json(args.output / 'run_metadata.json', report)
    print(json.dumps({'status': report['status'], 'output': str(args.output)}), flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
