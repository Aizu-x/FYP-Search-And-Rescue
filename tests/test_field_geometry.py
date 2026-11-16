"""Arena geometry taken from the map: where the walls are, corner start poses, the real size of the field."""
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from ugv_planner import Grid, load_environment
from ugv_nav.map_store import (CORNERS, corner_faces, corner_pose, example_points, inner_box, load_edited, load_env,
                               resample_to, save_edited, scaled_shape, validate_field_scale)
from ugv_nav.path_processing import validate_path

HALF_L, HALF_W, PLANNING_RADIUS = 0.11, 0.09, 0.163


class WallsFromTheMapTests(unittest.TestCase):
    def test_the_data_files_draw_a_field_of_about_2_76_m_inside_the_walls(self):
        for env in ('ENV1', 'ENV2', 'ENV3'):
            grid, gt = load_env(env)
            x0, x1, y0, y1 = inner_box(grid)
            self.assertAlmostEqual(x1 - x0, 2.76, delta=0.04, msg=env)        # NOT 3.00: the walls are inside the picture
            self.assertAlmostEqual(y1 - y0, 2.76, delta=0.04, msg=env)
            gx0, gx1, gy0, gy1 = inner_box(gt)                                # ground truth agrees within one cell
            for a, b in ((x0, gx0), (x1, gx1), (y0, gy0), (y1, gy1)):
                self.assertLessEqual(abs(a - b), 0.031, env)

    def test_inner_box_ignores_obstacles_and_rough_walls(self):
        cells = np.ones((60, 80), np.uint8)
        cells[5:55, 10:70] = 0                                               # floor
        cells[5:20, 10:16] = 1                                               # cupboard against the left wall
        cells[30, 10] = cells[31, 69] = 1                                    # rough spots
        x0, x1, y0, y1 = inner_box(Grid(cells))
        self.assertEqual([round(v, 3) for v in (x0, x1, y0, y1)], [0.30, 2.10, 0.15, 1.65])
        self.assertEqual(tuple(round(v, 3) for v in corner_faces(Grid(cells), 'tr')), (2.10, 1.65))
        self.assertEqual(tuple(round(v, 3) for v in corner_faces(Grid(cells), 'tl')), (0.30, 1.65))


class CornerStartTests(unittest.TestCase):
    def test_pose_is_measured_from_both_walls(self):
        cells = np.ones((100, 100), np.uint8)
        cells[4:96, 6:94] = 0                                                # walls at x 0.18 / 2.82, y 0.12 / 2.88
        grid = Grid(cells)
        self.assertEqual(corner_pose(grid, 'bl', 0, HALF_L, HALF_W, 0.10), (0.18 + 0.10 + 0.11, 0.12 + 0.10 + 0.09, 0.0))
        x, y, heading = corner_pose(grid, 'bl', 90, HALF_L, HALF_W, 0.10)    # turned: length is now along y
        self.assertEqual((round(x, 3), round(y, 3), heading), (0.37, 0.33, 90.0))
        x, y, heading = corner_pose(grid, 'tr', 185, HALF_L, HALF_W, 0.0)    # touching both walls, heading snapped
        self.assertEqual((round(x, 3), round(y, 3), heading), (2.71, 2.79, 180.0))
        self.assertEqual(corner_pose(grid, 'br', 270, HALF_L, HALF_W, 0.05)[2], 270.0)
        with self.assertRaises(ValueError):
            corner_pose(grid, 'middle', 0, HALF_L, HALF_W)

    def test_one_block_length_from_the_walls_is_a_valid_a_star_start(self):
        """10 cm = the long side of a WRO block: the operator needs no ruler."""
        for env in ('ENV1', 'ENV2', 'ENV3'):
            grid, _ = load_env(env)
            usable = 0
            for corner in CORNERS:
                poses = [corner_pose(grid, corner, heading, HALF_L, HALF_W, 0.10) for heading in (0, 90, 180, 270)]
                if all(validate_path([p[:2]], grid, PLANNING_RADIUS)[0] for p in poses):
                    usable += 1
            self.assertGreaterEqual(usable, 3, env)                           # (one corner may hold an obstacle)


class RealFieldSizeTests(unittest.TestCase):
    def test_same_size_is_the_same_map(self):
        grid, _ = load_env('ENV2')
        self.assertIs(resample_to(grid, grid.shape), grid)
        again, _ = load_env('ENV2', (1.0, 1.0))
        self.assertTrue(np.array_equal(again.cells, grid.cells))

    def test_stretching_the_map_to_the_real_size(self):
        base, _ = load_env('ENV1')
        bx0, bx1, by0, by1 = inner_box(base)
        scale = (3.00 / (bx1 - bx0), 2.90 / (by1 - by0))                     # the operator measured 3.00 x 2.90 m
        grid, gt = load_env('ENV1', scale)
        self.assertEqual(grid.shape, gt.shape)
        self.assertEqual(grid.shape, scaled_shape(base.shape, scale))
        self.assertAlmostEqual(grid.resolution, 0.03)                        # still 3 cm cells: sizes are real metres
        x0, x1, y0, y1 = inner_box(grid)
        self.assertAlmostEqual(x1 - x0, 3.00, delta=0.04)
        self.assertAlmostEqual(y1 - y0, 2.90, delta=0.04)
        self.assertAlmostEqual(grid.cells.mean(), base.cells.mean(), delta=0.03)      # same picture, other size
        start = example_points('ENV1', scale)[0]
        self.assertFalse(grid.cells[grid.world_to_cell(start)])

    def test_thin_walls_survive_and_do_not_double(self):
        cells = np.zeros((100, 100), np.uint8)
        cells[:, 37] = 1; cells[52, :] = 1
        for scale in ((0.8, 0.8), (0.93, 1.07), (1.09, 1.09), (1.3, 0.9), (1.6, 1.6)):
            out = resample_to(Grid(cells), scaled_shape(cells.shape, scale)).cells
            thick_v = out[5:int(40 * scale[1])].sum(axis=1)                   # the vertical wall, in rows away from the other
            thick_h = out[:, int(45 * scale[0]):].sum(axis=0)                 # the horizontal wall, right of the vertical one
            self.assertTrue(((thick_v >= 1) & (thick_v <= 2)).all(), (scale, thick_v.min(), thick_v.max()))
            self.assertTrue(((thick_h >= 1) & (thick_h <= 2)).all(), (scale, thick_h.min(), thick_h.max()))

    def test_absurd_sizes_are_refused(self):
        self.assertEqual(validate_field_scale((1.087, 0.95)), (1.087, 0.95))
        for bad in ((0.3, 1.0), (1.0, 2.5), (float('nan'), 1.0)):
            with self.assertRaises(ValueError):
                validate_field_scale(bad)

    def test_edited_maps_follow_the_field_size_and_the_data_files_never_change(self):
        before = {p.name: p.read_bytes() for p in Path(__file__).resolve().parents[1].joinpath('data').glob('*.npy')}
        with tempfile.TemporaryDirectory() as tmp:
            scale = (1.087, 1.087)
            grid, _ = load_env('ENV3', scale)
            cells = grid.cells.copy().astype(np.uint8)
            r, c = grid.world_to_cell((1.6, 1.2))
            cells[r - 1:r + 2, c - 1:c + 2] = 1                               # the operator painted an obstacle
            edited = Grid(cells, grid.resolution, grid.origin)
            save_edited('ENV3', edited, tmp, scale)
            self.assertTrue(np.array_equal(load_edited('ENV3', tmp, scale).cells, edited.cells))
            with self.assertRaises(ValueError):
                save_edited('ENV3', edited, tmp, (1.0, 1.0))                  # wrong size for that field
            old = load_edited('ENV3', tmp, (1.0, 1.0))                        # the field size was reset later
            self.assertEqual(old.shape, load_environment('ENV3')[0].shape)
            ro, co = old.world_to_cell((1.6 / 1.087, 1.2 / 1.087))
            self.assertTrue(old.cells[ro - 1:ro + 2, co - 1:co + 2].any())    # the painted obstacle is still there
        after = {p.name: p.read_bytes() for p in Path(__file__).resolve().parents[1].joinpath('data').glob('*.npy')}
        self.assertEqual(before, after)
        self.assertGreater(len(before), 5)
        self.assertTrue(all(math.isfinite(v) for v in inner_box(load_env('ENV3', scale)[0])))


if __name__ == '__main__':
    unittest.main()
