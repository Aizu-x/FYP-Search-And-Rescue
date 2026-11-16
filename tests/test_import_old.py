"""`nav_main.py import-old`: take the operator's own values from the previous project folder, read-only."""
import hashlib
import json
import shutil
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

import nav_main
from ugv_nav.config import DEFAULT_CONFIG, load_config


def tree_hash(folder):
    digest = hashlib.sha256()
    for path in sorted(Path(folder).rglob('*')):
        if path.is_file():
            digest.update(str(path.relative_to(folder)).encode() + b'\0' + path.read_bytes())
    return digest.hexdigest()


class ImportOldTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='ugv import test ')
        base = Path(self.tmp.name)
        self.old, self.new = base / 'old project', base / 'new project'
        # the "old" project: its own robot values, key, console settings and an edited map
        (self.old / 'ugv_planner').mkdir(parents=True)
        (self.old / 'config').mkdir()
        (self.old / 'data' / 'edited').mkdir(parents=True)
        text = DEFAULT_CONFIG.read_text(encoding='utf-8')
        for a, b in (('ugv_host = ""', 'ugv_host = "192.168.149.1"'), ('radius_m = 0.143', 'radius_m = 0.15'),
                     ('max_v = 0.15', 'max_v = 0.12'), ('drift_per_m = 0.05', 'drift_per_m = 0.02'),
                     ('max_sigma_m = 0.15', 'max_sigma_m = 0.2'), ('auto_continue_after_replan = false',
                                                                  'auto_continue_after_replan = true')):
            self.assertIn(a, text)
            text = text.replace(a, b)
        text = '\n'.join(line for line in text.splitlines()                  # an old file has none of the new keys
                         if not line.startswith(('smooth_path', 'engine', 'continuous_correction', 'period_s',
                                                 '[lidar_localization]')))
        (self.old / 'config' / 'navigation.toml').write_text(text, encoding='utf-8')
        (self.old / 'config' / 'mentorpi.key').write_text('ab' * 32 + '\n')
        (self.old / 'config' / 'gui_settings.json').write_text(json.dumps(dict(env='ENV3', radius='0.15')))
        np.save(self.old / 'data' / 'edited' / 'ENV3_occupancy_edited.npy', np.zeros((100, 100), np.uint8))
        # the "new" project folder (only what the import touches)
        (self.new / 'config').mkdir(parents=True)
        shutil.copy(DEFAULT_CONFIG, self.new / 'config' / 'navigation.toml')
        self.root, nav_main.ROOT = nav_main.ROOT, self.new

    def tearDown(self):
        nav_main.ROOT = self.root
        self.tmp.cleanup()

    def test_values_are_taken_over_and_the_old_folder_is_untouched(self):
        before = tree_hash(self.old)
        self.assertEqual(nav_main.import_old(SimpleNamespace(folder=str(self.old), force=False)), 0)
        self.assertEqual(tree_hash(self.old), before)                          # read-only on the old project
        cfg = load_config(self.new / 'config' / 'navigation.toml')
        self.assertEqual(cfg.network.ugv_host, '192.168.149.1')
        self.assertEqual(cfg.planner.footprint.radius_m, 0.15)
        self.assertEqual(cfg.navigator.controller.max_v, 0.12)
        self.assertEqual((cfg.navigator.localization.drift_per_m, cfg.navigator.localization.max_sigma_m), (0.02, 0.2))
        self.assertTrue(cfg.navigator.auto_continue_after_replan)
        self.assertTrue(cfg.navigator.smooth_path)                              # new options keep their defaults
        self.assertEqual(cfg.navigator.planner_engine, 'fast')
        self.assertTrue((self.new / 'config' / 'navigation.local.toml').is_file())   # your values live here ...
        self.assertEqual((self.new / 'config' / 'navigation.toml').read_bytes(),
                         DEFAULT_CONFIG.read_bytes())                           # ... the shipped file is untouched
        self.assertEqual((self.new / 'config' / 'mentorpi.key').read_text(), 'ab' * 32 + '\n')
        self.assertTrue((self.new / 'config' / 'gui_settings.json').is_file())
        self.assertTrue((self.new / 'data' / 'edited' / 'ENV3_occupancy_edited.npy').is_file())

    def test_existing_files_are_kept_unless_forced(self):
        (self.new / 'config' / 'mentorpi.key').write_text('cd' * 32 + '\n')
        nav_main.import_old(SimpleNamespace(folder=str(self.old), force=False))
        self.assertEqual((self.new / 'config' / 'mentorpi.key').read_text(), 'cd' * 32 + '\n')
        nav_main.import_old(SimpleNamespace(folder=str(self.old), force=True))
        self.assertEqual((self.new / 'config' / 'mentorpi.key').read_text(), 'ab' * 32 + '\n')

    def test_wrong_folders_are_refused(self):
        with self.assertRaises(SystemExit):
            nav_main.import_old(SimpleNamespace(folder=str(self.new), force=False))          # this folder itself
        with self.assertRaises(SystemExit):
            nav_main.import_old(SimpleNamespace(folder=self.tmp.name, force=False))          # not a project folder


if __name__ == '__main__':
    unittest.main()
