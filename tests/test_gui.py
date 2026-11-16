"""Real Tk widget/callback integration tests. Skipped on hosts without a display."""
import unittest
import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import tkinter as tk
from ugv_planner.gui import Application
from ugv_planner.grid import dataset_manifest


class GUIIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.root = tk.Tk(); cls.root.withdraw()
        except tk.TclError as exc:
            raise unittest.SkipTest(f'Display unavailable: {exc}')
        cls.errors = []
        cls.root.report_callback_exception = lambda *args: cls.errors.append(args)
        cls.app = Application(cls.root)

    @classmethod
    def tearDownClass(cls):
        cls.root.destroy()

    def test_interactive_workflow(self):
        app = self.app; before = dataset_manifest()
        for name in ['ENV1', 'ENV2', 'ENV3']:
            app.env.set(name); app.load(); self.root.update()
            self.assertTrue(app.result.found); self.assertEqual(app.sim.state, 'ready')
        app.env.set('ENV1'); app.load()
        app.start(); app.sim.step(.1); position = app.sim.position
        app.mode.set('goal')
        app.click(SimpleNamespace(inaxes=app.ax, xdata=1.5, ydata=.3, button=1))
        self.assertEqual(app.result.waypoints[0], position)
        self.assertEqual(app.sim.state, 'running')
        app.sim.pause(); self.assertEqual(app.sim.state, 'paused')
        app.start(); self.assertEqual(app.sim.state, 'running')
        app.reset(); self.assertEqual(app.sim.position, position)
        app.mode.set('start')
        app.click(SimpleNamespace(inaxes=app.ax, xdata=.01, ydata=.01, button=1))
        self.assertEqual(app.result.status, 'invalid'); self.assertEqual(app.sim.state, 'idle')
        app.load(); app.connectivity.set('4'); app.recalculate()
        self.assertTrue(app.result.found)
        app.radius.set('.1'); app.margin.set('.02'); app.recalculate()
        self.assertEqual(app.active_config.planning_radius, .1+.02)
        app.show_search.set(True); app.show_gt.set(True); app.recalculate()
        self.assertTrue(app.result.search_cells)
        app.mode.set('edit'); app.click(SimpleNamespace(inaxes=app.ax, xdata=1.5, ydata=.3, button=1))
        self.assertTrue(app.edited); self.assertEqual(dataset_manifest(), before)
        app.radius.set('bad'); app.recalculate()
        self.assertIn('Invalid settings', app.notice.get()); self.assertNotEqual(app.sim.state, 'running')
        app.radius.set(''); app.margin.set('0'); app.connectivity.set('8'); app.load()
        with tempfile.TemporaryDirectory(prefix='gui export with spaces ') as folder:
            with patch('ugv_planner.gui.ROOT', Path(folder)):
                app.export(); self.assertEqual(len(list(Path(folder).rglob('ENV1_live.png'))), 1)
                app.batch(); self.assertEqual(len(list(Path(folder).rglob('summary.csv'))), 1)
        self.assertEqual(self.errors, [])

    def test_unsafe_start_disabled(self):
        app = self.app
        app.env.set('ENV3'); app.radius.set(''); app.margin.set('0')
        app.clearance_weight.set('0'); app.load()
        self.assertFalse(app.dispatch_allowed)
        self.assertIn('disabled', app.start_button.state())
        app.start(); self.assertNotEqual(app.sim.state, 'running')
        app.clearance_weight.set('4'); app.margin.set('.1'); app.load()
        self.assertTrue(app.dispatch_allowed)
        app.start(); self.assertEqual(app.sim.state, 'running')


if __name__ == '__main__':
    unittest.main()
