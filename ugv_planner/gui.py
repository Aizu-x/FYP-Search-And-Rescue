"""Tk desktop UI. Planner and simulated motion have no GUI dependency."""
from pathlib import Path
from datetime import datetime
import time
import tkinter as tk
from tkinter import ttk, messagebox
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Circle
from .grid import load_environment
from .config import PlannerConfig, Footprint, EXAMPLES
from .interfaces import NavigationService
from .collision import CollisionChecker
from .evaluation import evaluate, simulation_preflight
from .simulation import SimulatedUGV
from .exporting import draw_map, export_run, export_trajectory

ROOT = Path(__file__).resolve().parents[1]


class Application:
    def __init__(self, root):
        self.root = root; root.title('UGV A* | Occupancy planning + independent GT validation')
        root.geometry('1280x900'); root.minsize(1050, 760)
        self.sim = SimulatedUGV(); self.reports = {}; self.edited = False
        self.mode = tk.StringVar(value='goal'); self.env = tk.StringVar(value='ENV1')
        self.connectivity = tk.StringVar(value='8'); self.radius = tk.StringVar(value='')
        self.margin = tk.StringVar(value='0'); self.speed = tk.DoubleVar(value=.25)
        self.clearance_weight = tk.StringVar(value='4.0')
        self.extra_clearance = tk.StringVar(value='0.20')
        self.inflation = tk.BooleanVar(value=True); self.verified = tk.BooleanVar(value=False)
        self.show_gt = tk.BooleanVar(value=False); self.show_search = tk.BooleanVar(value=False)
        self.show_path = tk.BooleanVar(value=True)
        self.sx = tk.StringVar(); self.sy = tk.StringVar(); self.gx = tk.StringVar(); self.gy = tk.StringVar()
        self.notice = tk.StringVar(value=''); self.live = tk.StringVar()
        sidebar = ttk.Frame(root); sidebar.pack(side='left', fill='y')
        scroll = tk.Canvas(sidebar, width=345, highlightthickness=0)
        scrollbar = ttk.Scrollbar(sidebar, orient='vertical', command=scroll.yview)
        scrollbar.pack(side='right', fill='y'); scroll.pack(side='left', fill='y')
        scroll.configure(yscrollcommand=scrollbar.set)
        left = ttk.Frame(scroll, padding=12)
        window = scroll.create_window((0, 0), window=left, anchor='nw')
        left.bind('<Configure>', lambda e: scroll.configure(scrollregion=scroll.bbox('all')))
        scroll.bind('<Configure>', lambda e: scroll.itemconfigure(window, width=e.width))
        right = ttk.Frame(root, padding=8); right.pack(side='right', fill='both', expand=True)
        ttk.Label(left, text='UGV PATH LAB', font=('Segoe UI', 17, 'bold')).pack(anchor='w')
        ttk.Label(left, text='A* global planner · offline validation').pack(anchor='w', pady=(0, 12))
        row = ttk.Frame(left); row.pack(fill='x')
        ttk.Label(row, text='Environment').pack(side='left')
        combo = ttk.Combobox(row, textvariable=self.env, values=list(EXAMPLES), state='readonly', width=10)
        combo.pack(side='right'); combo.bind('<<ComboboxSelected>>', lambda e: self.load())
        ttk.Separator(left).pack(fill='x', pady=10)
        for text, value in [('Set Start', 'start'), ('Set Goal', 'goal'), ('Toggle obstacle (simulation only)', 'edit')]:
            ttk.Radiobutton(left, text=text, variable=self.mode, value=value).pack(anchor='w')
        for label, x, y in [('Start x/y (m)', self.sx, self.sy), ('Goal x/y (m)', self.gx, self.gy)]:
            r = ttk.Frame(left); r.pack(fill='x', pady=3)
            ttk.Label(r, text=label, width=16).pack(side='left')
            ttk.Entry(r, textvariable=x, width=8).pack(side='left'); ttk.Entry(r, textvariable=y, width=8).pack(side='left')
        ttk.Button(left, text='Apply coordinates / replan', command=self.apply_coordinates).pack(fill='x', pady=4)
        ttk.Label(left, text='Clicks target cell centers (3 cm grid).\nNew goals use the current robot pose.', foreground='#586575').pack(anchor='w')
        ttk.Separator(left).pack(fill='x', pady=10)
        r = ttk.Frame(left); r.pack(fill='x')
        ttk.Label(r, text='Movement').pack(side='left')
        combo = ttk.Combobox(r, textvariable=self.connectivity, values=['4', '8'], state='readonly', width=7)
        combo.pack(side='right'); combo.bind('<<ComboboxSelected>>', lambda e: self.recalculate())
        for text, variable in [('Radius (m), blank = unknown', self.radius), ('Safety margin (m)', self.margin)]:
            ttk.Label(left, text=text).pack(anchor='w', pady=(5, 0))
            entry = ttk.Entry(left, textvariable=variable); entry.pack(fill='x')
            entry.bind('<Return>', lambda e: self.recalculate())
        ttk.Checkbutton(left, text='Measured bounding radius verified', variable=self.verified, command=self.recalculate).pack(anchor='w')
        ttk.Checkbutton(left, text='Use inflation for planning', variable=self.inflation, command=self.recalculate).pack(anchor='w')
        for label, var in [('Prefer extra clearance (m)', self.extra_clearance), ('Clearance preference weight (0 = shortest)', self.clearance_weight)]:
            row = ttk.Frame(left); row.pack(fill='x', pady=2)
            ttk.Label(row, text=label).pack(side='left')
            entry = ttk.Entry(row, textvariable=var, width=7); entry.pack(side='right')
            entry.bind('<Return>', lambda e: self.recalculate())
        ttk.Button(left, text='Apply footprint / replan', command=self.recalculate).pack(fill='x', pady=4)
        for text, var, command in [('Show GT overlay', self.show_gt, self.redraw), ('Show expanded nodes', self.show_search, self.recalculate), ('Show planned route', self.show_path, self.redraw)]:
            ttk.Checkbutton(left, text=text, variable=var, command=command).pack(anchor='w')
        ttk.Separator(left).pack(fill='x', pady=10)
        r = ttk.Frame(left); r.pack(fill='x')
        self.start_button = ttk.Button(r, text='Start / Resume', command=self.start)
        self.start_button.pack(side='left', expand=True, fill='x')
        ttk.Button(r, text='Pause', command=self.sim.pause).pack(side='left', expand=True, fill='x')
        ttk.Button(left, text='Reset simulation', command=self.reset).pack(fill='x', pady=4)
        ttk.Label(left, text='Simulation speed (0.05–1.00 m/s)').pack(anchor='w')
        ttk.Scale(left, from_=.05, to=1., variable=self.speed).pack(fill='x')
        ttk.Label(left, textvariable=self.live, wraplength=300).pack(anchor='w', pady=5)
        ttk.Button(left, text='Restore original environment', command=self.load).pack(fill='x', pady=3)
        ttk.Button(left, text='Export current PNG / CSV / JSON', command=self.export).pack(fill='x', pady=3)
        ttk.Button(left, text='Run & export all three examples', command=self.batch).pack(fill='x', pady=3)
        ttk.Label(left, textvariable=self.notice, wraplength=305, foreground='#914624').pack(anchor='w', pady=8)
        self.fig = Figure(figsize=(7, 6), constrained_layout=True); self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right); self.canvas.get_tk_widget().pack(fill='both', expand=True)
        self.canvas.mpl_connect('button_press_event', self.click)
        self.metrics = tk.Text(right, height=8, font=('Consolas', 10), wrap='word', background='#eef3f7', relief='flat')
        self.metrics.pack(fill='x', pady=6)
        self.load(); self.last_tick = time.monotonic(); self.root.after(40, self.tick)

    def config(self):
        return PlannerConfig(int(self.connectivity.get()), Footprint(
            None if not self.radius.get().strip() else float(self.radius.get()),
            float(self.margin.get()), self.verified.get()), self.inflation.get(), self.show_search.get(),
            float(self.clearance_weight.get()), float(self.extra_clearance.get()))

    def load(self):
        try:
            self.grid, self.gt = load_environment(self.env.get()); self.edited = False
            self.gt_checker = CollisionChecker(self.gt)
            self.start_point, self.goal_point = [self.grid.cell_to_world(self.grid.world_to_cell(p)) for p in EXAMPLES[self.env.get()]]
            self.sim.set_route([]); self.recalculate(use_current=False)
        except Exception as exc:
            messagebox.showerror('Map load failed', str(exc))

    def apply_coordinates(self):
        try:
            start = (float(self.sx.get()), float(self.sy.get()))
            goal = (float(self.gx.get()), float(self.gy.get()))
            # Explicit coordinate input changes both endpoints; no GT repair.
            self.start_point = self.grid.cell_to_world(self.grid.world_to_cell(start))
            self.goal_point = self.grid.cell_to_world(self.grid.world_to_cell(goal))
            self.recalculate(use_current=False)
        except ValueError as exc:
            self.sim.pause(); self.notice.set(str(exc))

    def click(self, event):
        if event.inaxes != self.ax or event.xdata is None or event.button != 1:
            return
        cell = self.grid.world_to_cell((event.xdata, event.ydata))
        if not self.grid.contains(cell):
            return
        if self.mode.get() == 'edit':
            self.grid = self.grid.edited(cell, not self.grid.cells[cell]); self.edited = True
            self.recalculate()
        elif self.mode.get() == 'start':
            self.start_point = self.grid.cell_to_world(cell); self.recalculate(use_current=False)
        else:
            self.goal_point = self.grid.cell_to_world(cell); self.recalculate()

    def recalculate(self, use_current=True):
        running = self.sim.state == 'running'; self.sim.pause()
        try:
            config = self.config()
            if use_current and self.sim.position is not None:
                self.start_point = self.sim.position
            service = NavigationService(self.grid, self.start_point, self.goal_point, config)
            result = service.replan()
            self.active_config = config; self.result = result
            self.occupancy_checker = CollisionChecker(self.grid)
            self.report = evaluate(result, self.gt, config.footprint)
            self.dispatch_allowed, self.dispatch_message = simulation_preflight(result, self.grid, self.report, config.footprint)
            self.report['dispatch_allowed'] = self.dispatch_allowed
            self.report['dispatch_message'] = self.dispatch_message
            self.start_button.configure(state='normal' if self.dispatch_allowed else 'disabled')
            self.report['simulation_only_edits'] = self.edited
            self.reports[self.env.get()] = self.report
            self.sim.set_route(result.waypoints)
            self.sx.set(f'{self.start_point[0]:.5f}'); self.sy.set(f'{self.start_point[1]:.5f}')
            self.gx.set(f'{self.goal_point[0]:.5f}'); self.gy.set(f'{self.goal_point[1]:.5f}')
            self.notice.set(result.message + (' Simulation-only map edits active.' if self.edited else ''))
            if not self.dispatch_allowed:
                self.notice.set(self.dispatch_message)
            if running:
                self.start()
            self.redraw()
        except (ValueError, TypeError) as exc:
            self.dispatch_allowed = False
            self.start_button.configure(state='disabled')
            self.notice.set(f'Invalid settings: {exc}. Motion paused; last result retained.')

    def redraw(self):
        if not hasattr(self, 'result'):
            return
        draw_map(self.ax, self.grid, self.gt, self.result, self.report, self.active_config,
                 self.show_gt.get(), self.show_search.get(), self.show_path.get())
        self.ax.set_title(self.env.get()+(' · SIMULATION EDITS' if self.edited else ' · ORIGINAL OCCUPANCY'))
        if not self.result.found:
            self.ax.scatter(*self.start_point, color='#087d67', marker='o', s=65)
            self.ax.scatter(*self.goal_point, color='#972f67', marker='*', s=110)
        self.robot = Circle(self.sim.position or self.start_point,
                            max(self.active_config.footprint.radius_m or 0, .018),
                            facecolor='#3e75d9', edgecolor='white', alpha=.85, zorder=10)
        self.ax.add_patch(self.robot)
        self.progress, = self.ax.plot([], [], color='#2878eb', lw=2.5, label='Progress')
        r = self.report
        def fmt(key, digits=3):
            return 'N/A' if r[key] is None else f'{r[key]:.{digits}f}'
        text = (f'{r["validation"]}\n{r["explanation"]}\n'
                f'Preference weight: {self.active_config.clearance_weight:g} | Extra clearance target: {self.active_config.preferred_clearance_m*100:g} cm (soft)\n'
                f'Length: {fmt("path_length_m")} m   Time: {r["planning_ms"]:.1f} ms   Expanded: {r["expanded_nodes"]}\n'
                f'Turns: {r["turns"]}   Detour: {fmt("detour_ratio")}   Min GT center clearance: {fmt("minimum_gt_clearance_cm", 2)} cm\n'
                f'GT center collision: {r["gt_center_collision"]}   Footprint collision: {r["gt_footprint_collision"]}\n'
                'Blue disk/line: simulated robot/progress. Occupancy/GT monitors stop before a violation.\n'
                'GT is evaluation only. Physical safety requires verified geometry, map alignment and localization.')
        self.metrics.configure(state='normal'); self.metrics.delete('1.0', 'end'); self.metrics.insert('1.0', text); self.metrics.configure(state='disabled')
        self.canvas.draw_idle()

    def start(self):
        if self.dispatch_allowed:
            self.sim.start()
        else:
            self.notice.set(getattr(self, 'dispatch_message', 'Apply valid settings before starting.'))

    def reset(self):
        self.sim.reset(); self.redraw()

    def tick(self):
        now = time.monotonic(); dt = min(now-self.last_tick, .1); self.last_tick = now
        self.sim.speed = float(self.speed.get())
        previous = self.sim.state
        self.sim.step(dt, lambda a, b: self.gt_checker.is_clear(a, b, self.active_config.footprint.effective_radius)
                      and self.occupancy_checker.is_clear(a, b, self.active_config.footprint.effective_radius))
        if self.sim.state == 'collision' and previous != 'collision':
            self.notice.set('Occupancy/GT monitor: motion stopped before collision/clearance violation. Route was not repaired using GT.')
        p = self.sim.position
        self.live.set(f'{self.sim.state.upper()} | {self.sim.speed:.2f} m/s\nPosition: '+(f'({p[0]:.3f}, {p[1]:.3f}) m' if p else 'no route'))
        if p and hasattr(self, 'robot'):
            self.robot.center = p
            self.progress.set_data([v[0] for v in self.sim.history], [v[1] for v in self.sim.history])
            if previous == 'running':
                self.canvas.draw_idle()
        self.root.after(40, self.tick)

    def export(self):
        folder = ROOT/'exports'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.report['simulation_state'] = self.sim.state
        self.report['simulation_final_position_m'] = self.sim.position
        export_run(folder, self.env.get(), self.grid, self.gt, self.result, self.report, self.active_config)
        export_trajectory(folder, self.env.get(), self.sim.history)
        self.fig.savefig(folder/f'{self.env.get()}_live.png', dpi=160)
        self.notice.set(f'Exported to {folder}')

    def batch(self):
        from .batch import run_batch
        self.sim.pause(); self.notice.set('Running original-map examples…'); self.root.update_idletasks()
        try:
            folder = ROOT/'exports'/datetime.now().strftime('comparison_%Y%m%d_%H%M%S_%f')
            reports = run_batch(folder, self.config())
            window = tk.Toplevel(self.root); window.title('Three-environment results (original maps)')
            text = tk.Text(window, width=115, height=24, font=('Consolas', 10)); text.pack(fill='both', expand=True)
            text.insert('end', f'{"Metric":34}'+''.join(f'{name:>22}' for name in reports)+'\n')
            for key in ['path_found', 'path_length_m', 'planning_ms', 'expanded_nodes', 'turns', 'detour_ratio', 'minimum_gt_clearance_cm', 'gt_center_collision', 'gt_footprint_collision', 'simulation_state']:
                values = [f'{r[key]:.4f}' if type(r[key]) is float else str(r[key]) for r in reports.values()]
                text.insert('end', f'{key:34}'+''.join(f'{v:>22}' for v in values)+'\n')
            for name, report in reports.items():
                text.insert('end', f'\n{name}: {report["validation"]}')
            text.configure(state='disabled'); self.notice.set(f'Comparison exported to {folder}')
        except ValueError as exc:
            self.notice.set(str(exc))


def launch():
    root = tk.Tk(); Application(root); root.mainloop()
