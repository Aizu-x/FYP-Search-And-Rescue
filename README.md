# UGV A* Path Planning System

A runnable Python project for the UAV → edge ERFNet → occupancy grid → UGV A* → motion-controller architecture. The planner uses only the supplied occupancy grids. The independent GT grids evaluate the result and monitor simulated motion; they never repair the route or select endpoints.

## Run in Windows / VS Code

Install Python 3.10 or newer with Tcl/Tk support. Open **this folder** in VS Code, then use its terminal:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe main.py
```

No activation or PowerShell execution-policy change is necessary. Select `.venv` as the VS Code Python interpreter. The supplied `.vscode/launch.json` provides interactive and batch debugging configurations. If your interpreter is already configured, run `python main.py`.

Ubuntu 22.04 setup (portable code, **not executed on Ubuntu here**):

```bash
sudo apt install python3-venv python3-tk
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python main.py
```

Only NumPy and Matplotlib are third-party runtime dependencies. Tkinter is supplied with standard Windows Python and via `python3-tk` on Ubuntu. ROS2, SciPy, web servers, and robot hardware are not required. Paths are anchored to the project directory, so launching from another working directory and paths containing spaces work.

## Operate the simulator

1. Select ENV1, ENV2, or ENV3. This restores that environment's original occupancy grid and documented example endpoints.
2. Choose **Set Start** or **Set Goal**, then click the map. Coordinates are in meters. Clicks and numeric fields explicitly select the containing cell's center; the displayed coordinates show that quantization. Occupied/out-of-map/inflated endpoints are rejected, not relocated to a free cell.
3. A* immediately recalculates. Numeric coordinates take effect with **Apply coordinates / replan**. That button explicitly sets both endpoints; clicking a new goal instead uses the current simulated pose.
4. Press **Start / Resume**. The blue marker moves continuously along the route, with a blue progress trace. **Pause**, **Reset simulation**, and the speed slider control motion. Reset returns to the start of the currently installed route.
5. A new goal during motion pauses planning, connects from the exact current pose, and resumes if a route exists. There is no jump to a cell center: the connector is checked and traversed. An invalid connector or goal stops motion and explains why.
6. Enter a circular bounding **radius** and **safety margin** in meters, then press **Apply footprint / replan** or Enter. Leave radius blank when unknown. Only check **Measured bounding radius verified** when you have measured and verified that the circle encloses the robot for every relevant orientation. No MentorPi dimensions are assumed.
7. Switch 4/8 connectivity, toggle inflation, GT overlay, route visibility, or expanded-node visualization. Search visualization shows the completed expanded set, not a slowed-down animated search. With inflation disabled, footprint validation and motion monitors still use the configured dimensions.
8. **Toggle obstacle** flips clicked cells in a simulation-only copy. The title identifies edited maps. **Restore original environment** discards those in-memory edits and restores example endpoints. Source `.npy` files are never rewritten.
9. **Export current** creates a timestamped `exports/` directory with PNG, a live progress PNG, metrics CSV, waypoint CSV, executed-position CSV, JSON metadata, and the exact planning-grid snapshot. Executed-position samples are ordered positions, not uniformly timed telemetry.
10. **Run & export all three examples** runs the fixed scenarios on the original maps with the current planner settings, opens a comparison table, and saves all outputs. It pauses the current simulation. It does not reuse current interactive endpoints or map edits.

White areas are free, dark areas occupancy obstacles, amber areas planning inflation, green the A* route, red GT-unsafe segments, green circle the start, and purple star the goal. The optional translucent pink overlay is GT occupancy. The small blue marker has a minimum display radius of 1.8 cm for visibility; it is **not a physical dimension** when the radius is unknown or smaller. A real configured radius larger than this is drawn to scale.

## Data and coordinate assumptions

All six original files are included byte-for-byte in `data/`, preserving `ENV1_GT (1).npy`. Their inspected shapes are `100 × 100`, dtype `uint8`, values `{0,1}`. Resolution is the user-specified 0.03 m/cell, covering 3 × 3 m. `data_manifest.json` records SHA-256 checksums and counts.

`.npy` does not encode a coordinate reference system, origin, rotation, or registration. We interpret source row 0 as the image top, based on the reference's ENV1 layout, and flip rows **in memory only**. Internal grids use row 0 at the bottom, columns increasing right, origin `(0,0)` at the lower-left corner. Cell `(r,c)` is centered at `((c+0.5)*0.03, (r+0.5)*0.03)`. Internal `Grid` inputs and exported planning snapshots are already bottom-first. Do not flip ROS grids again.

Array dimensions, common coordinate convention and nominal scale were checked. Physical registration and metric accuracy cannot be proved from these arrays; confirm them with the mapping pipeline. The actual ENV2/ENV3 maps differ materially from the reference illustration. The software uses the arrays, not traced rails or routes from that illustration.

Example center coordinates, explicitly selected using occupancy rather than GT:

| Environment | Start x, y (m) | Goal x, y (m) |
|---|---|---|
| ENV1 | 0.345, 0.315 | 2.655, 2.595 |
| ENV2 | 0.345, 0.315 | 2.505, 2.505 |
| ENV3 | 0.345, 0.315 | 1.965, 1.965 |

The ENV1-style goal `(2.655,2.595)` is occupied in ENV2 and is **not** used as an ENV2 valid-route example. It remains an invalid input if entered interactively. Modify `EXAMPLES` in `config.py` only when defining a new repeatable experimental protocol; normal interactive work needs no source edits.

## Standalone planning API

```python
from ugv_planner import load_environment, plan, evaluate, PlannerConfig, Footprint

occupancy, gt = load_environment('ENV1')
config = PlannerConfig(connectivity=8, footprint=Footprint(radius_m=None), inflation=True)
start = occupancy.world_to_cell((0.345, 0.315))
goal = occupancy.world_to_cell((2.655, 2.595))
result = plan(occupancy, start, goal, config)  # GT is not an argument
if result.found:
    print(result.waypoints)  # ordered (x,y) meters
    print(result.cost_cells, result.distance_m, result.expanded_nodes, result.computation_ms)
report = evaluate(result, gt, config.footprint)  # independent post-plan evaluation
```

`plan()` returns `found`, `invalid`, or `unreachable`; it never silently modifies the map/endpoints. Start and goal arguments are integer `(row,column)` cells. `NavigationService` accepts world poses and goals, adds a checked exact-pose connector, and exposes map/pose/goal update methods. Goals remain disclosed cell-center targets, not exact arbitrary subcell poses.

### Algorithm and geometry

The implementation is from scratch: `heapq` priority queue, best-g array, closed array, stale-entry rejection, and parent reconstruction. Four-way edges cost 1 with Manhattan heuristic; eight-way diagonal edges cost √2 with octile heuristic. Both heuristics are admissible and consistent for this graph. Removing edges due to collisions preserves consistency. Closed nodes therefore need not reopen. With clearance weight 0, the returned route is shortest on the configured discrete graph, not necessarily the globally shortest continuous curve. With positive weight, A* minimizes distance plus the documented clearance penalty; it does not claim shortest geometric distance. Equal-cost routes use deterministic tuple tie-breaking.

Occupied cells are **closed square areas**, not point obstacles. Touching counts as collision (numerical tolerance `1e-10 m`). The map exterior/boundary is blocked. Inflation computes disk-to-cell-square overlap at each candidate cell center using exact offset distances, including the map boundary. Diagonal moves require both orthogonal neighboring centers to be free in that same inflated map. Every candidate improved edge also passes an exact continuous swept-disk segment check against original obstacles. This diagonal rule is intentionally conservative, especially with nonzero radius.

The geometry uses segment/AABB slab intersection plus endpoint-to-square and square-corner-to-segment distance. GT validation computes the minimum over **entire route segments**, including any pose connector, and map boundaries. Minimum GT clearance means centerline-to-obstacle-edge/boundary distance; footprint clearance subtracts physical radius; passing the margin requires centerline clearance strictly greater than radius + margin. Corners cannot be missed by waypoint-only sampling.

Planner time includes inflation, search, and planning collision checks; it excludes GT evaluation, drawing, and the navigation service's pose connector. Cost is in cell units and distance in meters. For a service connector, its Euclidean length is also included in reported distance/cost. Turns count changes in segment direction; detour is route length / endpoint straight-line distance (defined as 1 for start=goal).

### Safety statuses

- **Path found and GT-clearance passed (configured model):** verified supplied circular footprint and margin clear the independent reference map under the stated geometry.
- **Path found but GT collision/clearance failed:** continuous centerline, footprint, or margin violation. Red segments identify it. A known violation is reported even when dimensions are unknown.
- **No path available:** the configured occupancy graph has no route.
- **Invalid start or goal:** outside map, occupied, inside planning inflation, or unsafe exact-pose connector.
- **Safety undetermined:** no detected configured-geometry violation, but robot radius is missing or unverified.

A preflight gate disables Start and automatic resume for known GT or configured-occupancy footprint failures. This veto never modifies the route. Unknown/unverified dimensions still allow explicitly conditional simulation. The GT monitor is an **offline experiment instrument**, not an onboard sensor. It stops motion before the next violating subsegment; it does not replan using GT. An occupancy monitor also checks the configured footprint, including when planning inflation is disabled. With unknown radius these are point-robot tests. Collision stop is latched until reset or replanning. No simulation result establishes physical navigation safety.


## Clearance-preferred planning (updated)

The GUI now defaults to **clearance weight 4** and **preferred extra clearance 0.20 m**. These are route-preference settings, not robot dimensions. The hard minimum remains the footprint radius plus safety margin. In a narrow corridor the preference may not be achievable; hard collision checks still apply. Weight 0 restores shortest-distance A*. The API and batch default to weight 0 for baseline reproducibility.

The preferred center clearance target is `planning_radius + preferred_clearance_m`. At each cell center, exact occupancy/boundary clearance is `d`. Its soft penalty is `clip((target-d)/preferred_clearance_m, 0, 1)^2`. Each edge costs `length_cells * (1 + weight * mean(endpoint_penalties))`. Costs remain at least the geometric edge length, so the original Manhattan/octile heuristic stays admissible and consistent. Continuous hard edge collision checks are unchanged. Soft preference is a discrete endpoint cost, not a continuous clearance guarantee.

`cost_cells` and `distance_m` retain geometric length. `objective_cost_cells` reports the separate weighted objective. Exact-pose connectors add a fixed connector cost using the containing center's penalty. This does not change the choice of the subsequent graph route. The figure/metrics report physical distance rather than relabeling the weighted objective as meters.

Press **Apply footprint / replan** to apply preference values, or Enter in either field. Use the sidebar scrollbar to reach lower controls on shorter windows. A red route is still shown for diagnosis, but **Start / Resume is disabled** if preflight finds a GT or configured-footprint violation. GT vetoes execution only; no retries use GT to tune weights, relocate endpoints or select a different route.

```powershell
python main.py --batch --radius 0 --margin 0.10 --clearance-weight 4 --extra-clearance 0.20 --output example_outputs/clearance_preferred
```

The three original-map example routes in this explicitly point-robot experiment reached their goals, with minimum GT clearances 19.5, 16.5 and 13.5 cm respectively. These are verified test outcomes, not a guarantee for all endpoints, edited maps, unknown obstacles, or the physical UGV. Radius 0 is a point model. Supply measured body dimensions before interpreting clearance as robot clearance.

## Tests and reproducible outputs

```powershell
python -m unittest discover -s tests -v
python main.py --batch
python main.py --batch --radius 0.10 --margin 0.02 --output example_outputs/inflation_experiment
```

The 0.10 m radius and 0.02 m margin in the second command are **arbitrary sensitivity-test inputs**, not MentorPi specifications; the footprint remains unverified. `--connectivity 4`, `--no-inflation`, `--radius`, `--margin`, and `--verified` configure batch runs. The GUI uses its own controls, not these batch flags.

See `RESULTS.md`, `example_outputs/`, and `verification/` for executed results. Tests cover analytical optima; 50 random graph comparisons against independent Dijkstra; blocked/out-of-bounds endpoints; unreachable goals; diagonal corner cutting; exact segment crossings/tangencies; inflation consistency and passage closure; unknown/verified footprint statuses; continuous motion/pause/resume/reset; large time steps through corners; exact-pose replanning; pure ROS occupancy conversion; all six actual files; export round trips; and real Tk callback workflows. GUI tests skip explicitly if no display is available (for example a headless Linux host).

## Modules and future UGV integration

| Module | Responsibility |
|---|---|
| `config.py` | Typed footprint/planning parameters, repeatable scenarios |
| `grid.py` | Immutable binary map, coordinate conversions, original file loading |
| `inflation.py` | Circular configuration-space obstacle inflation |
| `collision.py`, `clearance.py` | Continuous segment/disk geometry and occupancy-only clearance preference |
| `astar.py` | GUI-independent A* implementation |
| `evaluation.py` | GT-only post-plan validation and metrics |
| `simulation.py` | Replaceable ideal holonomic route follower |
| `interfaces.py` | Grid/pose/goal/replan service, controller protocol, pure ROS data conversion |
| `gui.py` | Tk/Matplotlib controls and visualization |
| `exporting.py`, `batch.py` | Repeatable reports, figures, trajectories and metadata |

See `INTEGRATION.md` for the replacement boundary and hardware work still required. The simulation uses constant-speed holonomic translation along the polyline, with instantaneous direction changes, zero latency, and no slip, acceleration, heading dynamics, or localization error. This is a planning test harness, not a mecanum dynamics model.

The laptop GUI performs planning synchronously. Current 100×100 examples complete in roughly a few hundred milliseconds on the test machine, during which motion is paused; this is not a real-time scheduling guarantee. Scaling up needs profiling, cancellation/asynchronous planning, map versioning and spatial indexing. A rectangular footprint needs orientation-aware swept-polygon checking and potentially a `(row,column,yaw)` graph; replacing just the drawing is insufficient.
