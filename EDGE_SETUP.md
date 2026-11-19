# Run on the edge computer

> **In the FYP_SRM folder** this guide's setup is already done: the Python environment exists (its path is in
> `FYP_SRM.ini`, used by `RUN_COMBINED.cmd` and `FYP_SRM.exe`) and the ORB assets are in `ORB_handoff/`.
> Go straight to step 4. The console in step 6 is the newer one (mission video, UAV section), and the checks
> were re-run on it with Python 3.14: see [MERGE_NOTES.md](MERGE_NOTES.md). On another computer, follow all steps.

Run commands from the project folder containing this guide, `DAG_scheduler`, `ugv_planner`, `ugv_nav`, and `third_party`.

## 1. Start with the component check

This runs the real A* and YOLOv5n code with saved ORB results. It works on Windows without a robot. Fresh ORB execution requires Linux/WSL. Physical robot operation is a later stage.

Use **Python 3.11 or newer; Python 3.12 was tested**. Some Ubuntu installations default to Python 3.10, which is insufficient for the supplied UGV code. The standalone scheduler's older minimum does not apply to this combined project.

## 2. Get the files

Clone this repository from GitHub or download and extract its ZIP. Keep the folder structure intact. UGV source, maps, model and example images are included.

For the combined release, download **ORB_assets.zip** from GitHub Releases. After creating the Python environment in step 3, install it with:

```text
python scripts/install_orb_release.py /path/to/ORB_assets.zip
```

Use the virtual environment's Python executable. This verifies the release archive against assets/ORB_ASSETS.json and installs the ORB_handoff folder next to DAG_scheduler. If you use this combined archive, skip the original-three-archives installation command below.

The large ORB assets are installed from these three original archives:

- FYP_Three_Modules_DAG_Handoff_2026-11-07 (1).zip
- ORB_video1_first20s_COMPLETE.zip
- ORB_video2_first20s_COMPLETE.zip

The first includes ORB C++ source, vocabulary, settings, saved outputs and documentation. The other two contain complete image sequences. Use the actual downloaded filenames if `(1)` is absent. The installer extracts only ORB, verifies the recordings and preserves different existing files.

## 3. Set up Python and install ORB inputs

### Windows PowerShell

Open a terminal in this folder:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-combined.txt
.\.venv\Scripts\python.exe DAG_scheduler\install_orb_handoff.py --package "$HOME\Downloads\FYP_Three_Modules_DAG_Handoff_2026-11-07 (1).zip" --video1 "$HOME\Downloads\ORB_video1_first20s_COMPLETE.zip" --video2 "$HOME\Downloads\ORB_video2_first20s_COMPLETE.zip"
```

If `py -3.12` is unavailable, install Python 3.12 or use an installed 3.11+ executable. Change Downloads paths if the archives are elsewhere.

### Linux / WSL

Check `python3 --version` is 3.11+, then:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-combined.txt
.venv/bin/python DAG_scheduler/install_orb_handoff.py --package "/path/to/FYP_Three_Modules_DAG_Handoff_2026-11-07 (1).zip" --video1 "/path/to/ORB_video1_first20s_COMPLETE.zip" --video2 "/path/to/ORB_video2_first20s_COMPLETE.zip"
```

Use Linux paths inside WSL, such as `/mnt/c/Users/<your-name>/Downloads/`. Create a separate Linux virtual environment; do not reuse a Windows `.venv`. Pinned main packages were verified on Windows/Python 3.12; target Linux installation still needs its own check.

## 4. Run the combined check

Windows: double-click **RUN_COMBINED.cmd**, or run:

```powershell
.\.venv\Scripts\python.exe DAG_scheduler\run_combined.py --check
.\.venv\Scripts\python.exe DAG_scheduler\run_combined.py
```

Linux:

```bash
.venv/bin/python DAG_scheduler/run_combined.py --check
bash run_combined.sh
```

Expected final status: **COMPONENT_INTEGRATION_SUCCEEDED**. The new folder under `combined_runs` contains:

- `combined_summary.json`: short results and limitations.
- `report.json`: all task outputs, statuses and timings.
- `events.jsonl`: execution order.
- `w...` folders: ORB poses/points, navigation inputs, route and detections.

Default inputs are Video 1, ENV1 occupancy and three saved detector examples. A* and YOLO execute now; ORB imports existing results. To check the second recording with the same reference map:

```text
python DAG_scheduler/run_combined.py --sequence video2 --environment ENV1
```

In these later examples, replace `python` with your virtual environment's Python. Use `--orb-root /path/to/ORB_SLAM3` for assets installed elsewhere. On the development computer the runner also recognizes the existing sibling ORB_handoff folder.

Selecting ENV2/ENV3 changes the reference map; it does not claim the Tello video came from that arena. Some example endpoints fail footprint checks. Select checked positions using `--start X Y --goal X Y` in the map's metre coordinates. The program will not alter goals or shrink the robot automatically.

For other saved detector images:

```text
python DAG_scheduler/run_combined.py --images /path/to/image1.jpg /path/to/image2.jpg
```

These remain offline inputs. Detection boxes are image pixels, not robot destinations. The model detects green/red blocks, not people.

## 5. Run ORB again from the images

Use a working Linux/WSL environment. The native C++ executable cannot run under Windows Python. Its viewer also needs a working OpenGL display/WSLg.

Follow the teammate's ORB dependency instructions inside the installed handoff. From this project's root:

```bash
bash ORB_handoff/ORB_SLAM3/build_monocular.sh
.venv/bin/python DAG_scheduler/run_combined.py --orb-source execute --sequence video1 --check
.venv/bin/python DAG_scheduler/run_combined.py --orb-source execute --sequence video1
```

`--check` checks inputs, executable presence and display variables; it does not establish successful tracking. The execution command requires zero exit and valid nonempty exports. A crash or timeout blocks the combined report. Use `--binary /path/to/mono_tum` for a compatible executable built elsewhere.

Fresh ORB tracking was **not verified** in this integration because WSL access was denied. Camera settings remain provisional. ORB coordinates need calibrated scale/alignment before navigation; see [COMBINED_INTEGRATION.md](COMBINED_INTEGRATION.md).

## 6. The included UGV console and robot

Windows:

```powershell
.\.venv\Scripts\python.exe nav_main.py gui
```

Linux:

```bash
.venv/bin/python nav_main.py gui
```

This is the colleague's original console. Follow [the user guide](docs/USER_GUIDE.md) to try its simulator. It is separate from the DAG component check.

For the physical MentorPi, follow [the robot setup guide](docs/ROBOT_SETUP.md): configure its bridge, share the authentication key only between your devices, confirm sensor topics/camera, then complete the staged connection and movement checks. Planning and YOLO run on the edge. The bridge runs in the MentorPi's ROS 2 environment; ROS 2 is not required on the edge for this TCP connection.

The DAG still needs current pose, request-bound navigation, confirmed arrival/stop and fresh post-arrival camera frames. The underlying console components exist, but their DAG task service and ORB-to-map conversion remain unfinished. No real robot movement was tested here.

## 7. Verify and share

```text
python scripts/verify_combined.py
python DAG_scheduler/run_mission.py --mode real --check
```

The first runs scheduler/adapter tests. The second intentionally lists missing full-mission connections. Do not replace missing real bindings with demo functions.

Share the repository, this guide, COMBINED_INTEGRATION.md and the three ORB archives. If the repository is private, colleagues need repository access. Keys, local machine settings, virtual environments and generated runs are excluded from Git.
