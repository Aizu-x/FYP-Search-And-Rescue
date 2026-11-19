# ORB-SLAM3 connected to the DAG

Combined-project update: A* and the supplied YOLOv5n are now connected for component checks in run_combined.py. See ../EDGE_SETUP.md. The fresh-ORB and calibration limits below still apply.

Updated 17 November 2026. This step connects ORB-SLAM3 only. A*, robot movement, map fusion, ERFNet inference, and the later detector are separate integration steps.

## What is working now

The actual adapters are connected in team_adapters.py and real_bindings.json. A focused runner uses the same Kahn scheduler as the mission:

**prepare_recording -> run_slam -> publish_slam**

- Input preparation checks all 400 frames for the selected sequence, original file hashes, timestamps, image dimensions, vocabulary, and camera settings.
- The ORB adapter accepts valid nonempty pose/point exports and connects each keyframe timestamp to its actual input image.
- It writes structured poses, geometry, selected-frame references, provenance, and an ORB-stage summary into a new task directory.
- Failed input, empty/malformed exports, unmatched frames, bad quaternions, changed source files, failed processes, and unconfirmed process stopping do not release dependent tasks.
- The complete robot mission still rejects the missing real modules. An ORB-only success is not a completed search-and-rescue mission.

## Verified real-data results

| Dataset | Input frames | Saved keyframe poses | Saved map points | Archived run chosen |
|---|---:|---:|---:|---|
| Video 1 | 400 | 17 | 1,755 | video1_3D_export |
| Video 2 | 400 | 21 | 1,943 | video2_stability_test |

Both archived outputs were successfully processed through the three-task graph. The older Video 2 3D-export run had a reported shutdown crash, so the adapter deliberately uses the later stability-test results. The archives do not record an actual shell exit code; this is retained as unknown rather than invented.

**These are imports of genuine saved ORB results, not new pose estimation from the videos.** Every report explicitly distinguishes archived_export_import from fresh_native_execution and records fresh_slam_run. All data in this stage is marked simulated=false because it is real supplied data; that flag alone does not mean SLAM was rerun.

## Run the connected stage now

The ORB inputs have already been installed beside the scheduler in this workspace, under ORB_handoff/ORB_SLAM3. No original Downloads files were changed.

Double-click **RUN_ORB_ARCHIVES.cmd**, or run from the scheduler folder:

```text
python run_orb.py
python run_orb.py --source archive --sequence video1
python run_orb.py --source archive --sequence video2
```

The first command checks both videos. It writes new reports under runs/orb_video.../. Open orb_stage.json for the short result, report.json for task details, and w002_1/slam_handoff.json for the next module's data. The original full demo still runs with RUN_DEMO.cmd.

## If you move the scheduler to another folder/computer

The scheduler ZIP contains the integration code and readers, not another copy of the large videos, vocabulary, or C++ source. Use the three ZIPs already supplied by the teammate. Run the following as one command, replacing the paths:

```text
python install_orb_handoff.py --package "path/to/FYP_Three_Modules_DAG_Handoff_2026-11-07 (1).zip" --video1 "path/to/ORB_video1_first20s_COMPLETE.zip" --video2 "path/to/ORB_video2_first20s_COMPLETE.zip"
```

This installs only the ORB assets in a sibling ORB_handoff folder, verifies the supplied companion-archive hashes and every frame/manifest hash, and preserves different existing files by refusing to overwrite them. It does not build or run the C++ source. An interrupted installation can be rerun; always run input checks afterwards.

```text
python run_orb.py --check
```

Use --orb-root with the actual ORB_SLAM3 directory if its location differs. The full mission's corresponding location is parameters.orb.root in real_bindings.json.

## Fresh ORB execution on Linux/WSL

The native-execution adapter is implemented, but an actual C++ build and fresh SLAM run could not be verified in this session: the WSL listing command returned Wsl/E_ACCESSDENIED. That does not establish whether WSL is installed or configured on the user's normal desktop session.

Inside the working Linux/WSL environment, install/use the dependencies documented in the teammate's guide, then build the supplied source using its reviewed build_monocular.sh helper. The helper is located at ORB_handoff/ORB_SLAM3/build_monocular.sh. The existing runner needs a working OpenGL display/WSLg session because it enables its viewer.

Then, from the scheduler folder using Linux Python 3.10 or later:

```text
python3 run_orb.py --source execute --sequence video1 --check
python3 run_orb.py --source execute --sequence video1 --timeout 300
python3 run_orb.py --source execute --sequence video2 --timeout 300
```

--check confirms input checks, executable availability, and the presence of a display environment variable; it does not prove the binary can link, open a viewer, or track successfully. --binary can point to the teammate's compatible compiled mono_tum. Use Linux paths inside WSL. Running Windows Python with a Linux binary is intentionally refused.

Each fresh call starts the exact supplied interface, without a shell:

```text
mono_tum vocabulary_path settings_path sequence_path
```

It runs in a new output directory, captures its log and exit status, and accepts outputs only after a zero exit code and successful export validation. Partial/stale exports and files left by a failed attempt are never reused as new successful results.

## Process stopping

A separate supervisor owns the native child. It holds a pipe connection to the task worker. If the task times out, is cancelled, or the worker dies, that pipe closes and the supervisor terminates the native process, waits for exit, and records acknowledgement. On Linux, the native program has its own process group for cleanup.

The DAG stop callback reads that acknowledgement from the original task directory. If no acknowledgement arrives, it reports stopping as unconfirmed. Tests checked command completion, nonzero exit, timeout, and worker cancellation using controlled local subprocess fixtures on Windows. The actual ORB binary and the Linux process-group branch remain deployment checks on the target environment.

Cancellation is not advertised as a graceful ORB map flush: interrupted outputs remain incomplete and are rejected.

## Data ready for the next module

The run_slam result retains the scheduler's recording_id, attempt_id, keyframe_ids, poses_ref and geometry_ref fields, and adds:

- selected_frames: each chosen input frame's ID, relative sequence filename, absolute image reference, timestamp, hash, and export-row index.
- execution_kind and fresh_slam_run: whether this stage imported saved exports or executed the native program.
- keyframe_count and map_point_count.
- pose_convention: T_wc camera-to-world, quaternion order XYZW.
- coordinate_units: arbitrary_monocular_slam_units.
- metric_calibrated=false, arena_alignment_verified=false, calibration_verified=false, navigation_ready=false.
- tracking_states_available=false, because the supplied C++ runner does not export per-frame tracking states.

keyframe_ids in the scheduler identify matched INPUT images. The supplied files do not contain internal ORB keyframe IDs, so export row numbers are never presented as those IDs. MapPoints.xyz has XYZ points without landmark IDs or point-to-image observation links. Both exporters cover the current Atlas map, not all maps created after resets.

## What is still needed for ORB

**To verify fresh execution:** a working Linux/WSL build of the supplied source, its runtime dependencies and viewer environment, followed by a fresh run with captured exit status and exports. No additional images are needed for the two supplied videos.

**Before using ORB geometry for robot navigation:** verified calibration for the actual camera/recording configuration and a measured scale/coordinate transformation connecting the SLAM world, arena grid, and UGV. Current settings are explicitly provisional and the XYZ values are not metres. The mapping/fusion owner must define how image labels and sparse geometry become a traversable ground map. The adapter must not invent this transformation or treat world Z as physical height.

If the mapping method needs internal landmark observation associations or live tracking-state events, those require additional C++ exports; they are not required for the saved-output adapter completed here.

## Other contents of the supplied handoff

The bundle also includes ERFNet and YOLOv5n. The implemented detector in that bundle is not YOLOv8. This task does not substitute it for the user's planned YOLOv8. The supplied ERFNet class numbering is 0=free_space, 1=blocked, 2=obstacle, 255=ignore; it will need an explicit mapping to the current DAG's 0=free, 1=obstacle, 2=blocked, -1=unknown convention when ERFNet is connected. Both non-free classes remain forbidden for planning.
