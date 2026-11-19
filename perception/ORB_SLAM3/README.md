# ORB-SLAM3: recorded sequences, keyframes and sparse maps

Run these commands from `perception/` unless noted. Start with the export-reader example in section 3.2 to inspect a real result: it needs only Python 3.10+ and the checked-in samples. Running the tracker also requires a compiled binary, vocabulary, display and complete sequence.

## Install large inputs for a tracking replay

Obtain the three files in [RELEASE_ASSETS.json](RELEASE_ASSETS.json) separately. Set `asset_dir` to their location:

```bash
asset_dir="$HOME/Downloads"
python3 ORB_SLAM3/restore_orb_vocabulary.py "$asset_dir/ORB_Vocabulary.zip"
python3 ORB_SLAM3/install_orb_frames.py --video1 "$asset_dir/ORB_video1_first20s_COMPLETE.zip" --video2 "$asset_dir/ORB_video2_first20s_COMPLETE.zip"
```

The helpers verify original hashes. The 76 checked-in sample images cover every saved keyframe. The other 724 frames come from these archives. The three downloads are not the combined `ORB_assets.zip` expected by the separate team installer.

### 3.1 Code, supporting files and setup

`ORB_SLAM3/source/ORB_SLAM3/` contains the original custom `System.h`, `System.cc`, `mono_tum.cc`, `build.sh`, `CMakeLists.txt`, complete `Tello_ENV1_provisional.yaml`, and (after restoration) `Vocabulary/ORBvoc.txt`. The remaining source and bundled DBoW2/g2o/Sophus dependencies were restored from the exact recorded upstream commit, `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`. **All 524 included source/support files match the owner's collection manifest byte-for-byte by size and SHA-256**, including the seven supplied originals. No generic replacement was made for the custom patches. Large unrelated example datasets and old backup files are omitted. CMake-referenced example source files are retained.

`source_reconstruction.json` records provenance and omissions. `evidence/COLLECTION_MANIFEST.json` preserves the owner's original file hashes, Git revision/status and environment probes. `evidence/upload_audit.json` records that all 21 files from the four received ZIPs match that manifest. These are integrity checks, not compilation or tracking tests.

| Setup item | Evidence / requirement |
|---|---|
| Execution placement | Original ASUS edge laptop, Ubuntu 22.04 under WSL2; not MentorPi. |
| C++ compiler | Collected GCC 11.4.0; C++11-capable compiler required. |
| CMake | Collected 3.22.1. |
| OpenCV C++ | Collected 4.5.4. Project CMake requests 4.4 or newer. This is separate from Python OpenCV used by ERFNet. |
| Eigen | System development headers required. Main CMake requests ≥3.1; the bundled Sophus build requests ≥3.3. Original installed version was not collected. |
| Pangolin | Version 0.6 reported in the technical context. `ldd` confirms `/usr/local/lib/libpangolin.so`; `pkg-config pangolin` failed to find metadata, which does not mean the library is absent. Exact installed version is not independently established by that probe. |
| Other build dependencies | Boost serialization, OpenSSL crypto, OpenGL/GLEW and normal compiler development packages. The original executable links Boost serialization 1.74 and libcrypto.so.3. |
| Bundled libraries | DBoW2, g2o and Sophus source included. System dependencies still need installation. RealSense is optional and not used by this Tello monocular runner. |
| Display | The existing runner enables the Pangolin viewer; provide a working OpenGL display/WSLg session. No headless command-line flag is implemented. |
| Binary | Original executable is not supplied. Build on the receiving Linux machine, or use the owner's existing compatible build. No fresh C++ build was performed here. |

On a new Ubuntu 22.04 machine, the system development-package starting point is:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git pkg-config libopencv-dev \
  libeigen3-dev libboost-serialization-dev libssl-dev libglew-dev \
  libgl1-mesa-dev libx11-dev
```

Install/configure the reported [Pangolin v0.6 release](https://github.com/stevenlovegrove/Pangolin/releases/tag/v0.6) using its own build instructions, or reuse the original working installation. It is an external dependency, not bundled in this handoff. Package-manager revisions on a new host may differ from the collected versions above; record them when validating the build. No fresh installation on a new host is claimed here.

On the original machine with its working dependencies, or a machine prepared with matching development packages, run from the perception directory:

```bash
bash ORB_SLAM3/build_monocular.sh
```

This new helper uses two build jobs by default, builds the bundled libraries and `mono_tum`, and stops on a build error. Set `ORB_BUILD_JOBS` to change concurrency. It uses the already-extracted vocabulary. The original `source/ORB_SLAM3/build.sh` is preserved unchanged but tries to extract `ORBvoc.txt.tar.gz`, which is not included because the restore helper installs the full text vocabulary. The new helper is syntax-checked, not build-tested here. For another machine, install the external C++ development libraries in the table and ensure CMake can find Pangolin before running it; a successful build on a new host remains a deployment check.

The actual provisional YAML contains:

| Setting | Value |
|---|---|
| Camera / image | `PinHole`, 960×720, 20 FPS, `Camera.RGB: 0` (OpenCV BGR input). |
| Intrinsics | fx 904.265879; fy 906.428764; cx 460.152136; cy 350.482616. |
| Distortion | k1 −0.012434; k2 0.033869; p1 −0.003006; p2 −0.002901; k3 0. |
| ORB extraction | 1,000 features; scale factor 1.2; 8 levels; FAST thresholds 20 / 7. |

Viewer settings are also present in the full YAML. These are exact configuration values, **not evidence of verified physical camera calibration**. The older `observed_settings_from_log.json` remains a rounded historical log extract; use the actual YAML for execution.

### 3.2 Inputs, availability and invocation

Input is a continuous, ordered monocular PNG sequence with one timestamp/filename row per image in `rgb.txt`, the vocabulary, and the camera YAML. Both original manifests are now supplied under `ORB_SLAM3/datasets/ENV1/`. They contain exactly three initial comment lines followed by 400 rows, from `0.000000 rgb/000001.png` to `19.950000 rgb/000400.png`. Timestamps are relative to the start of each sequence at 20 FPS. They are not wall-clock acquisition times or synchronized UGV timestamps.

All 400 frames per video are byte-identical to the original files listed in the collection manifest. The following six previously supplied samples are now part of each full sequence:

| Timestamp (s) | Zero-based input frame index | Filename |
|---|---|---|
| 2 | 40 | `rgb/000041.png` |
| 5 | 100 | `rgb/000101.png` |
| 8 | 160 | `rgb/000161.png` |
| 11 | 220 | `rgb/000221.png` |
| 14 | 280 | `rgb/000281.png` |
| 17 | 340 | `rgb/000341.png` |

**Availability after installing the two archives: 400/400 images for each video, 800/800 total; zero missing images.** The new `ORB_video1_first20s_COMPLETE.zip` and `ORB_video2_first20s_COMPLETE.zip` each contain 400 RGB PNGs plus `rgb.txt`. Their contents have been verified and successfully assembled with the main package during checking. They are delivered as separate companion ZIPs; run the installation command above before a full SLAM replay. The earlier incomplete frame uploads and unavailable `datasets(1).zip` are no longer needed. `evidence/complete_frames_verification.json` records archive hashes, per-file original-hash comparisons and successful PNG decoding at 960×720.

After installing both companion ZIPs, run the standard-library preflight to verify the complete input files:

```bash
python3 ORB_SLAM3/preflight.py \
  --dataset ORB_SLAM3/datasets/ENV1/video1_first20s \
  --dataset ORB_SLAM3/datasets/ENV1/video2_first20s \
  --collection-manifest ORB_SLAM3/evidence/COLLECTION_MANIFEST.json \
  --output orb_input_check.json
```

After installing both companion archives, it returns exit 0, `status: inputs_valid`, 800 available frames and zero missing frames. The recorded check in `ORB_SLAM3/evidence/input_availability.json` applies to the assembled three-ZIP handoff. The main ZIP alone has 76 frames, so its full-sequence preflight will report 724 images still needing installation. It validates manifest syntax, expected count/timing, available PNG headers/dimensions, and hashes against the original manifest. It also checks the supplied vocabulary/settings. The preflight itself does not fully decode images, verify calibration, compile or run SLAM. A separate archive audit has already fully decoded all 800 PNGs; its evidence is in `complete_frames_verification.json`. Add `--binary /absolute/path/to/mono_tum` to check that the compiled executable file exists. Exit 0 means **inputs passed these checks**, not successful tracking; an existing report is refused with exit 2. Ctrl+C returns 130. Default OS termination has no guaranteed report flush.

After installing the companion datasets and system dependencies and building the binary, the original entry point is exactly:

```text
mono_tum path_to_vocabulary path_to_settings path_to_sequence
```

For example, from a new empty results directory, with `orb_dir` set to the absolute path of this package's `ORB_SLAM3` directory:

```bash
"$orb_dir/source/ORB_SLAM3/Examples/Monocular/mono_tum" \
  "$orb_dir/source/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
  "$orb_dir/source/ORB_SLAM3/Examples/Monocular/Tello_ENV1_provisional.yaml" \
  "$orb_dir/datasets/ENV1/video1_first20s" > run.log 2>&1
orb_exit_code=$?
printf 'ORB exit code: %s\n' "$orb_exit_code"
```

Use a new working directory for each run because the C++ exporter writes fixed filenames there. The run command above is source-confirmed but **not executed during this update**. No ROS topic, socket protocol, live Tello receiver or scheduler adapter is implemented by this executable.

At C++ level, `System::TrackMonocular(im, timestamp)` consumes the image and sequence timestamp and returns a world-to-camera `T_cw` pose. Tracking state must be checked separately with `GetTrackingState()` before accepting a pose. The existing `mono_tum` loop discards the returned pose and does not stream per-frame states or poses. It calls `Shutdown()` after all images, then exports map points and saved keyframe poses.

The export reader runs immediately on the archived successful data:

```bash
python3 ORB_SLAM3/read_exports.py \
  --poses ORB_SLAM3/historical_results/video1_3D_export/KeyFrameTrajectory.txt \
  --points ORB_SLAM3/historical_results/video1_3D_export/MapPoints.xyz \
  --rgb-list ORB_SLAM3/datasets/ENV1/video1_first20s/rgb.txt \
  --collection-manifest ORB_SLAM3/evidence/COLLECTION_MANIFEST.json \
  --output runs/orb_replay/video1.json
```

Python entry point: `read_exports(poses_path, points_path, rgb_list=None, collection_manifest=None)` in `read_exports.py`; it imports the included `preflight.py`. Both use only the Python standard library. Import from within `ORB_SLAM3/`, or add that directory to Python's module search path. This operation parses saved outputs; it does not estimate poses from images.

### 3.3 Actual outputs, coordinates and selected keyframes

The Video 1 archived example contains **17 keyframe rows and 1,755 point rows**, from a log reporting 400 input images. The re-parsed JSON is `ORB_SLAM3/sample_output/video1_export_data.json`. The three Video 2 export sets are also parsed in that directory, with their provenance retained.

`KeyFrameTrajectory.txt` contains eight columns:

| Field | Meaning |
|---|---|
| `timestamp_s` | Saved keyframe's input timestamp, relative to its sequence. |
| `tx ty tz` | Camera position in the SLAM world frame, arbitrary monocular units. |
| `qx qy qz qw` | Camera-to-world rotation quaternion in XYZW order. |

An actual Video 1 row is:

```text
2.000000 -0.0157064 -0.0050245 -0.0028497 0.0009135 0.0006292 -0.0006976 0.9999992
```

The **supplied custom source** confirms `SaveKeyFrameTrajectoryTUM()` uses `GetPoseInverse()` and writes `T_wc`. A point in camera coordinates therefore transforms as `p_world = R_wc p_camera + t_wc`. This is distinct from `TrackMonocular()`'s `T_cw` return value. The exporter sorts surviving, non-bad keyframes by internal keyframe ID, but does not save those IDs. Camera optical coordinates use x right, y down, z forward; the exported world frame has no verified arena/gravity alignment. No measured metric scale or transform to ERFNet, arena or UGV coordinates is supplied. Do not label XYZ as metres or assume world Z is physical height.

`MapPoints.xyz` contains `x y z` only. The custom exporter excludes null, bad and non-finite points. Both exporters obtain data from **the current Atlas map**, as confirmed by the hash-matched `Atlas.cc`; they do not combine all Atlas maps after tracking resets. There are no landmark IDs, map IDs, semantic classes, confidence values, covariance or point-to-keyframe observation links in these text outputs. Pose count is keyframe count, not one pose for every input frame.

The reader adds `export_row_index` (zero-based row number, **not internal keyframe ID**), `input_frame_index` (zero-based position in `rgb.txt`), `image_relative_path`, and `image_available`. Available images receive their SHA-256 and original-manifest comparison. With `--rgb-list`, the timestamp-to-filename association is exact within the export's six-decimal timestamp precision. The caller must choose the corresponding video: both sequences reuse the same relative timestamps, so timestamps alone cannot identify which video produced an export. A missing image is marked unavailable; it does not prevent reading a valid archived pose. The legacy `estimated_frame_index_at_20fps` field is retained for compatibility; prefer the verified `input_frame_index`.

`keyframe_image_links.json` records all **83 exported keyframe rows**, and every row now points to an available original PNG verified by SHA-256. Those rows refer to 66 distinct image files because the three Video 2 runs can select overlapping frames. For example, Video 1 export row 4 at 2.0 s maps to `datasets/ENV1/video1_first20s/rgb/000041.png`. The older Video 2 export still belongs to a run that later crashed; adding its input images does not turn it into evidence of clean execution. Original-image access is complete for these four archived exports, but internal keyframe IDs, landmark-observation links and geometric-semantic fusion remain absent.

### 3.4 Failure, completion and cancellation

| Situation | Existing implementation behavior / adapter implication |
|---|---|
| Wrong argument count | `mono_tum` prints usage and returns 1. |
| Missing/malformed/empty `rgb.txt` | Original loader has no open/parse validation and skips three lines unconditionally. A failed open can hang its EOF loop; an empty list can later cause invalid indexing. Run preflight and use a process timeout. |
| Missing/unreadable image | `imread` returns empty; runner prints `Failed to load image at: ...` and returns 1 before its normal `Shutdown()` call. |
| Settings/vocabulary cannot open | `System` prints an error and calls `exit(-1)` (normally shell status 255 on POSIX). Other configuration/display exceptions are not converted to structured statuses. |
| Tracking cannot initialize or is lost | Tracking states are available internally; log messages may indicate failure or a new map. The runner keeps processing and can ultimately produce empty or partial current-map outputs. There is no per-frame status file in the existing runner. |
| Export cannot be written | Map-point exporter prints an error and returns `void`; keyframe exporter does not check stream-open success. Main may still return 0. Validate fresh, nonempty outputs in addition to checking process exit. |
| Normal completion | Runner finishes input loop, calls patched `Shutdown()`, joins mapper/loop-closer/viewer threads, writes `MapPoints.xyz`, prints timing, writes `KeyFrameTrajectory.txt`, returns 0. Log text saying “Saving...” alone is not a completion acknowledgement. |
| Cancellation | No signal handler or stop acknowledgement is implemented. A supervisor may terminate the process, confirm that it exited, and mark outputs incomplete. Graceful cancellation/map flushing is not verified. |

`GetTrackingState()` exposes this enum from the included `Tracking.h`:

| Value | State |
|---|---|
| −1 | `SYSTEM_NOT_READY` |
| 0 | `NO_IMAGES_YET` |
| 1 | `NOT_INITIALIZED` |
| 2 | `OK` |
| 3 | `RECENTLY_LOST` |
| 4 | `LOST` |
| 5 | `OK_KLT` (enum member; not demonstrated by these runs). |

An adapter needing state transitions must call/export this API; this executable does not publish those transitions. Do not use `System::isLost()` or `isFinished()` as generic monocular lost/completed flags in this revision: they depend on IMU initialization. `isShutDown()` is set at the start of shutdown, before thread joins complete; it does not prove that export files are finished.

The old full Video 2 log shows repeated tracking failures/new-map creation and an empty trajectory. The earlier `video2_3D_export` produced 2,017 points/24 keyframe rows but ended in a shutdown crash. Newly supplied `video2_shutdown_fixed` and `video2_stability_test` logs both reach normal end-of-run export, with no crash line. This supports the reported shutdown fix. A captured shell exit code is not present in those logs, and no new SLAM run was performed here.

The Python reader returns exit 0 for valid nonempty finite exports; missing/empty/malformed files, invalid quaternions, unmatched timestamps or failed supplied hash checks return 1 with a JSON error on stderr. An existing output is refused with 2. Input-validation errors create no output file. Ctrl+C terminates it; no atomic partial-write guarantee is made. `status: completed` means the **export parsing** completed, not that SLAM or a robot just ran.

### 3.5 Timing and stopping point

| Archived run | Frames in log | Saved keyframes / points | Median tracking ms | Mean tracking ms | End-of-run evidence |
|---|---:|---:|---:|---:|---|
| Video 1 3D export | 400 | 17 / 1,755 | 17.9878 | 19.8006 | Export/timing log. |
| Video 2 earlier 3D export | 400 | 24 / 2,017 | 18.4082 | 19.7632 | Later shutdown crash; not clean completion. |
| Video 2 shutdown fixed | 400 | 21 / 1,953 | 25.7718 | 28.6401 | Normal-ending supplied log; no captured exit status. |
| Video 2 stability test | 400 | 21 / 1,943 | 25.2232 | 27.4419 | Normal-ending supplied log; no captured exit status. |

These measurements cover the `TrackMonocular` call, excluding vocabulary load, image loading, pacing, viewer startup, final thread joins and export. The runner sleeps to respect 0.05 s input intervals, so the nominal playback alone is about 20 s for 400 frames, plus overhead. Total measured wall time and cancellation latency remain unavailable. The export reader's measured parse times are recorded separately in its sample JSON; milliseconds of file parsing are not SLAM performance.

