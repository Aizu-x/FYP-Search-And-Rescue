# ORB integration verification

Verified on 17 November 2026 using Windows and the available bundled Python runtime. This report covers the DAG connection, not perception accuracy or physical navigation.

## Automated checks

**49 tests passed in 33.346 seconds:** the existing 32 scheduler/mission tests and 17 ORB integration tests. The complete output is in ORB_TEST_RESULTS.txt.

Checks include Kahn ordering, dependency blocking, input timestamps and file integrity, empty/malformed exports, non-finite coordinates, invalid quaternions, unmatched keyframe images, literal process arguments, nonzero exits, timeouts, worker cancellation, and refusal to accept unconfirmed stopping. The original full simulated mission and its failure/replanning checks still pass.

Testing exposed a Windows file-sharing conflict when a reader briefly prevented an atomic status update. The supervisor now retries that specific type of access failure for a bounded interval, retains a diagnostic log, and still rejects persistent failures. After the fix, the worker-cancellation/process-cleanup test passed 12 consecutive repetitions, followed by the complete 49-test suite. A regression test checks both transient and persistent status-publication failures.

Native-process tests use controlled Python subprocesses on Windows. They do not run the C++ ORB executable or verify Linux process-group cleanup.

## Supplied-data integration

The installer verified both companion ZIP hashes and all 802 original frame/list files, comprising 800 images and two timestamp lists. Input preparation also verifies the vocabulary, settings, image dimensions, timestamps, and frame hashes. No files in Downloads were changed.

Both saved result sets completed **prepare_recording -> run_slam -> publish_slam** through the existing Kahn scheduler in real-data mode:

| Sequence | Input images | Keyframe poses | Map points | Result |
|---|---:|---:|---:|---|
| Video 1, video1_3D_export | 400 | 17 | 1,755 | SUCCEEDED |
| Video 2, video2_stability_test | 400 | 21 | 1,943 | SUCCEEDED |

The most recent full reports are in the sibling ORB_integration_results folder:

- orb_video1_a2603ef5aec4/report.json and orb_stage.json.
- orb_video2_9de9480668cb/report.json and orb_stage.json.

Each task writes into its own attempt directory. The run_slam task's slam_handoff.json, poses.json, and geometry.json contain the structured data for later adapters. Accepted keyframe rows link to matching verified original images. The release ZIP contains the code and this verification record; the large source assets and full run folders remain separate.

## What this does not verify

- These two runs imported the teammate's saved exports; they did not recompute poses from the images. Reports explicitly record archived_export_import and fresh_slam_run=false.
- The native-execution adapter is implemented, but a fresh C++ build and replay were not verified. WSL listing returned Wsl/E_ACCESSDENIED during this session.
- The archived logs do not contain captured shell exit codes. The adapter preserves that uncertainty. It excludes the older Video 2 export run whose log records a shutdown crash.
- Camera settings remain provisional. Geometry remains in arbitrary monocular units; calibration, metric scale, arena/UGV alignment, and navigation readiness are not established.
- No Tello flight, Wi-Fi capture, ERFNet fusion, A* navigation, YOLOv8 inference, or MentorPi movement was tested or connected in this step.

The current package is ready for saved-output integration and for fresh-execution validation in the team's working Linux environment. See ORB_INTEGRATION.md for the exact next requirements.
