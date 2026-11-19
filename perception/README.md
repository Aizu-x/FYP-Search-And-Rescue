# Optional project reference: ORB-SLAM3 and ERFNet

You can skip this entire folder when installing or running the team program. Nothing here runs automatically. The two module folders document the offline perception work on the edge laptop. Start with the saved examples; neither requires a robot to inspect.

| Module | What went in | What came out | Where to look |
|---|---|---|---|
| [ORB-SLAM3](ORB_SLAM3/README.md) | Two recorded monocular sequences from ENV1 | Sparse 3D landmarks and timestamped keyframe poses in arbitrary SLAM units | [Trajectories, points and logs](ORB_SLAM3/historical_results/); [keyframe–image links](ORB_SLAM3/keyframe_image_links.json) |
| [ERFNet](ERFNet/README.md) | Original photographs of ENV1, ENV2 and ENV3 | Per-pixel predictions, refined top-down masks and 60×60 occupancy grids | [Inputs](ERFNet/inputs/); [raw, refined and numerical outputs](ERFNet/sample_output/); [evaluation](ERFNet/evidence/HISTORICAL_REFINEMENT_REPORT.md) |

ORB does not supply the alignment used by ERFNet. ERFNet's saved-scene geometry and arena boundary are annotation-assisted. The grids exclude robot footprint and clearance; the nominal 5 cm cell width assumes a 3 m arena. This directory does not change the team scheduler, existing maps or planner.

**Large ORB inputs:** the vocabulary and two full recordings are separate archives identified in the [asset inventory](ORB_SLAM3/RELEASE_ASSETS.json). See the ORB guide to install them. Viewing saved results and running ERFNet do not require those archives.

**Attribution:** the ORB tree is based on [UZ-SLAMLab/ORB_SLAM3 at the recorded revision](https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4). The [original license](ORB_SLAM3/source/ORB_SLAM3/LICENSE) and bundled third-party notices are retained. The module guides distinguish upstream code, project changes and archived results. No new license is assigned here. The ERFNet model definition (`ERFNet/erfnet_tools/architecture.py`) is [ERFNet](https://github.com/Eromera/erfnet_pytorch) by Eduardo Romera (Romera et al., IEEE Transactions on Intelligent Transportation Systems, 2018), licensed CC BY-NC 4.0 (non-commercial use).
