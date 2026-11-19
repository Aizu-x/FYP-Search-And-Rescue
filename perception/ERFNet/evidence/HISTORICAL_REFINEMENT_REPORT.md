# ERFNet semantic-map refinement: method and measured results

**Status on 22 October 2026:** substantial improvement to the marked rail gaps; ENV1 and ENV2 pass the existing offline route checks. ENV3 remains rejected. These results support a partial implementation result, not a completed three-environment autonomous demonstration.

## Method suitable for the report

ERFNet generated three-class semantic scores for free space, blocked rails and obstacles. The scores were projected into the existing top-down image frame, with the existing annotation-assisted arena region retained. An automatic refinement stage used colour-connected regions supported by ERFNet scores to recover fragmented yellow rails, pink rail panels and coloured obstacles. Small-gap closing and constrained dark-rail bridging/edge recovery reduced gaps in rail boundaries. The same settings were applied to all three scenes. No ground-truth interior class labels or manually drawn repair regions were supplied to the refinement function.

The semantic labels were then converted directly into 60 × 60 occupancy grids using conservative any-overlap pooling. Blocked and obstacle pixels became occupied cells; unknown regions remained non-traversable. A footprint-aware A* planner generated candidate routes, which were checked against the separately annotated reference grid using independent geometric collision tests. Invalid routes were rejected.

The ERFNet checkpoint is unchanged in the adopted pipeline. Measured improvements below therefore belong to **ERFNet with automatic semantic-map refinement**. The visual outputs should use that method name.

## Segmentation results

The physical-scene values below are evaluated in the same 800 × 800 rectified frame and on the same valid pixels for both methods. They are not directly interchangeable with the earlier 640-pixel original-camera-view scores.

| Evaluation set | Previous ERFNet mIoU | With automatic refinement | Interpretation |
| --- | ---: | ---: | --- |
| Physical scenes, 3 rectified maps | 80.79% | 85.55% | Previously inspected development scenes |
| Phone validation, 4 images | 85.05% | 88.54% | Used to select refinement parameters |
| Existing phone test, 5 images | 84.12% | 86.63% | Held out of this refinement parameter selection; previously evaluated and small/correlated set |
| Tello Video2 diagnostic, 6 frames | 67.25% | 64.42% | Regression; do not claim camera-independent improvement |

| Physical-scene metric | Previous ERFNet | Refined pipeline |
| --- | ---: | ---: |
| Blocked-class recall | 91.83% | 98.15% |
| Blocked-class IoU | 87.82% | 92.84% |
| Obstacle-class IoU | 57.53% | 65.66% |
| Occupied-pixel recall, classes 1 and 2 combined | 91.68% | 98.21% |
| Free pixels incorrectly labelled occupied | 1.14% | 1.44% |

False-free pixels on interior rails decreased from **21,554 to 2,907** (86.5% reduction). This interior diagnostic excludes a 64-pixel band around the image border so the outer arena wall does not dominate the count. The improvement introduces some additional false occupied pixels; it is not error-free.

## Occupancy grids and A*

| Environment | Occupied cells incorrectly marked free: before → after | Free cells incorrectly marked occupied: before → after | Reference-validated route after refinement |
| --- | ---: | ---: | --- |
| ENV1 | 31 → 2 | 13 → 30 | Passed both existing endpoint sets |
| ENV2 | 71 → 2 | 62 → 80 | Passed both existing endpoint sets |
| ENV3 | 16 → 13 | 66 → 89 | Rejected in both endpoint sets |

The existing diagnostic start and goal positions were retained. Before this repair only ENV1 passed reference validation; after refinement ENV1 and ENV2 pass. ENV3 still fails near a black-rail boundary. A trial that favoured more clearance also encountered a missed small blue obstacle. Invalid candidate routes are not shown as successful routes in the delivered figure.

The planner uses nominal 5 cm cells for an assumed 3 m arena, a provisional 212 × 171 mm rectangular body, a 20 mm body margin and 16 discrete yaw orientations. These dimensions and localization errors have not been physically verified. The route checks are offline map-geometry checks, not robot experiments.

## What was tried and not adopted

A four-epoch rail-focused fine-tuning trial used the original 14 phone training images and 6 Tello Video1 frames, with 28 mixed training steps per epoch. It increased the blocked-class loss weight to 2.5, retained obstacle weight 4, froze batch-normalization statistics, and added class-targeted crops and appearance augmentation. Epoch 3 was selected on phone validation (85.76% mIoU). On the three physical original-view diagnostics its raw mIoU was 78.70%, compared with 79.69% for the parent model, and false occupied cells increased. The final package uses the parent checkpoint with the validated refinement stage.

A uniform extra one-cell grid margin was tested, but rejected because it blocked the unchanged start or goal positions. A soft clearance-cost A* variant was also tested; it still failed ENV3 reference checks. Neither trial is enabled by default. Full metrics are retained under `evidence/`.

## Figure captions

**Top-down comparison:** Original photographs, manually annotated reference masks and semantic maps produced by ERFNet with automatic appearance-guided refinement for ENV1–ENV3. Green denotes free space, orange blocked rails and boundaries, and red obstacles. The arena ROI is annotation-assisted. Predictions retain residual errors.

**Before/after comparison:** Effect of automatic semantic-map refinement on the existing ERFNet predictions. The refinement recovers missing yellow-rail segments and improves rail continuity using the input photographs and model scores. The model checkpoint is unchanged.

**Grid and route figure:** Occupancy grids derived from the refined semantic masks through conservative pooling. A* routes for ENV1 and ENV2 pass independent reference-map collision checks. The ENV3 route is withheld because validation fails. All results use nominal geometry and diagnostic endpoints.

## Reproducibility and limits

The three final semantic masks were reproduced pixel-for-pixel by fresh inference from the included original photographs and selected checkpoint. The package includes the actual label arrays, exported grids, refinement parameters, evaluation JSON, training trial records and reproducible scripts. The maps were not painted or generated as illustrations.

The three arena photographs motivated the repair and have already been inspected repeatedly; they are development cases, not an independent new generalization test. The four-image validation set is small and has also been reused. Appearance thresholds depend on the arena materials and lighting, and the Tello diagnostic regression limits the claim to the evaluated phone-image conditions.

A complete all-three-environment success cannot yet be claimed. Further reviewed labels from the intended operating camera should emphasize rail edges over dark floor regions, rail corners and small blue obstacles. Separate evaluation views/layouts must remain outside training and parameter selection. Physical calibration and robot execution remain separate work.

## Implementation references

- OpenCV: [HSV colour-range segmentation](https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html). The implementation uses HSV ranges to identify candidate appearance regions, with additional model-score and component constraints.
- OpenCV: [Morphological transformations](https://docs.opencv.org/4.x/d9/d61/tutorial_py_morphological_ops.html). Closing and dilation are used with bounded kernels; large enclosed free regions are not filled.
- PyTorch: [CrossEntropyLoss](https://docs.pytorch.org/docs/stable/generated/torch.nn.CrossEntropyLoss.html). Class weights were used in the separate fine-tuning trial.
