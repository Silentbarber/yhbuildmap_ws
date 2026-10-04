# Source And Modification Record

Release date: 2026-10-03. Repository: https://github.com/Silentbarber/yhbuildmap_ws.

## Faster-LIO

- Upstream: https://github.com/gaoxiang12/faster-lio.
- Base commit: `8038fbac95283708fe2aa30658b7e116a7ace6d8`.
- Original source archive SHA256: `7417200b4f841bca34c19c245b24349c9fa4b775ea4f734f75e38b5413d43cd6`.
- Preserved actual upstream GPLv2 LICENSE and existing copyright notices. Upstream package metadata said BSD; the release package metadata has been corrected to match the LICENSE.
- Modified `include/laser_mapping.h`, `src/laser_mapping.cc`: observation trace, scan timing diagnostics and optional weak-observation/raw-end experiments. Both experiments are disabled by the accepted pipeline.
- Modified `src/pointcloud_preprocess.cc`: MID360 validation/filtering and diagnostics.
- Modified `CMakeLists.txt`: shared ROS1 Livox message package and preprocessing regression executable.
- Added handheld MID360 launch and raw/pitch-compensated recording profiles.
- Upstream documentation, hardware bundles, TBB archives and generated artifacts are omitted. `README.MD` under the algorithm remains the upstream reference; use the repository root README for this release.
- Existing notices in inherited FAST-LIO and other library headers retain their original meaning and scope.

## ROS Adapters

- `src/livox_ros_driver/msg`: CustomMsg/CustomPoint definitions from Livox-SDK/livox_ros_driver, retained without changes. MIT license from the upstream LICENSE endpoint is included as `LICENSE.txt`.
- `src/livox_ros_driver/CMakeLists.txt` and `package.xml`: message-only adaptation added on the release date. No Livox hardware SDK or driver binary is distributed.
- `src/livox_pointcloud2_to_custommsg`: recorded MID360 PointCloud2 bridge, BSD-3-Clause as declared in its package manifest. Preserves point timing and orders offsets before Faster-LIO preprocessing.

## Offline Backend And Tooling

- `scripts/refine_independent_pose_graph.py`: independent validated pose graph; uses Open3D and SciPy, not another estimator's trajectory. Observation-dependent weights are heuristic, not measured full covariance.
- Runner and capture scripts: isolated ROS1 replay, message counts, full output coverage, source hashes and frame capture.
- Diagnostics: return alignment, temporal comparisons and surface checks. Diagnostic registration does not reposition the published maps.
- Build/test and local Open3D viewing entry points added for the standalone release.
- New backend, replay and diagnostic tooling is distributed under the repository root GPLv2 license.

The accepted backend is offline. Building a production live localization or loop-correction system requires further work; this release does not claim that functionality.

## Dynamic Filtering Development Branch

- Added `filter_dynamic_map.py`: experimental measured-ray visibility votes on the fixed baseline map. Uses SciPy cKDTree and Open3D PCD parsing; no Removert, ERASOR or Dynablox source is incorporated.
- Added structural subset audits, a four-recording runner, synthetic geometry/CLI regressions and an optional `--dynamic-filter` pipeline stage. Default baseline estimation and mapping remain unchanged.
- Four candidate maps and the baseline/removed-point web views are delivered locally. The published code includes the existing Open3D PCD viewer; the local web catalog and building data are not distributed with this repository.
- The filter is offline and experimental. It identifies temporal inconsistency, not semantic motion ground truth. Current parameters and limits are recorded in `docs/DYNAMIC_FILTERING.md`.
- Faster-LIO world-point publication preserves curvature timing for new ROS1 captures. Optional point-time ray origins use trajectory interpolation and bounded time groups; the accepted baseline map remains unchanged.
- Added a grouped frame-end origin control and `compare_ray_origin_ablation.py` to isolate origin changes, reject mixed evidence modes and export changed baseline points for review. The first recording's control did not establish a structural quality improvement.

No raw bags, building maps, captured scan arrays, credentials, machine-specific output paths or other algorithm checkouts are included.
