# Standalone Release Validation

Date: 2026-10-03. Target: ROS1 Noetic / Ubuntu 20.04 / Python 3.8.10.

## Package Verification

- Fresh catkin build in the standalone release directory completed with GCC 11.4, C++17 and system TBB. Only `/opt/ros/noetic` was overlaid; no original algorithm workspace was sourced.
- Six Python regression tests passed: degenerate loop rejection, nondegenerate loop acceptance, plane information null directions, weak-observation weighting, timestamp jitter and serialized CustomMsg Header reading.
- MID360 C++ preprocessing regression passed 100 trials with zero near-range or nonfinite point leaks; valid tag/line/time, empty and single-point behavior passed.
- Pipeline, backend and viewer CLI help entry points passed.
- Accepted pipeline, geometric backend, capture script, both calibration profiles, launch and PointCloud2 bridge match the previously validated working version byte-for-byte. Faster-LIO estimation code only gained modification-date notices; release packaging changes are documented separately.
- The two committed workspace symlinks use relative targets. Generated catkin top-level symlink, build/devel, bag and captured maps are excluded from Git.
- New release tooling passed the staged whitespace check. Inherited source, message definitions and upstream license formatting were retained.

## Short ROS Replay

A 10-second segment from the beginning of the original 21:52:47 recording was generated locally to check the standalone runtime paths. It is not distributed and does not replace the previous four complete recordings' quality assessment.

- Raw input: 100 lidar messages and 2001 IMU messages.
- Bridge output: 100 converted lidar messages.
- Estimator output: 97 point-cloud frames and 97 odometry records after initialization.
- Runner completed with all expected input messages and final output coverage.
- Estimator output topics remained available through the end of playback.
- State trace included `normalized_yaw_information` and `matched_range_rms_m`, required by the observation-weighted backend.
- Frontend evaluation passed with no failures, finite trajectory and no nonpositive trajectory time differences.

The short segment does not contain a representative building traversal or validated loop closure. Offline geometric behavior was checked by the focused regressions and the already completed four-recording runs documented in the root README. Absolute accuracy was not measured against external ground truth.
