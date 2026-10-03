#!/usr/bin/env bash
set -euo pipefail
release_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  printf '%s\n' 'ROS1 Noetic is required at /opt/ros/noetic.' >&2
  exit 1
fi
# ROS setup files reference variables that may be unset.
set +u
source /opt/ros/noetic/setup.bash
set -u
cd "$release_root/algorithms/gaoxiang12__faster-lio"
catkin_make -j"${BUILD_JOBS:-2}" -l"${BUILD_JOBS:-2}" \
  -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DCATKIN_ENABLE_TESTING=ON -DHANDHELD_BUILD_TESTS=ON
