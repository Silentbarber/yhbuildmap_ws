#!/usr/bin/env bash
set -euo pipefail
release_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/noetic/setup.bash
source "$release_root/algorithms/gaoxiang12__faster-lio/devel/setup.bash"
set -u
cd "$release_root"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
/usr/bin/python3 -m unittest discover -s scripts/tests -p 'test_*.py' -v
"$release_root/algorithms/gaoxiang12__faster-lio/devel/lib/faster_lio/faster_mid360_preprocess_test"
/usr/bin/python3 scripts/run_faster_robust_mapping.py --help >/dev/null
/usr/bin/python3 scripts/refine_independent_pose_graph.py --help >/dev/null
/usr/bin/python3 scripts/view_pcd.py --help >/dev/null
/usr/bin/python3 scripts/filter_dynamic_map.py --help >/dev/null
/usr/bin/python3 scripts/run_dynamic_filtering.py --help >/dev/null
/usr/bin/python3 scripts/audit_dynamic_filter.py --help >/dev/null
/usr/bin/python3 scripts/audit_angular_visibility.py --help >/dev/null
/usr/bin/python3 scripts/audit_dynamic_regions.py --help >/dev/null
/usr/bin/python3 scripts/sweep_dynamic_filter_params.py --help >/dev/null
/usr/bin/python3 scripts/verify_temporal_region_sources.py --help >/dev/null
/usr/bin/python3 scripts/audit_dynamic_motion.py --help >/dev/null
/usr/bin/python3 scripts/filter_sensor_motion.py --help >/dev/null
/usr/bin/python3 scripts/audit_ray_footprint.py --help >/dev/null
/usr/bin/python3 scripts/audit_livox_point_timing.py --help >/dev/null
/usr/bin/python3 - <<'PY'
import sys
sys.path.insert(0, 'scripts')
import deskew_ray_origin
PY
