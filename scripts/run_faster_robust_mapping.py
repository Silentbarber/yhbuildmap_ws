#!/usr/bin/env python3
"""ROS1 Faster-LIO replay followed by validated, independent loop optimization."""
import argparse
import json
import os
import pathlib
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'algorithms/gaoxiang12__faster-lio/src/algorithm/config'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=pathlib.Path, required=True)
    parser.add_argument('--calibration', choices=['raw', 'pitch25'], required=True)
    parser.add_argument('--rate', type=float, default=.75)
    parser.add_argument('--output-dir', type=pathlib.Path)
    parser.add_argument('--backend-profile', choices=['observation_geometry', 'legacy'], default='observation_geometry',
                        help='Observation geometry checks local motion and wider revisits; legacy is the previous closure-only comparison')
    parser.add_argument('--experimental-hard-guard', action='store_true',
                        help='Experimental: can lose tracking after long weak returns; not a product default')
    parser.add_argument('--experimental-raw-scan-end', action='store_true',
                        help='Experimental: worsened temporal consistency in the two weak-return bags')
    parser.add_argument('--dynamic-filter', action='store_true',
                        help='Experimental offline visibility cleaning after optimized map reconstruction')
    angular = parser.add_mutually_exclusive_group()
    angular.add_argument('--dynamic-angular-support', action='store_true', default=None,
                         help='Measured-ray triangle coverage; default with --dynamic-filter')
    angular.add_argument('--dynamic-allow-angular-extrapolation', action='store_true',
                         help='Experimental legacy comparison: permit direction extrapolation')
    parser.add_argument('--dynamic-max-hit-bins', type=int, choices=[0, 1, 2, 3],
                        help='Dynamic candidate support threshold, default 3; 1 for conservative comparison')
    parser.add_argument('--dynamic-point-time-groups-ms', type=float, default=0.,
                        help='Experimental: use captured per-point times and interpolated ray origins in short groups')
    args = parser.parse_args()
    if (args.dynamic_angular_support is not None or args.dynamic_allow_angular_extrapolation
            or args.dynamic_max_hit_bins is not None or args.dynamic_point_time_groups_ms) and not args.dynamic_filter:
        parser.error('Dynamic settings require --dynamic-filter')
    args.dynamic_angular_support = args.dynamic_filter and not args.dynamic_allow_angular_extrapolation
    if args.dynamic_max_hit_bins is None:
        args.dynamic_max_hit_bins = 3
    if args.dynamic_point_time_groups_ms and not .5 <= args.dynamic_point_time_groups_ms <= 20:
        parser.error('dynamic point-time groups must be .5--20ms')
    if not 0 < args.rate <= 2:
        parser.error('rate must be in (0, 2]')
    bag = args.bag.resolve()
    if not bag.is_file():
        parser.error('bag does not exist')
    output = args.output_dir.resolve() if args.output_dir else ROOT / 'results/faster_lio_optimized_runs' / (bag.stem + '_' + time.strftime('%Y%m%d_%H%M%S'))
    try:
        output.relative_to(ROOT)
    except ValueError:
        parser.error('output directory must be inside this workspace')
    if output.exists():
        parser.error('output directory already exists; use a fresh directory')
    output.mkdir(parents=True)
    env = os.environ.copy()
    env.setdefault('OMP_NUM_THREADS', '2')
    env.setdefault('OPENBLAS_NUM_THREADS', '1')

    def run(script, *arguments, check=True):
        return subprocess.run([sys.executable, str(ROOT / 'scripts' / script), *map(str, arguments)], env=env, check=check)

    config = CONFIG / ('handheld_mid360.yaml' if args.calibration == 'raw' else 'handheld_mid360_cloud_pitch25.yaml')
    frontend = output / 'frontend'
    run('run_independent_lio.py', '03', '--bag', bag, '--rate', args.rate, '--output-dir', frontend,
        '--launch-arg', 'config:=' + str(config),
        '--launch-arg', 'guard_override:=' + str(args.experimental_hard_guard).lower(),
        '--launch-arg', 'raw_end_override:=' + str(args.experimental_raw_scan_end).lower(),
        '--launch-arg', 'trace_path:=' + str(frontend / 'state_trace.csv'))
    run('evaluate_independent_run.py', frontend)
    evaluation = json.loads((frontend / 'evaluation.json').read_text())
    if evaluation['failures']:
        raise RuntimeError('Frontend failed full-run checks; see ' + str(frontend / 'evaluation.json'))
    backend = output / 'backend'
    backend_arguments = ['--validate-loops', '--output-dir', backend]
    if args.backend_profile == 'observation_geometry':
        backend_arguments += ['--submap-half-window', .15, '--max-candidates', 8,
                              '--keyframe-step', 1, '--local-geometric-edges', '--plane-information',
                              '--observation-weighted-odometry', '--loop-search-radius', 5,
                              '--loop-overlap', .35, '--heldout-overlap', .30]
    else:
        backend_arguments += ['--submap-half-window', .5, '--max-candidates', 6]
    process = run('refine_independent_pose_graph.py', frontend, *backend_arguments, check=False)
    report_path = backend / 'report.json'
    backend_report = json.loads(report_path.read_text()) if report_path.exists() else {}
    if process.returncode and backend_report.get('status') != 'no_validated_loop_constraints':
        raise RuntimeError('Backend failed; inspect its logs and candidates before using any output')
    map_path = backend / 'optimized_3cm.pcd' if process.returncode == 0 else pathlib.Path(evaluation['primary_map'])
    unfiltered_map = map_path
    dynamic_report = None
    if args.dynamic_filter:
        if process.returncode != 0:
            raise RuntimeError('Dynamic filtering requires the completed backend trajectory; baseline map remains available')
        dynamic = output / 'dynamic'
        filter_arguments = ['--frame-step', .3, '--angle-deg', .7,
                            '--max-hit-bins', args.dynamic_max_hit_bins, '--output-dir', dynamic,
                            '--point-time-groups-ms', args.dynamic_point_time_groups_ms]
        if args.dynamic_angular_support:
            filter_arguments += ['--require-angular-support']
        run('filter_dynamic_map.py', frontend, backend, *filter_arguments)
        dynamic_report = json.loads((dynamic / 'report.json').read_text())
        run('audit_dynamic_filter.py', unfiltered_map, dynamic)
        dynamic_report['structural_audit'] = json.loads((dynamic / 'audit/report.json').read_text())
        map_path = dynamic / 'filtered_3cm.pcd'
    validation = output / 'validation'
    map_directory = backend if process.returncode == 0 else frontend / 'maps'
    arguments = ['--output', validation]
    if process.returncode == 0:
        arguments += ['--map-dir', map_directory]
    run('validate_faster_return_alignment.py', frontend, *arguments)
    report = dict(status='requires_full_scene_visual_validation', bag=str(bag),
                  calibration=args.calibration, raw_scan_end=args.experimental_raw_scan_end,
                  experimental_hard_guard=args.experimental_hard_guard,
                  frontend=str(frontend), backend=backend_report, map=str(map_path),
                  backend_profile=args.backend_profile,
                  all_estimator_output_frames_retained=True, no_foreign_trajectory=True,
                  backend_mode='offline after ROS1 replay; does not correct live frontend outputs',
                  dynamic_filter=dynamic_report, unfiltered_map=str(unfiltered_map),
                  validation_scope='Unfiltered baseline temporal alignment; filtered subset structure audited separately when enabled',
                  absolute_accuracy='not measured without external ground truth')
    (output / 'pipeline.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
