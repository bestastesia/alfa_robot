#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEST_SCRIPT="$REPO_ROOT/ros2_ws/src/alfa_robot_moveit_config/test/test_v3_box_wall_sequence.py"
VARIANTS="topk,shortcut_ruckig,chomp_ruckig"
SEEDS="104729,104731,104733"
TOP_K_COMPLETE=3
SAMPLE_PERIOD=0.05
OUTPUT_DIR="$REPO_ROOT/../日志/wall_trajectory_variants"
X=0.50
INITIAL_POSE=arms_down
WALL_BOTTOM_Z=0.000001
FRONT_RATIO=0.95
EMPTY_VELOCITY_SCALING=0.50
EMPTY_ACCELERATION_SCALING=0.50
LOADED_VELOCITY_SCALING=0.25
LOADED_ACCELERATION_SCALING=0.25
ARM_MAX_JERK=2.0
HEAD_MAX_JERK=2.0
UPDOWN_MAX_JERK=0.30
ROS_DOMAIN=197
ROS_LOCALHOST=1
WALL_CONTEXT=full
REQUIRE_COMPLETE=1
SUMMARIZE_ONLY=0
VALIDATOR=""
ENVIRONMENT_FILE=""

usage() {
  cat <<'EOF'
Usage: tools/compare_wall_trajectory_variants.sh [options]

Runs wall transfers sequentially; each variant/seed gets its own artifacts and live.rrd.

  --variants LIST                 Comma list: topk,shortcut_ruckig,chomp_ruckig
  --seeds LIST                    Comma-separated integer seeds
  --top-k-complete N              Complete candidates per transfer, 1..8
  --sample-period SEC             Replay sampling period (default: 0.05)
  --output-dir PATH               Output root
  --x METERS                      Wall distance (default: 0.50)
  --initial-pose POSE             home or arms_down (default: arms_down)
  --wall-bottom-z METERS          Wall bottom Z (default: 0.000001)
  --front-ratio RATIO             Set comfort min/preferred/max (default: 0.95)
  --empty-velocity-scaling VALUE  Default: 0.50
  --empty-acceleration-scaling V  Default: 0.50
  --loaded-velocity-scaling VALUE Default: 0.25
  --loaded-acceleration-scaling V Default: 0.25
  --arm-max-jerk VALUE            rad/s^3 (default: 2.0)
  --head-max-jerk VALUE           rad/s^3 (default: 2.0)
  --updown-max-jerk VALUE         m/s^3 (default: 0.30)
  --ros-domain-id N               Default: 197
  --ros-localhost-only 1          Acceptance runner currently requires 1
  --wall-context VALUE            full or sequence_prefix
  --validator PATH                Optional independent replay validator
  --environment-file PATH         Optional collision fixture
  --require-complete              Fail overall if any run is incomplete (default)
  --allow-incomplete              Keep summaries without requiring success
  --summarize-only                Rebuild summaries from existing artifacts
  -h, --help                      Show this help
EOF
}

need_value() { [[ $# -ge 2 ]] || { echo "missing value for $1" >&2; exit 2; }; }
while (($#)); do
  case "$1" in
    --variants) need_value "$@"; VARIANTS=$2; shift 2 ;;
    --seeds) need_value "$@"; SEEDS=$2; shift 2 ;;
    --top-k-complete) need_value "$@"; TOP_K_COMPLETE=$2; shift 2 ;;
    --sample-period|--trajectory-sample-period) need_value "$@"; SAMPLE_PERIOD=$2; shift 2 ;;
    --output-dir) need_value "$@"; OUTPUT_DIR=$2; shift 2 ;;
    --x) need_value "$@"; X=$2; shift 2 ;;
    --initial-pose) need_value "$@"; INITIAL_POSE=$2; shift 2 ;;
    --wall-bottom-z) need_value "$@"; WALL_BOTTOM_Z=$2; shift 2 ;;
    --front-ratio) need_value "$@"; FRONT_RATIO=$2; shift 2 ;;
    --empty-velocity-scaling) need_value "$@"; EMPTY_VELOCITY_SCALING=$2; shift 2 ;;
    --empty-acceleration-scaling) need_value "$@"; EMPTY_ACCELERATION_SCALING=$2; shift 2 ;;
    --loaded-velocity-scaling) need_value "$@"; LOADED_VELOCITY_SCALING=$2; shift 2 ;;
    --loaded-acceleration-scaling) need_value "$@"; LOADED_ACCELERATION_SCALING=$2; shift 2 ;;
    --arm-max-jerk) need_value "$@"; ARM_MAX_JERK=$2; shift 2 ;;
    --head-max-jerk) need_value "$@"; HEAD_MAX_JERK=$2; shift 2 ;;
    --updown-max-jerk) need_value "$@"; UPDOWN_MAX_JERK=$2; shift 2 ;;
    --ros-domain-id) need_value "$@"; ROS_DOMAIN=$2; shift 2 ;;
    --ros-localhost-only) need_value "$@"; ROS_LOCALHOST=$2; shift 2 ;;
    --wall-context) need_value "$@"; WALL_CONTEXT=$2; shift 2 ;;
    --validator) need_value "$@"; VALIDATOR=$2; shift 2 ;;
    --environment-file) need_value "$@"; ENVIRONMENT_FILE=$2; shift 2 ;;
    --require-complete) REQUIRE_COMPLETE=1; shift ;;
    --allow-incomplete) REQUIRE_COMPLETE=0; shift ;;
    --summarize-only) SUMMARIZE_ONLY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

IFS=',' read -r -a variant_list <<< "$VARIANTS"
IFS=',' read -r -a seed_list <<< "$SEEDS"
((${#variant_list[@]} && ${#seed_list[@]})) || { echo "variants and seeds must not be empty" >&2; exit 2; }
for variant in "${variant_list[@]}"; do
  case "$variant" in topk|shortcut_ruckig|chomp_ruckig) ;; *) echo "unsupported variant: $variant" >&2; exit 2 ;; esac
done
for seed in "${seed_list[@]}"; do [[ "$seed" =~ ^[0-9]+$ ]] || { echo "invalid seed: $seed" >&2; exit 2; }; done
[[ "$TOP_K_COMPLETE" =~ ^[1-8]$ ]] || { echo "top-k-complete must be 1..8" >&2; exit 2; }
[[ "$INITIAL_POSE" == home || "$INITIAL_POSE" == arms_down ]] || { echo "invalid initial pose" >&2; exit 2; }
[[ "$WALL_CONTEXT" == full || "$WALL_CONTEXT" == sequence_prefix ]] || { echo "invalid wall context" >&2; exit 2; }
[[ "$ROS_DOMAIN" =~ ^[0-9]+$ ]] && ((ROS_DOMAIN <= 232)) || { echo "ROS domain must be 0..232" >&2; exit 2; }
[[ "$ROS_LOCALHOST" == 1 ]] || { echo "ros-localhost-only must be 1: the existing acceptance runner enforces it" >&2; exit 2; }
python3 - "$X" "$WALL_BOTTOM_Z" "$FRONT_RATIO" "$SAMPLE_PERIOD" \
  "$EMPTY_VELOCITY_SCALING" "$EMPTY_ACCELERATION_SCALING" \
  "$LOADED_VELOCITY_SCALING" "$LOADED_ACCELERATION_SCALING" \
  "$ARM_MAX_JERK" "$HEAD_MAX_JERK" "$UPDOWN_MAX_JERK" <<'PY'
import math, sys
values = [float(value) for value in sys.argv[1:]]
if not all(map(math.isfinite, values)) or values[3] <= 0 or any(not 0 < value <= 1 for value in values[4:8]) or any(value <= 0 for value in values[8:]):
    raise SystemExit('invalid finite/scaling/jerk parameter')
PY
OUTPUT_DIR="$(realpath -m "$OUTPUT_DIR")"
mkdir -p "$OUTPUT_DIR"

summarize() {
  python3 - "$OUTPUT_DIR" "$VARIANTS" "$SEEDS" "$REQUIRE_COMPLETE" <<'PY'
import csv
import json
import math
import sys
from pathlib import Path

root = Path(sys.argv[1])
variants = sys.argv[2].split(',')
seeds = [int(value) for value in sys.argv[3].split(',')]
require_complete = bool(int(sys.argv[4]))
metric_names = ('shortcut_ms', 'chomp_ms', 'totg_ms', 'ruckig_ms')
fields = ('success', 'completed_count', 'segment_count', 'dual_success_count', 'fallback_count',
          'planning_total_ms', 'execution_duration_s', 'selection_score_sum', 'selection_score_mean',
          *metric_names, 'max_velocity', 'max_acceleration', 'max_jerk',
          'requested_variant', 'effective_variant', 'variant_identity_valid', 'seed_identity_valid', 'seed',
          'sequence_path', 'sequence_exists', 'sequence_nonempty',
          'rrd_path', 'rrd_exists', 'rrd_nonempty',
          'command_path', 'command_exists', 'command_nonempty',
          'runner_exit_code', 'run_complete')
rows = []
for variant in variants:
    for seed in seeds:
        artifacts = root / 'artifacts' / variant / f'seed_{seed}'
        sequence_path = artifacts / 'sequence.json'
        rrd_path = artifacts / 'live.rrd'
        command_path = artifacts / 'command.txt'
        def flags(path):
            exists = path.is_file()
            return exists, exists and path.stat().st_size > 0
        sequence_exists, sequence_nonempty = flags(sequence_path)
        rrd_exists, rrd_nonempty = flags(rrd_path)
        command_exists, command_nonempty = flags(command_path)
        row = {key: '' for key in fields}
        row.update(requested_variant=variant, effective_variant='', variant_identity_valid=False, seed_identity_valid=False, seed=seed,
                   sequence_path=str(sequence_path), sequence_exists=sequence_exists,
                   sequence_nonempty=sequence_nonempty, rrd_path=str(rrd_path),
                   rrd_exists=rrd_exists, rrd_nonempty=rrd_nonempty,
                   command_path=str(command_path), command_exists=command_exists,
                   command_nonempty=command_nonempty, runner_exit_code='', run_complete=False)
        exit_path = artifacts / 'runner_exit_code.txt'
        if exit_path.exists():
            row['runner_exit_code'] = exit_path.read_text().strip()
        if sequence_nonempty:
            task = json.loads(sequence_path.read_text())
            segments = task.get('boxes', [])
            successful = [segment for segment in segments if segment.get('success', False)]
            scores = [float(segment['selection_score']) for segment in successful
                      if isinstance(segment.get('selection_score'), (int, float)) and math.isfinite(segment['selection_score'])]
            metrics = task.get('metrics', {})
            def metric(name):
                if isinstance(metrics.get(name), (int, float)):
                    return metrics[name]
                return sum(segment.get('metrics', {}).get(name, 0.0) for segment in segments)
            effective = sorted({segment.get('effective_trajectory_variant') for segment in segments
                                if segment.get('effective_trajectory_variant')})
            task_requested = task.get('requested_trajectory_variant')
            segment_requested = [segment.get('requested_trajectory_variant') for segment in segments]
            requested = sorted({value for value in [task_requested, *segment_requested] if value})
            row.update(
                success=bool(task.get('success', False)),
                completed_count=task.get('completed_count', 0),
                segment_count=task.get('segment_count', len(segments)),
                dual_success_count=task.get('dual_success_count', 0),
                fallback_count=task.get('fallback_count', 0),
                planning_total_ms=task.get('total_ms', 0.0),
                execution_duration_s=sum(float(segment.get('execution_duration_s', 0.0)) for segment in successful),
                selection_score_sum=sum(scores),
                selection_score_mean=sum(scores) / len(scores) if scores else '',
                max_velocity=max([float(task.get('max_velocity', 0.0))] + [float(s.get('max_velocity', 0.0)) for s in successful]),
                max_acceleration=max([float(task.get('max_acceleration', 0.0))] + [float(s.get('max_acceleration', 0.0)) for s in successful]),
                max_jerk=max([float(task.get('max_jerk', 0.0))] + [float(s.get('max_jerk', 0.0)) for s in successful]),
                requested_variant='|'.join(requested),
                effective_variant='|'.join(effective) or task.get('effective_trajectory_variant', ''),
                variant_identity_valid=(task_requested == variant and
                                        all(value == variant for value in segment_requested)),
                seed_identity_valid=(task.get('planning_seed') == seed),
                **{name: metric(name) for name in metric_names},
            )
        row['run_complete'] = (bool(row['success']) and row['runner_exit_code'] == '0'
            and row['completed_count'] == 25 and row['segment_count'] == 15
            and row['dual_success_count'] == 10 and row['fallback_count'] == 0
            and row['variant_identity_valid'] and row['seed_identity_valid']
            and bool(task.get('full_dual_pass', False))
            and all((sequence_nonempty, rrd_nonempty, command_nonempty)))
        rows.append(row)
(root / 'summary.json').write_text(json.dumps({'runs': rows}, indent=2, ensure_ascii=False) + '\n')
with (root / 'summary.csv').open('w', newline='') as stream:
    writer = csv.DictWriter(stream, fieldnames=fields)
    writer.writeheader(); writer.writerows(rows)
with (root / 'summary.md').open('w') as stream:
    stream.write('| ' + ' | '.join(fields) + ' |\n|' + '|'.join(['---'] * len(fields)) + '|\n')
    for row in rows:
        stream.write('| ' + ' | '.join(str(row[key]).replace('|', '/') for key in fields) + ' |\n')
print(f'wrote {root / "summary.csv"}, {root / "summary.json"}, {root / "summary.md"}')
if require_complete and not all(row['run_complete'] for row in rows):
    raise SystemExit(1)
PY
}

if ((SUMMARIZE_ONLY)); then summarize; exit 0; fi
[[ -f "$TEST_SCRIPT" ]] || { echo "missing runner: $TEST_SCRIPT" >&2; exit 2; }
set +u
source "$REPO_ROOT/tools/ros_humble_env.sh"
source "$REPO_ROOT/ros2_ws/install/setup.bash"
set -u
REAL_ROS2="$(command -v ros2)"
SHIM_DIR="$OUTPUT_DIR/.ros2_shim"
mkdir -p "$SHIM_DIR"
cat > "$SHIM_DIR/ros2" <<'SHIM'
#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" == launch && "${2:-}" == alfa_robot_moveit_config && "${3:-}" == v3_box_wall_sequence_demo.launch.py ]]; then
  exec "$COMPARE_REAL_ROS2" "$@" \
    "trajectory_sample_period:=$COMPARE_SAMPLE_PERIOD" \
    "empty_velocity_scaling:=$COMPARE_EMPTY_VELOCITY_SCALING" \
    "empty_acceleration_scaling:=$COMPARE_EMPTY_ACCELERATION_SCALING" \
    "loaded_velocity_scaling:=$COMPARE_LOADED_VELOCITY_SCALING" \
    "loaded_acceleration_scaling:=$COMPARE_LOADED_ACCELERATION_SCALING" \
    "arm_max_jerk:=$COMPARE_ARM_MAX_JERK" \
    "head_max_jerk:=$COMPARE_HEAD_MAX_JERK" \
    "updown_max_jerk:=$COMPARE_UPDOWN_MAX_JERK"
fi
exec "$COMPARE_REAL_ROS2" "$@"
SHIM
chmod +x "$SHIM_DIR/ros2"
export COMPARE_REAL_ROS2="$REAL_ROS2" COMPARE_SAMPLE_PERIOD="$SAMPLE_PERIOD"
export COMPARE_EMPTY_VELOCITY_SCALING="$EMPTY_VELOCITY_SCALING" COMPARE_EMPTY_ACCELERATION_SCALING="$EMPTY_ACCELERATION_SCALING"
export COMPARE_LOADED_VELOCITY_SCALING="$LOADED_VELOCITY_SCALING" COMPARE_LOADED_ACCELERATION_SCALING="$LOADED_ACCELERATION_SCALING"
export COMPARE_ARM_MAX_JERK="$ARM_MAX_JERK" COMPARE_HEAD_MAX_JERK="$HEAD_MAX_JERK" COMPARE_UPDOWN_MAX_JERK="$UPDOWN_MAX_JERK"
export ROS_DOMAIN_ID="$ROS_DOMAIN" ROS_LOCALHOST_ONLY="$ROS_LOCALHOST"

failed=0
for variant in "${variant_list[@]}"; do
  for seed in "${seed_list[@]}"; do
    artifacts="$OUTPUT_DIR/artifacts/$variant/seed_$seed"
    mkdir -p "$artifacts"
    rm -f "$artifacts/sequence.json" "$artifacts/live.rrd" "$artifacts/runner_exit_code.txt"
    command=(python3 "$TEST_SCRIPT" --artifacts "$artifacts" --x "$X" --initial-pose "$INITIAL_POSE"
      --wall-bottom-z "$WALL_BOTTOM_Z" --seed "$seed" --trajectory-variant "$variant"
      --top-k-complete "$TOP_K_COMPLETE" --front-ratio "$FRONT_RATIO" --wall-context "$WALL_CONTEXT"
      --incremental-rerun)
    ((REQUIRE_COMPLETE)) && command+=(--require-complete)
    [[ -n "$VALIDATOR" ]] && command+=(--validator "$VALIDATOR")
    [[ -n "$ENVIRONMENT_FILE" ]] && command+=(--environment-file "$ENVIRONMENT_FILE")
    {
      printf 'cd %q\n' "$REPO_ROOT"
      printf 'set +u\nsource %q\nsource %q\nset -u\n' \
        "$REPO_ROOT/tools/ros_humble_env.sh" "$REPO_ROOT/ros2_ws/install/setup.bash"
      printf 'export ROS_DOMAIN_ID=%q ROS_LOCALHOST_ONLY=%q\n' "$ROS_DOMAIN" "$ROS_LOCALHOST"
      printf 'export COMPARE_REAL_ROS2=%q COMPARE_SAMPLE_PERIOD=%q COMPARE_EMPTY_VELOCITY_SCALING=%q COMPARE_EMPTY_ACCELERATION_SCALING=%q COMPARE_LOADED_VELOCITY_SCALING=%q COMPARE_LOADED_ACCELERATION_SCALING=%q COMPARE_ARM_MAX_JERK=%q COMPARE_HEAD_MAX_JERK=%q COMPARE_UPDOWN_MAX_JERK=%q\n' \
        "$REAL_ROS2" "$SAMPLE_PERIOD" "$EMPTY_VELOCITY_SCALING" "$EMPTY_ACCELERATION_SCALING" \
        "$LOADED_VELOCITY_SCALING" "$LOADED_ACCELERATION_SCALING" \
        "$ARM_MAX_JERK" "$HEAD_MAX_JERK" "$UPDOWN_MAX_JERK"
      printf 'env PATH=%q ' "$SHIM_DIR:$PATH"
      printf '%q ' "${command[@]}"
      printf '\n'
    } > "$artifacts/command.txt"
    echo "=== variant=$variant seed=$seed artifacts=$artifacts ==="
    set +e
    PATH="$SHIM_DIR:$PATH" "${command[@]}" 2>&1 | tee "$artifacts/test.log"
    status=${PIPESTATUS[0]}
    set -e
    printf '%s\n' "$status" > "$artifacts/runner_exit_code.txt"
    ((status == 0)) || failed=1
  done
done
summarize || failed=1
exit "$failed"
