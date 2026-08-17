#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from motion_internal_interfaces.srv import RunDualGraspTask


DEFAULT_SEQUENCE = "1,3;1,6;4,3;4,6;4,9;7,6;7,9;7,12;10,9;10,12"
FRONT_SUCTION_BOX_IDS = {1, 3, 4, 6}
FRONT_TOOL_RPY = (math.pi, math.pi / 2.0, math.pi)
TOP_TOOL_RPY = (math.pi, 0.0, 0.0)


@dataclass(frozen=True)
class TaskSpec:
    index: int
    left_box: int
    right_box: int
    left_mode: str
    right_mode: str
    left_pose_6d: tuple[float, float, float, float, float, float]
    right_pose_6d: tuple[float, float, float, float, float, float]


def parse_pair_sequence(value: str) -> list[tuple[int, int]]:
    pairs: list[tuple[int, int]] = []
    for segment in value.split(';'):
        segment = segment.strip()
        if not segment:
            continue
        sep = ',' if ',' in segment else '/'
        parts = [part.strip() for part in segment.split(sep)]
        if len(parts) != 2:
            raise ValueError(f'invalid pair segment: {segment}')
        pairs.append((int(parts[0]), int(parts[1])))
    if not pairs:
        raise ValueError('empty pair sequence')
    return pairs


def grasp_mode_for_box(box_id: int) -> str:
    return 'front' if int(box_id) in FRONT_SUCTION_BOX_IDS else 'top_suction'


def convert_mixed_to_front(left_modes: list[str], right_modes: list[str]) -> tuple[list[str], list[str]]:
    left = list(left_modes)
    right = list(right_modes)
    for index, (left_mode, right_mode) in enumerate(zip(left, right)):
        if left_mode != right_mode and 'top_suction' in (left_mode, right_mode):
            left[index] = 'front'
            right[index] = 'front'
    return left, right


def parse_mode_sequence(value: str, count: int) -> list[str] | None:
    if not value.strip():
        return None
    aliases = {
        'front': 'front',
        'side': 'front',
        'side_suction': 'front',
        'top': 'top_suction',
        'top_suction': 'top_suction',
        'down': 'top_suction',
    }
    out: list[str] = []
    for segment in value.split(';'):
        key = segment.strip().lower()
        if not key:
            continue
        if key not in aliases:
            raise ValueError(f'invalid grasp mode: {segment}')
        out.append(aliases[key])
    if len(out) != count:
        raise ValueError(f'mode count {len(out)} != task count {count}')
    return out


def make_boxes(box_front_x: float, scene_y_shift: float) -> dict[int, tuple[float, float, float]]:
    rows_top_to_bottom = [
        [(1, 0.4), (2, 0.0), (3, -0.4)],
        [(4, 0.4), (5, 0.0), (6, -0.4)],
        [(7, 0.4), (8, 0.0), (9, -0.4)],
        [(10, 0.4), (11, 0.0), (12, -0.4)],
    ]
    row_count = len(rows_top_to_bottom)
    boxes: dict[int, tuple[float, float, float]] = {}
    for row_index, row in enumerate(rows_top_to_bottom):
        z = (float(row_count - row_index) - 0.5) * 0.5
        for box_id, y in row:
            boxes[int(box_id)] = (float(box_front_x), float(y) + float(scene_y_shift), z)
    return boxes


def position_for_box(
    box_id: int,
    mode: str,
    *,
    box_front_x: float,
    top_box_front_x: float,
    scene_y_shift: float,
    world_to_base_z: float,
    top_suction_x_offset: float,
    top_suction_z_offset: float,
) -> tuple[float, float, float]:
    boxes = make_boxes(top_box_front_x if mode == 'top_suction' else box_front_x, scene_y_shift)
    if box_id not in boxes:
        raise ValueError(f'unknown box id: {box_id}')
    x, y, z = boxes[box_id]
    if mode == 'top_suction':
        return (x + top_suction_x_offset, y, z + top_suction_z_offset - world_to_base_z)
    return (x, y, z - world_to_base_z)


def pose_6d_for_box(
    box_id: int,
    mode: str,
    **position_kwargs,
) -> tuple[float, float, float, float, float, float]:
    position = position_for_box(box_id, mode, **position_kwargs)
    orientation = TOP_TOOL_RPY if mode == 'top_suction' else FRONT_TOOL_RPY
    return (*position, *orientation)


def make_tasks(args) -> list[TaskSpec]:
    pairs = parse_pair_sequence(args.pair_sequence)
    left_modes = parse_mode_sequence(args.left_grasp_mode_sequence, len(pairs))
    right_modes = parse_mode_sequence(args.right_grasp_mode_sequence, len(pairs))
    if left_modes is None:
        left_modes = [grasp_mode_for_box(left_id) for left_id, _ in pairs]
    if right_modes is None:
        right_modes = [grasp_mode_for_box(right_id) for _, right_id in pairs]
    if args.convert_mixed_to_front:
        left_modes, right_modes = convert_mixed_to_front(left_modes, right_modes)

    top_box_front_x = args.top_box_front_x
    if top_box_front_x is None:
        top_box_front_x = args.box_front_x - args.top_approach_forward

    tasks: list[TaskSpec] = []
    for index, ((left_id, right_id), left_mode, right_mode) in enumerate(
        zip(pairs, left_modes, right_modes), start=1
    ):
        tasks.append(
            TaskSpec(
                index=index,
                left_box=left_id,
                right_box=right_id,
                left_mode=left_mode,
                right_mode=right_mode,
                left_pose_6d=pose_6d_for_box(
                    left_id,
                    left_mode,
                    box_front_x=args.box_front_x,
                    top_box_front_x=top_box_front_x,
                    scene_y_shift=args.scene_y_shift,
                    world_to_base_z=args.world_to_base_z,
                    top_suction_x_offset=args.top_suction_x_offset,
                    top_suction_z_offset=args.top_suction_z_offset,
                ),
                right_pose_6d=pose_6d_for_box(
                    right_id,
                    right_mode,
                    box_front_x=args.box_front_x,
                    top_box_front_x=top_box_front_x,
                    scene_y_shift=args.scene_y_shift,
                    world_to_base_z=args.world_to_base_z,
                    top_suction_x_offset=args.top_suction_x_offset,
                    top_suction_z_offset=args.top_suction_z_offset,
                ),
            )
        )
    return tasks


def select_tasks(tasks: list[TaskSpec], args) -> list[TaskSpec]:
    if args.only:
        wanted = {int(item) for item in args.only.split(',') if item.strip()}
        return [task for task in tasks if task.index in wanted]
    return [task for task in tasks if task.index >= args.start_index]


class SequenceClient(Node):
    def __init__(self, args):
        super().__init__('lhy_dual_grasp_sequence_client')
        self.args = args
        self.client = self.create_client(RunDualGraspTask, args.service_name)

    def wait_service(self) -> None:
        if not self.client.wait_for_service(timeout_sec=self.args.service_timeout_s):
            raise TimeoutError(f'service not available: {self.args.service_name}')

    @staticmethod
    def assign_pose_6d(target, values: tuple[float, float, float, float, float, float]) -> None:
        (
            target.pose_6d.x,
            target.pose_6d.y,
            target.pose_6d.z,
            target.pose_6d.roll,
            target.pose_6d.pitch,
            target.pose_6d.yaw,
        ) = values

    def call_task(self, task: TaskSpec):
        request = RunDualGraspTask.Request()
        request.request_id = f'{self.args.request_prefix}_{task.index:02d}_L{task.left_box}_R{task.right_box}'
        self.assign_pose_6d(request.left, task.left_pose_6d)
        self.assign_pose_6d(request.right, task.right_pose_6d)
        request.left.grasp_mode = task.left_mode
        request.right.grasp_mode = task.right_mode
        request.execute = bool(self.args.execute)

        future = self.client.call_async(request)
        deadline = time.monotonic() + self.args.result_timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError(f'task {request.request_id} result timeout')
        return future.result()


def parse_args():
    parser = argparse.ArgumentParser(description='Send the current 10 dual-grasp endpoint tasks.')
    parser.add_argument('--service-name', default='/robot_motion/run_dual_grasp_task')
    parser.add_argument('--service-timeout-s', type=float, default=30.0)
    parser.add_argument('--result-timeout-s', type=float, default=300.0)
    parser.add_argument('--request-prefix', default='lhy_dual_grasp')
    parser.add_argument('--frame-id', default='base_link')
    parser.add_argument('--scene-id', default='manual_box_stack')
    parser.add_argument('--state-id', default='live_joint_states')
    parser.add_argument('--pair-sequence', default=DEFAULT_SEQUENCE)
    parser.add_argument('--left-grasp-mode-sequence', default='')
    parser.add_argument('--right-grasp-mode-sequence', default='')
    parser.add_argument(
        '--convert-mixed-to-front',
        action='store_true',
        default=False,
        help='Legacy-only sender conversion. Normally the algorithm classifies mixed tasks itself.',
    )
    parser.add_argument('--box-front-x', type=float, default=0.925)
    parser.add_argument('--top-approach-forward', type=float, default=0.30)
    parser.add_argument('--top-box-front-x', type=float, default=None)
    parser.add_argument('--scene-y-shift', type=float, default=0.0)
    parser.add_argument('--world-to-base-z', type=float, default=0.202094)
    parser.add_argument('--top-suction-x-offset', type=float, default=0.15)
    parser.add_argument('--top-suction-z-offset', type=float, default=0.25)
    parser.add_argument('--velocity-scale', type=float, default=1.0)
    parser.add_argument('--acceleration-scale', type=float, default=1.0)
    parser.add_argument('--start-index', type=int, default=1)
    parser.add_argument('--only', default='', help='Run only 1-based task indices, e.g. 4 or 1,2,3.')
    parser.add_argument('--list', action='store_true', help='Print tasks and exit.')
    parser.add_argument('--continue-on-failure', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--yes-execute', action='store_true')
    parser.add_argument('--execute-backend', choices=['planner-live', 'service'], default='planner-live')
    parser.add_argument('--planner-live-script', default='/home/ar/lhy_dev/ros2_ws/src/alfa_robot_moveit_config/scripts/execute_l6_r8_real_live.py')
    parser.add_argument('--planner-output-root', default='/home/ar/lhy_dev/data/ik_benchmark/live_real_execution')
    parser.add_argument('--hz', type=float, default=10.0)
    parser.add_argument('--max-joint-speed-deg-s', type=float, default=10.0)
    parser.add_argument('--max-updown-speed-m-s', type=float, default=0.05)
    parser.add_argument('--updown-acceleration-m-s2', type=float, default=0.05)
    parser.add_argument('--updown-deceleration-m-s2', type=float, default=0.05)
    parser.add_argument('--planner-service-timeout-s', type=float, default=120.0)
    parser.add_argument('--planner-compute-timeout-s', type=float, default=10.0)
    parser.add_argument('--no-send-updown', dest='send_updown', action='store_false', default=True)
    return parser.parse_args()


def print_tasks(tasks: list[TaskSpec]) -> None:
    for task in tasks:
        print(
            f'{task.index:02d}: L{task.left_box}/R{task.right_box} '
            f'modes=({task.left_mode},{task.right_mode}) '
            f'Lpose6d=({",".join(f"{value:.3f}" for value in task.left_pose_6d)}) '
            f'Rpose6d=({",".join(f"{value:.3f}" for value in task.right_pose_6d)})',
            flush=True,
        )



def run_planner_live_task(args, task: TaskSpec) -> int:
    cmd = [
        '/usr/bin/python3',
        args.planner_live_script,
        '--executor-mode', 'real',
        '--ros-domain-id', 'inherit',
        '--output-root', args.planner_output_root,
        '--left-box-id', str(task.left_box),
        '--right-box-id', str(task.right_box),
        '--left-grasp-mode', task.left_mode,
        '--right-grasp-mode', task.right_mode,
        '--box-front-x', str(args.box_front_x),
        '--top-approach-forward', str(args.top_approach_forward),
        '--scene-y-shift', str(args.scene_y_shift),
        '--top-suction-x-offset', str(args.top_suction_x_offset),
        '--top-suction-z-offset', str(args.top_suction_z_offset),
        '--fixed-updown-from-joint-states',
        '--home-start', 'current',
        '--trajectory-timing-source', 'snapshot',
        '--hz', str(args.hz),
        '--max-joint-speed-deg-s', str(args.max_joint_speed_deg_s),
        '--max-updown-speed-m-s', str(args.max_updown_speed_m_s),
        '--updown-acceleration-m-s2', str(args.updown_acceleration_m_s2),
        '--updown-deceleration-m-s2', str(args.updown_deceleration_m_s2),
        '--service-timeout', str(args.planner_service_timeout_s),
        '--compute-timeout', str(args.planner_compute_timeout_s),
        '--candidate-limit', '64',
        '--extract-workers', '16',
        '--extract-success-quorum', '3',
        '--extract-quality-success-quorum', '1',
        '--extract-quality-loaded-distance-sum', '5.0',
        '--extract-rollout-mode', 'box_pose_rrt',
        '--loaded-planning-mode', 'shortcut',
        '--loaded-candidate-limit', '8',
        '--loaded-workers', '8',
    ]
    if args.top_box_front_x is not None:
        cmd.extend(['--top-box-front-x', str(args.top_box_front_x)])
    if args.send_updown:
        cmd.append('--send-updown')
    print('  backend=planner-live', flush=True)
    print('  command=' + ' '.join(cmd), flush=True)
    completed = subprocess.run(cmd, check=False)
    return int(completed.returncode)

def main() -> int:
    args = parse_args()
    if args.execute and args.dry_run:
        print('Invalid: --execute and --dry-run are mutually exclusive.', file=sys.stderr)
        return 2
    if not args.execute and not args.dry_run and not args.list:
        print('Refusing to do nothing ambiguous: use --dry-run, --execute --yes-execute, or --list.', file=sys.stderr)
        return 2
    if args.execute and not args.yes_execute:
        print('Refusing real execution: add --yes-execute after confirming hardware safety.', file=sys.stderr)
        return 2

    tasks = select_tasks(make_tasks(args), args)
    print_tasks(tasks)
    if args.list:
        return 0

    if args.execute and args.execute_backend == 'planner-live':
        ok_count = 0
        for task in tasks:
            print(f'\nExecuting task {task.index:02d}: L{task.left_box}/R{task.right_box}', flush=True)
            rc = run_planner_live_task(args, task)
            if rc == 0:
                ok_count += 1
                print(f'  success=True task={task.index:02d}', flush=True)
            else:
                print(f'  success=False task={task.index:02d} returncode={rc}', file=sys.stderr, flush=True)
                if not args.continue_on_failure:
                    print(f'Stopping at failed task {task.index:02d}.', file=sys.stderr, flush=True)
                    return rc or 1
        print(f'\nDone: {ok_count}/{len(tasks)} tasks succeeded.', flush=True)
        return 0 if ok_count == len(tasks) else 1

    rclpy.init()
    node = SequenceClient(args)
    try:
        node.wait_service()
        ok_count = 0
        for task in tasks:
            print(f'\nSending task {task.index:02d}: L{task.left_box}/R{task.right_box}', flush=True)
            response = node.call_task(task)
            print(
                f'  success={response.success} state={response.state} '
                f'request_id={response.request_id}',
                flush=True,
            )
            print(f'  message={response.message}', flush=True)
            if response.success:
                ok_count += 1
            elif not args.continue_on_failure:
                print(f'Stopping at failed task {task.index:02d}.', file=sys.stderr, flush=True)
                return 1
        print(f'\nDone: {ok_count}/{len(tasks)} tasks succeeded.', flush=True)
        return 0 if ok_count == len(tasks) else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
