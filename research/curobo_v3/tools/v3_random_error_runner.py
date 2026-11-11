import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from curobo_core.adapter import matrix_pose
from curobo_core.scene import RobotState, Pose
from v3_tcp_scene_contract import GraspInput
from v3_tcp_rezero_pipeline import TcpMotionPlanner, TcpMotionRequest, tool_to_box
from v3_rezero_task_planner import migrated_assets
from v3_random_error_placement import navigation_and_box_goals


class RandomErrorRunner:
    def __init__(self):
        self.planner = None

    def run(self, evidence, home, directory, progress):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        (directory / 'input.json').write_text(json.dumps(dict(evidence=evidence, home=home), ensure_ascii=False, indent=2))
        if self.planner is None:
            progress('初始化当前新零位模型及GPU规划器…')
            self.planner = TcpMotionPlanner(migrated_assets())
        planner = self.planner
        base = np.asarray(evidence['parking_world_from_footprint'])
        state = RobotState.from_mapping(home, 0, matrix_pose(base))
        inputs = tuple(GraspInput(side, item['grasp_mode'], np.asarray(item['tcp_in_footprint'])) for side, item in evidence['inputs'].items())
        request = TcpMotionRequest(inputs, state, tuple((grasp.side, Pose((.55, .425 if grasp.side == 'left' else -.425, 1.082))) for grasp in inputs))
        planner.bind_request(request)
        result = planner.plan_to_transport(progress)
        result['tool_to_box'] = {grasp.side: tool_to_box(grasp.mode).tolist() for grasp in inputs}
        result['visualization_scene_snapshot'] = planner.snapshot.to_dict()
        (directory / 'pick.json').write_text(json.dumps(result, ensure_ascii=False))
        if result['success']:
            names = result['joint_names']
            last = np.asarray(result['frames'][-1])
            from v3_wall_ik_benchmark import ACTIVE_JOINTS
            active = [names.index(name) for name in ACTIVE_JOINTS]
            held_tools = planner.tool_matrices(last[active])
            reference = planner.conveyor_reference_targets()
            held_boxes = {grasp.side:held_tools[grasp.side]@tool_to_box(grasp.mode) for grasp in inputs}
            placed_base,box_goals = navigation_and_box_goals(held_boxes,reference,planner.conveyor_push_m)
            goals = {side:matrix_pose(pose) for side,pose in box_goals.items()}
            feedback = RobotState.from_mapping(dict(zip(names, last)), 1, matrix_pose(placed_base))
            progress('模拟导航到位，规划外部放箱目标…')
            result = planner.plan_external_placement(feedback, goals, result, progress)
            (directory / 'placement.json').write_text(json.dumps(result, ensure_ascii=False))
            if result['success']:
                result = planner.confirm_release_and_plan_home(result, True)
        result['visualization_scene_snapshot'] = planner.snapshot.to_dict()
        result['tool_to_box'] = {grasp.side: tool_to_box(grasp.mode).tolist() for grasp in inputs}
        result['input_evidence'] = evidence
        (directory / 'result.json').write_text(json.dumps(result, ensure_ascii=False))
        return result
