import ctypes
import json
import math
from pathlib import Path
import time
import threading

import numpy as np
from scipy.spatial.transform import Rotation
import torch
import yourdfpy

from curobo.types import JointState

from v3_batched_loaded_search import GpuValidity, densify
from .backend import CuroboBackend, WORKSPACE_ROOT, goal
from v3_wall_ik_benchmark import ACTIVE_JOINTS, canonical_side_suction_quaternion_wxyz
from .adapter import task_attachments
from .contracts import PlanRequest
from .scene import Pose, SceneStore
from .distance_metric import joint_distance_weights


class CycleBlocked(RuntimeError):
    def __init__(self, stage, details, partial=None):
        super().__init__(f"{stage}: {details}")
        self.stage = stage
        self.details = details
        self.partial = partial


class AnalyticBridge:
    def __init__(self):
        path = WORKSPACE_ROOT / "generated/v3_analytic_071cb95/libv3_analytic_bridge.so"
        self.library = ctypes.CDLL(str(path))
        array = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
        self.library.v3_swivel.argtypes = [ctypes.c_int, array]
        self.library.v3_swivel.restype = ctypes.c_double
        self.library.v3_solve.argtypes = [ctypes.c_int, array, ctypes.c_double, array, array, ctypes.c_int]
        self.library.v3_solve.restype = ctypes.c_int

    def swivel(self, side, values):
        return self.library.v3_swivel(side, np.ascontiguousarray(values, dtype=np.float64))

    def solve(self, side, target, swivel, seed):
        output = np.zeros((16, 7), dtype=np.float64)
        count = self.library.v3_solve(
            side, np.ascontiguousarray(target, dtype=np.float64), swivel,
            np.ascontiguousarray(seed, dtype=np.float64), output, len(output),
        )
        return output[:count]


class FullCyclePlanner(CuroboBackend):
    analytic_bridge_type = AnalyticBridge

    def __init__(self, args):
        super().__init__(args)
        self.analytic = self.analytic_bridge_type()
        self.fk_robot = yourdfpy.URDF.load(args.urdf, load_meshes=False, build_scene_graph=True)
        self.last_partial = None
        self._planning_lock = threading.Lock()

    def checker(self, boxes, payload=False, mobile=False):
        return self.collision_checker(boxes, payload=payload, mobile=mobile)

    def contact_poses_for_task(self, boxes):
        output = {}
        for side, box_id in boxes.items():
            item = self.snapshot.object(f"wall_box_{box_id:02d}")
            position = np.asarray(item.pose.position).copy()
            position[0] -= item.dimensions_m[0] * 0.5
            output[f"{side}_tool0"] = {"position": position, "quaternion": canonical_side_suction_quaternion_wxyz(side)}
        return output

    def plan_contact_approach(self, contact, checker, validity, seed):
        return self.joint_plan(self.home_values, contact, checker, validity, seed)

    def demo_request(self, boxes):
        return PlanRequest(self.snapshot, tuple(boxes.items()), tuple(
            (name, Pose(tuple(pose["position"]), tuple(pose["quaternion"])))
            for name, pose in self.target_poses.items()))

    def plan_request(self, request, progress=lambda text: None):
        if not self._planning_lock.acquire(blocking=False):
            raise RuntimeError("planning core is busy")
        try:
            return self._plan_request(request, progress)
        finally:
            self._planning_lock.release()

    def _plan_request(self, request, progress):
        if request.snapshot.state.base_pose != Pose():
            raise ValueError("current full cycle requires fixed base at map origin")
        state = dict(zip(request.snapshot.state.joint_names, request.snapshot.state.positions))
        if any(abs(state.get(name, 0.0)) > 1e-12 for name in ("head_joint", "head_pitch_joint")):
            raise ValueError("current model locks both head joints at zero")
        self.set_snapshot(request.snapshot)
        self.home_values = np.asarray(request.snapshot.state.ordered(ACTIVE_JOINTS), dtype=np.float32)
        self.target_poses = {name: {"position": pose.position, "quaternion": pose.quaternion_wxyz}
                             for name, pose in request.targets}
        boxes = dict(request.tasks)
        for box_id in boxes.values():
            item = request.snapshot.object(f"wall_box_{box_id:02d}")
            if item.pose.quaternion_wxyz != Pose().quaternion_wxyz:
                raise ValueError("current extraction contract requires axis-aligned task boxes")
            if not np.allclose(item.dimensions_m, self.box_fit["dimensions_m"], atol=1e-12):
                raise ValueError("task box dimensions differ from payload sphere model")
        try:
            result = self._full_cycle(boxes, progress)
        except CycleBlocked as error:
            result = error.partial or self.last_partial
            result["blocked_stage"] = error.stage
            result["blocked_details"] = str(error.details)
        except RuntimeError as error:
            result = self.last_partial or {"success": False, "frames": [], "phases": [], "payload": []}
            result["blocked_stage"] = "planning_failure"
            result["blocked_details"] = str(error)
        result["request_id"] = request.identity
        result["model_id"] = request.snapshot.model_id
        result["scene_id"] = request.snapshot.identity
        result["scene_revision"] = request.snapshot.revision
        result["state_id"] = request.snapshot.state.identity
        result["policy"] = request.snapshot.to_dict()["policy"]
        result["error"] = None if result["success"] else {
            "code": "PLANNING_FAILED", "stage": result.get("blocked_stage"),
            "message": result.get("blocked_details"),
        }
        result["predicted_scene_events"] = self._predicted_events(request, result)
        return result

    def plan_from_store(self, store, tasks, targets, progress=lambda text: None):
        request = PlanRequest(store.snapshot(), tuple(tasks.items()), tuple(targets.items()))
        result = self.plan_request(request, progress)
        if not store.is_current(request.snapshot):
            result["success"] = False
            result["error"] = {"code": "SCENE_CHANGED", "stage": "acceptance",
                               "message": "scene or robot state changed during planning"}
        return result

    def full_cycle(self, boxes, progress=lambda text: None):
        return self.plan_request(self.demo_request(boxes), progress)

    def _predicted_events(self, request, result):
        predicted = SceneStore(request.snapshot)
        events = []
        attachments = self.task_attachments(dict(request.tasks))
        for index, phase in enumerate(result.get("phases", ())):
            if phase == "attach" and not events:
                values = dict(zip(result["joint_names"], result["frames"][index]))
                predicted.update_state(predicted.snapshot().state.with_positions(values),
                                       predicted.snapshot().revision)
                predicted.attach_many(attachments, predicted.snapshot().revision)
                events.append({"phase": phase, "frame_index": index,
                               "snapshot": predicted.snapshot().to_dict()})
            elif phase == "release" and len(events) == 1:
                values = dict(zip(result["joint_names"], result["frames"][index]))
                base_pose = Pose((values["base_x"], values["base_y"], 0.0),
                                 (math.cos(values["base_yaw"] / 2), 0.0, 0.0,
                                  math.sin(values["base_yaw"] / 2)))
                predicted.update_state(predicted.snapshot().state.with_positions(values, base_pose),
                                       predicted.snapshot().revision)
                for attachment in attachments:
                    predicted.release(attachment.object_id, predicted.snapshot().revision)
                events.append({"phase": phase, "frame_index": index,
                               "snapshot": predicted.snapshot().to_dict()})
        return events

    def fk(self, values):
        mapping = dict(zip(ACTIVE_JOINTS, values))
        self.fk_robot.update_cfg({name: mapping.get(name, 0.0)
                                  for name in self.fk_robot.actuated_joint_names})

    def analytic_extract(self, contact, empty_validity, loaded_validity):
        self.last_extract_frames = np.asarray(contact, dtype=np.float64).reshape(1, -1)
        self.fk(contact)
        world_to_carriage = np.linalg.inv(
            self.fk_robot.get_transform("arm_carriage", self.fk_robot.base_link))
        targets = {side: self.fk_robot.get_transform(f"{side}_tool0", self.fk_robot.base_link).copy()
                   for side in ("left", "right")}
        swivels = [self.analytic.swivel(side, contact[1 + side * 7:8 + side * 7])
                   for side in range(2)]
        rows = [np.asarray(contact, dtype=np.float64)]
        for distance in np.linspace(0.01, 0.35, 35):
            next_row = rows[-1].copy()
            for side_index, side in enumerate(("left", "right")):
                target = targets[side].copy()
                target[0, 3] -= distance
                offset = 1 + side_index * 7
                seed = rows[-1][offset:offset + 7]
                solutions = self.analytic.solve(side_index, world_to_carriage @ target,
                                                swivels[side_index], seed)
                if len(solutions) == 0:
                    self.last_extract_frames = np.asarray(rows)
                    return None, f"{side}固定ψ解析无解@{distance*100:.1f}cm"
                nearest = np.argmin(np.sum((solutions - seed) ** 2, axis=1))
                next_row[offset:offset + 7] = solutions[nearest]
            rows.append(next_row)
        dense = np.asarray(densify(np.asarray(rows)))
        self.last_extract_frames = dense
        extraction_validity = empty_validity if self.snapshot.policy.defer_payload_until_extract_end else loaded_validity
        valid = extraction_validity.mask(torch.tensor(dense, device="cuda", dtype=torch.float32))
        if not bool(valid.all().item()):
            return None, f"抽离机器人碰撞/稳定性失败@frame{int((~valid).nonzero()[0])+1}"
        if not bool(loaded_validity.mask(torch.tensor(dense[-1:], device="cuda", dtype=torch.float32)).item()):
            return None, "抽离终点携箱碰撞/稳定性失败"
        maximum_line_error = 0.0
        maximum_orientation_error = 0.0
        for values in dense:
            self.fk(values)
            for side in ("left", "right"):
                actual = self.fk_robot.get_transform(f"{side}_tool0", self.fk_robot.base_link)
                maximum_line_error = max(maximum_line_error,
                                         float(np.linalg.norm(actual[1:3, 3] - targets[side][1:3, 3])))
                maximum_orientation_error = max(maximum_orientation_error, float(Rotation.from_matrix(
                    actual[:3, :3].T @ targets[side][:3, :3]).magnitude()))
        if maximum_line_error > 0.002 or maximum_orientation_error > math.radians(1.0):
            return None, f"抽离插值偏离直线/朝向: {maximum_line_error*1000:.3f}mm"
        return dense, {"fixed_updown_m": float(contact[0]), "fixed_psi_rad": swivels,
                       "max_line_error_mm": maximum_line_error * 1000,
                       "max_orientation_error_deg": math.degrees(maximum_orientation_error)}

    def joint_plan(self, start, target, checker, validity, seed):
        lower, upper = checker.kinematics.get_joint_limits().position
        path, stats = self.search_path(
            torch.tensor(start, device="cuda", dtype=torch.float32),
            torch.tensor(np.asarray(target).reshape(1, -1), device="cuda", dtype=torch.float32),
            lower + 1e-5, upper - 1e-5, validity, 2.0, seed,
        )
        if path is None:
            return None, stats
        dense = np.asarray(densify(path))
        if not bool(validity.mask(torch.tensor(dense, device="cuda", dtype=torch.float32)).all().item()):
            return None, {"failure": "密化轨迹验收失败"}
        return dense, stats

    def _full_cycle(self, boxes, progress=lambda text: None):
        started = time.perf_counter()
        report = {"success": False, "task": boxes, "joint_names": list(
            self.mobile_robot["kinematics"]["cspace"]["joint_names"]),
            "frames": [], "phases": [], "payload": [], "attempts": []}
        report["planner"] = self.planner_kind
        timing = {}
        report["timing_ms"] = timing
        self.last_partial = report
        mobile_names = report["joint_names"]

        def append(rows, phase, payload):
            for row in rows:
                mapping = dict(zip(ACTIVE_JOINTS, row)) if len(row) == 15 else dict(zip(mobile_names, row))
                values = [float(mapping.get(name, 0.0)) for name in mobile_names]
                if report["frames"] and phase not in ("release", "attach") and np.allclose(
                        report["frames"][-1], values, atol=1e-10, rtol=0.0):
                    continue
                report["frames"].append(values)
                report["phases"].append(phase)
                report["payload"].append(payload)

        progress("检查初始状态并求32个接触IK")
        segment_started = time.perf_counter()
        empty_checker = self.checker(boxes)
        empty_validity = GpuValidity(empty_checker, check_ground=True,
                                     ground_z=self.snapshot.policy.ground_z_m)
        if not bool(empty_validity.mask(torch.tensor(self.home_values[None], device="cuda")).item()):
            raise CycleBlocked("初始姿态", "碰撞或越限", report)
        append([self.home_values], "home", False)
        torch.cuda.synchronize()
        timing["initial_checker_setup_and_validation"] = (time.perf_counter()-segment_started)*1000
        segment_started = time.perf_counter()
        solver = self.ik_solver(boxes)
        torch.cuda.synchronize()
        timing["contact_solver_setup"] = (time.perf_counter()-segment_started)*1000
        segment_started = time.perf_counter()
        contact_poses = self.contact_poses_for_task(boxes)
        result = solver.solve_pose(goal(contact_poses), JointState.from_position(
            torch.tensor(self.home_values[None], device="cuda"), joint_names=ACTIVE_JOINTS), return_seeds=32)
        names = list(result.js_solution.joint_names)
        values = result.js_solution.position.reshape(-1, len(names))
        successful = result.success.reshape(-1).bool()
        candidates = values[successful][:, [names.index(name) for name in ACTIVE_JOINTS]].cpu().numpy()
        if len(candidates) == 0:
            raise CycleBlocked("接触IK", "cuRobo没有返回成功候选", report)
        weights = np.array(joint_distance_weights(ACTIVE_JOINTS))
        candidates = candidates[np.argsort(np.sum(((candidates - self.home_values) * weights) ** 2, axis=1))]
        report["contact_ik_count"] = len(candidates)
        report["contact_candidates"] = candidates.tolist()
        fixed_contact = getattr(self, "comparison_contact", None)
        if fixed_contact is not None:
            candidates = np.asarray(fixed_contact).reshape(1, -1)
            report["comparison_contact_fixed"] = True
        torch.cuda.synchronize()
        timing["contact_ik_and_ranking"] = (time.perf_counter()-segment_started)*1000
        segment_started = time.perf_counter()
        loaded_checker = self.checker(boxes, payload=True)
        loaded_validity = GpuValidity(loaded_checker, active_sides=("left", "right"),
                                      max_box_tilt_deg=self.snapshot.policy.max_box_tilt_deg, check_ground=True,
                                      ground_z=self.snapshot.policy.ground_z_m)
        extracting_validity = GpuValidity(empty_checker, active_sides=("left", "right"),
                                          max_box_tilt_deg=self.snapshot.policy.max_box_tilt_deg, check_ground=True,
                                          ground_z=self.snapshot.policy.ground_z_m)
        torch.cuda.synchronize()
        timing["loaded_checker_setup"] = (time.perf_counter()-segment_started)*1000
        timing["analytic_extract_and_validation"] = 0.0
        timing["approach_rrt_and_validation"] = 0.0
        selected = None
        for index, contact in enumerate(candidates):
            progress(f"候选{index+1}/{len(candidates)}：固定Updown解析抽离")
            attempt_started = time.perf_counter()
            extracted, details = self.analytic_extract(contact, extracting_validity, loaded_validity)
            torch.cuda.synchronize()
            extraction_ms = (time.perf_counter()-attempt_started)*1000
            timing["analytic_extract_and_validation"] += extraction_ms
            attempt = {"candidate": index+1, "extraction": details, "extraction_ms": extraction_ms}
            report["attempts"].append(attempt)
            if extracted is None:
                attempt["wall_ms"] = (time.perf_counter()-attempt_started)*1000
                continue
            progress(f"候选{index+1}抽离成功：规划初始→接触")
            segment_started = time.perf_counter()
            approach, stats = self.plan_contact_approach(contact, empty_checker,
                                                       empty_validity, 20261003 + index)
            torch.cuda.synchronize()
            approach_ms = (time.perf_counter()-segment_started)*1000
            timing["approach_rrt_and_validation"] += approach_ms
            attempt["approach"] = stats
            attempt["approach_ms"] = approach_ms
            attempt["wall_ms"] = (time.perf_counter()-attempt_started)*1000
            if approach is not None:
                selected = (approach, extracted)
                report["selected_candidate"] = index+1
                break
        if selected is None:
            raise CycleBlocked("候选筛选", f"{len(candidates)}个候选均未通过抽离与到位", report)
        approach, extracted = selected
        split = getattr(self, 'contact_approach_split', len(approach))
        append(approach[:split], "approach", False)
        append(approach[split:], "contact_approach", False)
        append([approach[-1]], "attach", True)
        append(extracted, "extract", True)
        progress("规划抽离终点→放置目标并验证携箱转身")
        segment_started = time.perf_counter()
        transport = self.plan(boxes, extracted[-1])
        torch.cuda.synchronize()
        timing["transport_pipeline_total"] = (time.perf_counter()-segment_started)*1000
        report["transport_timing_breakdown_ms"] = transport.get("timing_ms", {})
        if not transport["success"]:
            report["transport_failure"] = transport["failure"]
            append(transport["frames"], "transport_diagnostic", True)
            failure = transport["failure"]
            raise CycleBlocked("搬运/携箱转身", f"{failure['stage']}@frame{failure['frame_index']+1}", report)
        append(transport["frames"][:transport["transport_frames"]], "transport", True)
        append(transport["frames"][transport["transport_frames"]:], "turn_loaded", True)
        report["transport_timing_ms"] = transport["total_ms"]
        report["transport_search_stats"] = transport["rrt_stats"]
        if hasattr(self, 'conveyor_suffix'):
            self.conveyor_suffix(boxes, report, append, progress)
            rows = np.asarray(report['frames'])
            report['final_home_error'] = float(np.max(np.abs(rows[-1, [mobile_names.index(name) for name in ACTIVE_JOINTS]] - self.home_values)))
            report['max_adjacent_joint_step_deg'] = float(np.degrees(np.abs(np.diff(rows[:, [mobile_names.index(name) for name in ACTIVE_JOINTS[1:]]], axis=0)).max()))
            report['total_ms'] = (time.perf_counter() - started) * 1000
            report['success'] = True
            return report
        append([report["frames"][-1]], "release", False)
        progress("检查箱体释放后原地转回")
        segment_started = time.perf_counter()
        endpoint = np.array(report["frames"][-1])
        yaw_index = mobile_names.index("base_yaw")
        turn_back = np.repeat(endpoint[None], 361, axis=0)
        turn_back[:, yaw_index] = np.linspace(math.pi, 0, 361)
        validate_turns=getattr(self,'validate_turns',True)
        if validate_turns:
            mobile_checker = self.checker(boxes, mobile=True)
            mobile_validity = GpuValidity(mobile_checker, check_ground=True, joint_names=mobile_names,
                                          ground_z=self.snapshot.policy.ground_z_m,
                                          weights=[2., 2., 1., 5.] + [1.] * 14)
        torch.cuda.synchronize()
        timing["empty_turn_checker_setup"] = (time.perf_counter()-segment_started)*1000
        segment_started = time.perf_counter()
        mask = mobile_validity.mask(torch.tensor(turn_back, device="cuda", dtype=torch.float32)) if validate_turns else torch.ones(len(turn_back),device='cuda',dtype=torch.bool)
        torch.cuda.synchronize()
        timing["empty_turn_validation"] = (time.perf_counter()-segment_started)*1000
        if not bool(mask.all().item()):
            raise CycleBlocked("空载转回", f"碰撞@{int((~mask).nonzero()[0])+1}", report)
        append(turn_back, "turn_empty", False)
        progress("规划双臂空载回到原始Home")
        segment_started = time.perf_counter()
        return_start = np.array([turn_back[-1, mobile_names.index(name)] for name in ACTIVE_JOINTS])
        home_path, stats = self.joint_plan(return_start, self.home_values, empty_checker,
                                            empty_validity, 20261004)
        torch.cuda.synchronize()
        timing["home_rrt_and_validation"] = (time.perf_counter()-segment_started)*1000
        if home_path is None:
            raise CycleBlocked("空载回Home", f"2秒内无路径或验收失败: {stats}", report)
        report["home_search_stats"] = stats
        append(home_path, "return_home", False)
        rows = np.asarray(report["frames"])
        report["max_adjacent_joint_step_deg"] = float(np.degrees(
            np.abs(np.diff(rows[:, 4:], axis=0)).max()))
        report["final_home_error"] = float(np.max(np.abs(
            rows[-1, [mobile_names.index(name) for name in ACTIVE_JOINTS]] - self.home_values)))
        report["total_ms"] = (time.perf_counter()-started)*1000
        timing["other_assembly_and_bookkeeping"] = report["total_ms"] - sum(timing.values())
        report["success"] = True
        return report
