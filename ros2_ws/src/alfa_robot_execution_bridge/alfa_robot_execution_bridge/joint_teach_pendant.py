"""Interactive 14-axis teaching pendant with live Rerun preview.

Slider changes only update the preview robot in Rerun. A complete, smooth
14-axis trajectory is sent to rt-control only after the operator explicitly
unlocks execution and presses the execute button.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from alfa_robot_execution_bridge.joints import (
    ARM_JOINT_POSITION_LIMITS_RAD,
    RT_CONTROL_ACTION_NAME,
    RT_CONTROL_JOINT_NAMES,
    UPDOWN_LOGICAL_LOWER_M,
    UPDOWN_LOGICAL_UPPER_M,
    model_to_rt_control_acceleration,
    model_to_rt_control_position,
    model_to_rt_control_velocity,
    rt_control_to_model_position,
)


TF_ROOT_FRAME = "base_footprint"
ACTIVE_GOAL_STATUSES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}
SMOOTHSTEP_MAX_VELOCITY_GAIN = 1.875
SMOOTHSTEP_MAX_ACCELERATION_GAIN = 10.0 / math.sqrt(3.0)


@dataclass(frozen=True)
class JointSpec:
    name: str
    title: str
    lower: float
    upper: float
    resolution: float
    unit: str
    rotary: bool = True

    def to_display(self, model_value: float) -> float:
        return math.degrees(model_value) if self.rotary else model_value

    def to_model(self, display_value: float) -> float:
        return math.radians(display_value) if self.rotary else display_value


def _arm_specs(side: str, title: str) -> list[JointSpec]:
    return [
        JointSpec(
            name=f"{side}_joint{index}",
            title=f"{title} J{index}",
            lower=math.degrees(ARM_JOINT_POSITION_LIMITS_RAD[f"{side}_joint{index}"][0]),
            upper=math.degrees(ARM_JOINT_POSITION_LIMITS_RAD[f"{side}_joint{index}"][1]),
            resolution=0.1,
            unit="°",
        )
        for index in range(1, 7)
    ]


JOINT_SPECS = [
    *_arm_specs("left", "左臂"),
    *_arm_specs("right", "右臂"),
    JointSpec("turn", "Turn", -180.0, 180.0, 0.1, "°"),
    JointSpec(
        "updown",
        "Updown",
        UPDOWN_LOGICAL_LOWER_M,
        UPDOWN_LOGICAL_UPPER_M,
        0.001,
        "m",
        rotary=False,
    ),
]
JOINT_SPEC_BY_NAME = {spec.name: spec for spec in JOINT_SPECS}
REQUIRED_TF_LINKS = (
    "base_link",
    "pitch",
    "turn",
    "updown",
    *(f"left_joint{index}" for index in range(1, 7)),
    *(f"right_joint{index}" for index in range(1, 7)),
)


def quintic_smoothstep(phase: float) -> tuple[float, float, float]:
    phase = min(1.0, max(0.0, float(phase)))
    position = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
    velocity = 30.0 * phase**2 - 60.0 * phase**3 + 30.0 * phase**4
    acceleration = 60.0 * phase - 180.0 * phase**2 + 120.0 * phase**3
    return position, velocity, acceleration


def seconds_to_duration(seconds: float):
    point = JointTrajectoryPoint()
    seconds = max(0.0, float(seconds))
    whole = math.floor(seconds)
    point.time_from_start.sec = int(whole)
    point.time_from_start.nanosec = int(round((seconds - whole) * 1e9))
    if point.time_from_start.nanosec >= 1_000_000_000:
        point.time_from_start.sec += 1
        point.time_from_start.nanosec -= 1_000_000_000
    return point.time_from_start


def trajectory_duration_s(
    current: dict[str, float],
    target: dict[str, float],
    *,
    minimum_s: float,
    rotary_velocity_rad_s: float,
    rotary_acceleration_rad_s2: float,
    updown_velocity_m_s: float,
    updown_acceleration_m_s2: float,
) -> float:
    duration = max(0.1, float(minimum_s))
    for name in RT_CONTROL_JOINT_NAMES:
        delta = abs(float(target[name]) - float(current[name]))
        if delta <= 1e-12:
            continue
        if name == "updown":
            velocity_limit = updown_velocity_m_s
            acceleration_limit = updown_acceleration_m_s2
        else:
            velocity_limit = rotary_velocity_rad_s
            acceleration_limit = rotary_acceleration_rad_s2
        duration = max(
            duration,
            delta * SMOOTHSTEP_MAX_VELOCITY_GAIN / velocity_limit,
            math.sqrt(delta * SMOOTHSTEP_MAX_ACCELERATION_GAIN / acceleration_limit),
        )
    return duration


def build_smoothstep_trajectory(
    current: dict[str, float],
    target: dict[str, float],
    *,
    hz: float,
    minimum_s: float,
    rotary_velocity_rad_s: float,
    rotary_acceleration_rad_s2: float,
    updown_velocity_m_s: float,
    updown_acceleration_m_s2: float,
) -> tuple[JointTrajectory, float]:
    duration = trajectory_duration_s(
        current,
        target,
        minimum_s=minimum_s,
        rotary_velocity_rad_s=rotary_velocity_rad_s,
        rotary_acceleration_rad_s2=rotary_acceleration_rad_s2,
        updown_velocity_m_s=updown_velocity_m_s,
        updown_acceleration_m_s2=updown_acceleration_m_s2,
    )
    steps = max(1, int(math.ceil(duration * hz)))
    trajectory = JointTrajectory()
    trajectory.joint_names = list(RT_CONTROL_JOINT_NAMES)
    for step in range(steps + 1):
        phase = step / steps
        position_scale, velocity_scale, acceleration_scale = quintic_smoothstep(phase)
        point = JointTrajectoryPoint()
        point.time_from_start = seconds_to_duration(duration * phase)
        for name in RT_CONTROL_JOINT_NAMES:
            start = float(current[name])
            delta = float(target[name]) - start
            point.positions.append(
                model_to_rt_control_position(name, start + delta * position_scale)
            )
            point.velocities.append(
                model_to_rt_control_velocity(name, delta * velocity_scale / duration)
            )
            point.accelerations.append(
                model_to_rt_control_acceleration(
                    name, delta * acceleration_scale / (duration * duration)
                )
            )
        trajectory.points.append(point)
    return trajectory, duration


class RerunTeachView:
    ACTUAL_PATH = "world/current_robot"
    PREVIEW_PATH = "world/slider_preview"

    def __init__(self, preview_offset_y_m: float, spawn: bool) -> None:
        import rerun as rr
        from alfa_robot_rerun import visualize_rerun as rerun_helpers

        self.rr = rr
        self.helpers = rerun_helpers
        self.robot = rerun_helpers.UrdfRobot(rerun_helpers.render_current_urdf())
        rr.init("alfa_joint_teach_pendant", spawn=spawn)
        rerun_helpers.log_robot_static_model(self.robot, self.ACTUAL_PATH, log_meshes=True)
        rerun_helpers.log_robot_static_model(self.robot, self.PREVIEW_PATH, log_meshes=True)
        rr.log(
            self.PREVIEW_PATH,
            rr.Transform3D(translation=[0.0, float(preview_offset_y_m), 0.0]),
            static=True,
        )
        rr.log(
            f"{self.ACTUAL_PATH}/identity",
            rr.Points3D([[0.0, 0.0, 0.0]], colors=[[40, 220, 100]], labels=["当前状态"]),
            static=True,
        )
        rr.log(
            f"{self.PREVIEW_PATH}/identity",
            rr.Points3D([[0.0, 0.0, 0.0]], colors=[[255, 170, 30]], labels=["滑块预览"]),
            static=True,
        )
        self.sample = 0

    def log(self, actual_transforms: dict[str, object], preview: dict[str, float]) -> None:
        self.helpers.set_sample_time(self.sample)
        for link_name, transform_stamped in actual_transforms.items():
            transform = transform_stamped.transform
            self.rr.log(
                f"{self.ACTUAL_PATH}/{link_name}",
                self.rr.Transform3D(
                    translation=[
                        float(transform.translation.x),
                        float(transform.translation.y),
                        float(transform.translation.z),
                    ],
                    rotation=self.rr.Quaternion(
                        xyzw=[
                            float(transform.rotation.x),
                            float(transform.rotation.y),
                            float(transform.rotation.z),
                            float(transform.rotation.w),
                        ]
                    ),
                ),
            )
        self.helpers.log_robot_state(self.robot, preview, self.PREVIEW_PATH)
        self.sample += 1


class JointTeachPendant(Node):
    def __init__(self, args: argparse.Namespace, root: tk.Tk) -> None:
        super().__init__("alfa_joint_teach_pendant")
        self.args = args
        self.root = root
        self.action_client = ActionClient(self, FollowJointTrajectory, args.action_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.actual: dict[str, float] = {}
        self.actual_tf: dict[str, object] = {}
        self.preview: dict[str, float] = {}
        self.target_initialized = False
        self.last_joint_state_at = 0.0
        self.controller_statuses: list[int] = []
        self.goal_handle = None
        self.result_future = None
        self.execution_active = False
        self.last_rerun_at = 0.0
        self.last_interlock_at = 0.0
        self.slider_vars: dict[str, tk.DoubleVar] = {}
        self.actual_labels: dict[str, ttk.Label] = {}
        self.target_labels: dict[str, ttk.Label] = {}
        self.unlock_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="等待完整 /joint_states …")
        self.tf_status_var = tk.StringVar(value=f"TF: 等待 {args.tf_root_frame} 本体树…")
        self.interlock_var = tk.StringVar(value="正在检查控制权…")
        self.rerun = RerunTeachView(args.preview_offset_y_m, not args.no_spawn_rerun)

        self.create_subscription(
            JointState,
            args.joint_state_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            GoalStatusArray,
            f"{args.action_name}/_action/status",
            self._on_controller_status,
            10,
        )
        self._build_ui()

    def _build_ui(self) -> None:
        self.root.title("ALFA 14轴示教器（Rerun预览）")
        self.root.geometry("1180x820")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        warning = ttk.Label(
            self.root,
            text=(
                "滑块只改变 Rerun 预览，不会驱动实体机器人。执行轨迹不做碰撞规划；"
                "必须确认现场安全；控制器已有活动轨迹时禁止发送。"
            ),
            foreground="#b00020",
            wraplength=1120,
        )
        warning.pack(fill="x", padx=12, pady=(10, 4))

        state_frame = ttk.Frame(self.root)
        state_frame.pack(fill="x", padx=12, pady=4)
        ttk.Label(state_frame, textvariable=self.status_var).pack(side="left")
        ttk.Label(state_frame, textvariable=self.tf_status_var).pack(side="left", padx=24)
        ttk.Label(state_frame, textvariable=self.interlock_var).pack(side="right")

        body = ttk.Frame(self.root)
        body.pack(fill="both", expand=True, padx=12, pady=4)
        left_frame = ttk.LabelFrame(body, text="左臂")
        right_frame = ttk.LabelFrame(body, text="右臂")
        left_frame.pack(side="left", fill="both", expand=True, padx=(0, 6))
        right_frame.pack(side="left", fill="both", expand=True, padx=(6, 0))
        for spec in JOINT_SPECS[:6]:
            self._add_slider(left_frame, spec)
        for spec in JOINT_SPECS[6:12]:
            self._add_slider(right_frame, spec)

        base_frame = ttk.LabelFrame(self.root, text="公共轴")
        base_frame.pack(fill="x", padx=12, pady=4)
        for spec in JOINT_SPECS[12:]:
            self._add_slider(base_frame, spec)

        controls = ttk.Frame(self.root)
        controls.pack(fill="x", padx=12, pady=(6, 12))
        ttk.Button(controls, text="重新载入当前状态", command=self.reset_preview).pack(
            side="left", padx=(0, 8)
        )
        ttk.Checkbutton(
            controls,
            text="我已确认人员、急停和运动空间安全",
            variable=self.unlock_var,
        ).pack(side="left", padx=8)
        self.execute_button = ttk.Button(
            controls, text="执行滑块目标", command=self.execute_preview
        )
        self.execute_button.pack(side="right", padx=(8, 0))
        self.cancel_button = ttk.Button(
            controls, text="取消当前运动", command=self.cancel_motion, state="disabled"
        )
        self.cancel_button.pack(side="right")

    def _add_slider(self, parent: ttk.Widget, spec: JointSpec) -> None:
        frame = ttk.Frame(parent)
        frame.pack(fill="x", padx=8, pady=3)
        ttk.Label(frame, text=spec.title, width=10).pack(side="left")
        variable = tk.DoubleVar(value=0.0)
        self.slider_vars[spec.name] = variable
        scale = tk.Scale(
            frame,
            from_=spec.lower,
            to=spec.upper,
            resolution=spec.resolution,
            orient="horizontal",
            showvalue=False,
            variable=variable,
            command=lambda value, name=spec.name: self._on_slider(name, value),
            length=340,
        )
        scale.pack(side="left", fill="x", expand=True)
        target_label = ttk.Label(frame, text=f"0.0{spec.unit}", width=12)
        actual_label = ttk.Label(frame, text=f"当前 --{spec.unit}", width=17)
        target_label.pack(side="left", padx=(6, 0))
        actual_label.pack(side="left", padx=(6, 0))
        self.target_labels[spec.name] = target_label
        self.actual_labels[spec.name] = actual_label

    def _on_joint_state(self, message: JointState) -> None:
        values: dict[str, float] = {}
        for name, position in zip(message.name, message.position):
            if name in JOINT_SPEC_BY_NAME:
                values[name] = rt_control_to_model_position(name, float(position))
        if not all(name in values for name in RT_CONTROL_JOINT_NAMES):
            return
        self.actual = values
        self.last_joint_state_at = time.monotonic()
        for name, value in values.items():
            spec = JOINT_SPEC_BY_NAME[name]
            self.actual_labels[name].configure(
                text=f"当前 {spec.to_display(value):.3f}{spec.unit}"
            )
        if not self.target_initialized:
            self._set_preview(values)
            self.target_initialized = True
            self.status_var.set("已收到14轴状态；滑块仅预览，点击按钮才执行")

    def _on_controller_status(self, message: GoalStatusArray) -> None:
        self.controller_statuses = [int(item.status) for item in message.status_list]

    def _on_slider(self, name: str, value: str) -> None:
        if not self.target_initialized:
            return
        spec = JOINT_SPEC_BY_NAME[name]
        display_value = float(value)
        self.preview[name] = spec.to_model(display_value)
        self.target_labels[name].configure(text=f"{display_value:.3f}{spec.unit}")

    def _set_preview(self, values: dict[str, float]) -> None:
        self.preview = dict(values)
        for name, value in values.items():
            spec = JOINT_SPEC_BY_NAME[name]
            display_value = spec.to_display(value)
            self.slider_vars[name].set(display_value)
            self.target_labels[name].configure(text=f"{display_value:.3f}{spec.unit}")

    def reset_preview(self) -> None:
        if not self.actual:
            messagebox.showwarning("尚无状态", "还没有收到完整14轴 /joint_states。")
            return
        self._set_preview(self.actual)
        self.status_var.set("滑块目标已重置为当前实体状态")

    def _controller_busy(self) -> bool:
        return any(status in ACTIVE_GOAL_STATUSES for status in self.controller_statuses)

    def _read_rt_control_tf(self) -> None:
        transforms: dict[str, object] = {}
        missing_required: list[str] = []
        for link_name in self.rerun.robot.links:
            if link_name == "world":
                continue
            try:
                transforms[link_name] = self.tf_buffer.lookup_transform(
                    self.args.tf_root_frame,
                    link_name,
                    Time(),
                )
            except TransformException:
                if link_name in REQUIRED_TF_LINKS:
                    missing_required.append(link_name)
        self.actual_tf = transforms
        required_count = len(REQUIRED_TF_LINKS) - len(missing_required)
        if missing_required:
            preview = ",".join(missing_required[:3])
            suffix = "…" if len(missing_required) > 3 else ""
            self.tf_status_var.set(
                f"TF: {required_count}/{len(REQUIRED_TF_LINKS)}，缺少 {preview}{suffix}"
            )
        else:
            self.tf_status_var.set(
                f"TF: rt-control {self.args.tf_root_frame} 树完整 "
                f"({len(transforms)}帧)"
            )

    def _refresh_interlock(self) -> None:
        if self._controller_busy() and not self.execution_active:
            self.interlock_var.set("执行锁定：rt-control 正在执行其他轨迹")
        else:
            self.interlock_var.set("手动执行可用：控制器当前空闲")

    def _execution_problem(self) -> str | None:
        if not self.target_initialized or not self.actual:
            return "尚未收到完整14轴 /joint_states"
        if time.monotonic() - self.last_joint_state_at > self.args.joint_state_timeout_s:
            return "最新 /joint_states 已超时"
        if not all(link_name in self.actual_tf for link_name in REQUIRED_TF_LINKS):
            return f"rt-control TF 树不完整，缺少 {self.args.tf_root_frame} 到机械本体的变换"
        if self._controller_busy() and not self.execution_active:
            return "rt-control 正在执行其他轨迹"
        if not self.unlock_var.get():
            return "请先勾选现场安全确认"
        return None

    def execute_preview(self) -> None:
        problem = self._execution_problem()
        if problem is not None:
            messagebox.showerror("禁止执行", problem)
            return
        if self.execution_active:
            return
        if not self.action_client.wait_for_server(timeout_sec=0.2):
            messagebox.showerror("控制器不可用", f"Action 不可用：{self.args.action_name}")
            return
        trajectory, duration = build_smoothstep_trajectory(
            self.actual,
            self.preview,
            hz=self.args.trajectory_hz,
            minimum_s=self.args.minimum_duration_s,
            rotary_velocity_rad_s=math.radians(self.args.max_velocity_deg_s),
            rotary_acceleration_rad_s2=math.radians(self.args.max_acceleration_deg_s2),
            updown_velocity_m_s=self.args.max_updown_velocity_m_s,
            updown_acceleration_m_s2=self.args.max_updown_acceleration_m_s2,
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self.execution_active = True
        self.execute_button.configure(state="disabled")
        self.cancel_button.configure(state="normal")
        self.status_var.set(
            f"正在发送轨迹：{len(trajectory.points)}点，预计 {duration:.2f}s"
        )
        future = self.action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self._finish_execution(f"发送失败：{exc}", error=True)
            return
        if self.goal_handle is None or not self.goal_handle.accepted:
            self._finish_execution("控制器拒绝了轨迹", error=True)
            return
        self.status_var.set("控制器已接受轨迹，机器人正在运动")
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self._on_result)

    def _on_feedback(self, _feedback) -> None:
        self.status_var.set("机器人正在执行滑块目标；可点击取消")

    def _on_result(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                self._finish_execution("轨迹执行完成")
            else:
                self._finish_execution(
                    f"轨迹执行失败：code={result.error_code} {result.error_string}",
                    error=True,
                )
        except Exception as exc:
            self._finish_execution(f"读取执行结果失败：{exc}", error=True)

    def cancel_motion(self) -> None:
        if self.goal_handle is None or not self.execution_active:
            return
        self.status_var.set("正在请求取消当前轨迹…")
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(lambda _future: self.status_var.set("取消请求已发送"))

    def _finish_execution(self, text: str, *, error: bool = False) -> None:
        self.execution_active = False
        self.goal_handle = None
        self.result_future = None
        self.execute_button.configure(state="normal")
        self.cancel_button.configure(state="disabled")
        self.unlock_var.set(False)
        self.status_var.set(text)
        if error:
            messagebox.showerror("执行结果", text)

    def poll(self) -> None:
        if not rclpy.ok():
            return
        for _ in range(8):
            rclpy.spin_once(self, timeout_sec=0.0)
        now = time.monotonic()
        if now - self.last_rerun_at >= 1.0 / 15.0:
            self._read_rt_control_tf()
            if self.preview:
                self.rerun.log(self.actual_tf, self.preview)
            self.last_rerun_at = now
        if now - self.last_interlock_at >= 0.5:
            self._refresh_interlock()
            self.last_interlock_at = now
        self.root.after(20, self.poll)

    def _on_close(self) -> None:
        if self.execution_active:
            messagebox.showwarning("运动尚未结束", "请先取消当前运动并等待控制器返回。")
            return
        self.root.quit()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ALFA 14轴滑块示教器与Rerun实时预览")
    parser.add_argument("--action-name", default=RT_CONTROL_ACTION_NAME)
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--tf-root-frame", default=TF_ROOT_FRAME)
    parser.add_argument("--trajectory-hz", type=float, default=30.0)
    parser.add_argument("--minimum-duration-s", type=float, default=1.0)
    parser.add_argument("--max-velocity-deg-s", type=float, default=10.0)
    parser.add_argument("--max-acceleration-deg-s2", type=float, default=10.0)
    parser.add_argument("--max-updown-velocity-m-s", type=float, default=0.05)
    parser.add_argument("--max-updown-acceleration-m-s2", type=float, default=0.05)
    parser.add_argument("--joint-state-timeout-s", type=float, default=1.0)
    parser.add_argument("--preview-offset-y-m", type=float, default=2.0)
    parser.add_argument("--no-spawn-rerun", action="store_true")
    args = parser.parse_args(remove_ros_args(argv)[1:])
    for name in (
        "trajectory_hz",
        "minimum_duration_s",
        "max_velocity_deg_s",
        "max_acceleration_deg_s2",
        "max_updown_velocity_m_s",
        "max_updown_acceleration_m_s2",
        "joint_state_timeout_s",
    ):
        if float(getattr(args, name)) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} 必须大于0")
    return args


def main() -> int:
    args = parse_args(sys.argv)
    rclpy.init(args=sys.argv)
    root = tk.Tk()
    node = JointTeachPendant(args, root)
    request_exit = lambda *_args: root.after_idle(root.quit)
    signal.signal(signal.SIGINT, request_exit)
    signal.signal(signal.SIGTERM, request_exit)
    try:
        root.after(20, node.poll)
        root.mainloop()
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        root.destroy()


if __name__ == "__main__":
    raise SystemExit(main())
