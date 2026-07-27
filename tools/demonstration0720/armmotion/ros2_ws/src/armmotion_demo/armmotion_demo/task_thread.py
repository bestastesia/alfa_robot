from __future__ import annotations

import threading
import time
import uuid
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from .algorithm_thread import STATUS_QOS
from .common import STAGE_LABELS, decode_message, encode_message, parse_task_code


class TaskThread(Node):
    def __init__(self) -> None:
        super().__init__("armmotion_task_thread")
        self.task_publisher = self.create_publisher(String, "/armmotion/task_request", 10)
        self.stage_publisher = self.create_publisher(String, "/armmotion/next_stage", 10)
        self.initialize_publisher = self.create_publisher(String, "/armmotion/initialize", 10)
        self.create_subscription(String, "/armmotion/status", self._on_status, STATUS_QOS)
        self._condition = threading.Condition()
        self._events: list[dict[str, Any]] = []
        self.algorithm_ready = False

    def _on_status(self, message: String) -> None:
        try:
            event = decode_message(message.data)
        except Exception as exc:
            self.get_logger().warning(f"忽略非法状态消息: {exc}")
            return
        with self._condition:
            self._events.append(event)
            if len(self._events) > 200:
                del self._events[:-100]
            if event.get("event") == "ready":
                self.algorithm_ready = True
            self._condition.notify_all()

    def publish_task(self, request_id: str, task_code: str, front: float, top: float) -> None:
        self._wait_for_subscriber(self.task_publisher, "/armmotion/task_request")
        message = String()
        message.data = encode_message(
            "task_request",
            request_id=request_id,
            task_code=task_code,
            front_distance_m=front,
            top_distance_m=top,
        )
        self.task_publisher.publish(message)

    def publish_initialize(self, request_id: str) -> None:
        self._wait_for_subscriber(self.initialize_publisher, "/armmotion/initialize")
        message = String()
        message.data = encode_message("initialize", request_id=request_id)
        self.initialize_publisher.publish(message)

    def publish_stage(self, request_id: str, stage: int) -> None:
        self._wait_for_subscriber(self.stage_publisher, "/armmotion/next_stage")
        message = String()
        message.data = encode_message(
            "next_stage",
            request_id=request_id,
            stage=stage,
        )
        self.stage_publisher.publish(message)

    @staticmethod
    def _wait_for_subscriber(publisher, topic: str, timeout_s: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_s
        while publisher.get_subscription_count() < 1:
            if time.monotonic() >= deadline:
                raise TimeoutError(f"发布前未发现订阅者: {topic}")
            time.sleep(0.05)

    def wait_for_event(
        self,
        request_id: str,
        accepted_events: set[str],
        timeout_s: float,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                for index, event in enumerate(self._events):
                    if event.get("request_id") != request_id:
                        continue
                    if str(event.get("event")) in accepted_events:
                        return self._events.pop(index)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(
                        f"等待 {sorted(accepted_events)} 超时，request_id={request_id}"
                    )
                self._condition.wait(timeout=remaining)


def prompt_distance(label: str, current: float) -> float:
    while True:
        value = input(f"{label} baselink 到箱子前表面距离（m）[{current:.3f}]：").strip()
        if not value:
            return current
        try:
            candidate = float(value)
            parse_task_code("B1", candidate if "侧吸" in label else 0.9, candidate if "顶吸" in label else 0.7)
            return candidate
        except ValueError as exc:
            print(f"输入错误：{exc}")


def print_plan_summary(event: dict[str, Any]) -> None:
    print(
        "计算完成："
        f"墙钟 {float(event.get('wall_ms', 0.0)) / 1000.0:.3f}s，"
        f"算法 {float(event.get('total_ms', 0.0)) / 1000.0:.3f}s，"
        f"IK {float(event.get('ik_ms', 0.0)) / 1000.0:.3f}s，"
        f"抽离 {float(event.get('extract_ms', 0.0)) / 1000.0:.3f}s，"
        f"负重 {float(event.get('loaded_ms', 0.0)) / 1000.0:.3f}s，"
        f"收尾 {float(event.get('final_ms', 0.0)) / 1000.0:.3f}s"
    )
    print(f"快照：{event.get('snapshot', '')}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskThread()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True, name="task-thread-ros-spin")
    spin_thread.start()
    try:
        print("等待算法线程 /armmotion/status ...", flush=True)
        deadline = time.monotonic() + 30.0
        while not node.algorithm_ready and time.monotonic() < deadline:
            time.sleep(0.05)
        if not node.algorithm_ready:
            raise RuntimeError("30秒内未发现算法线程，请先启动 run_algorithm_thread.sh")

        input("算法线程已就绪。确认人员和设备安全后，直接回车让机器人运动到负重初始位：")
        initialize_request_id = f"initialize-{int(time.time())}-{uuid.uuid4().hex[:6]}"
        node.publish_initialize(initialize_request_id)
        initialization_event = node.wait_for_event(
            initialize_request_id,
            {"initialization_complete", "initialization_failed"},
            180.0,
        )
        if initialization_event.get("event") != "initialization_complete":
            raise RuntimeError(
                f"初始化负重位失败：{initialization_event.get('reason', initialization_event)}"
            )
        print(
            "初始化负重位完成："
            f"实际 {float(initialization_event.get('wall_ms', 0.0)) / 1000.0:.3f}s，"
            f"规划时长 {float(initialization_event.get('planned_duration_s', 0.0)):.3f}s。"
        )

        front_distance_m = prompt_distance("侧吸", 0.90)
        top_distance_m = prompt_distance("顶吸", 0.70)
        print(
            f"距离已确认：侧吸={front_distance_m:.3f}m，顶吸={top_distance_m:.3f}m。\n"
            "A=横向偏移5cm，B=居中；1..5=从上到下五组。\n"
            "输入 A1..A5/B1..B5 开始计算；输入 D 修改距离；输入 Q 退出。"
        )

        while rclpy.ok():
            command = input("\n任务编号：").strip().upper()
            if command == "Q":
                break
            if command == "D":
                front_distance_m = prompt_distance("侧吸", front_distance_m)
                top_distance_m = prompt_distance("顶吸", top_distance_m)
                print(
                    f"距离已更新：侧吸={front_distance_m:.3f}m，顶吸={top_distance_m:.3f}m"
                )
                continue
            try:
                task = parse_task_code(command, front_distance_m, top_distance_m)
            except ValueError as exc:
                print(f"输入错误：{exc}")
                continue
            request_id = f"{task.code}-{int(time.time())}-{uuid.uuid4().hex[:6]}"
            print(
                f"发送 {task.code}: L{task.left_box_id}/R{task.right_box_id}，"
                f"布局={task.layout}，吸附={task.grasp_family}，"
                f"有效距离={task.effective_distance_m:.3f}m"
            )
            node.publish_task(
                request_id,
                task.code,
                front_distance_m,
                top_distance_m,
            )
            plan_event = node.wait_for_event(
                request_id,
                {"planning_complete", "failed", "request_rejected"},
                240.0,
            )
            if plan_event.get("event") != "planning_complete":
                print(f"任务未进入执行：{plan_event.get('reason', plan_event)}")
                continue
            print_plan_summary(plan_event)

            task_failed = False
            for stage in range(1, 8):
                input(f"第{stage}/7步：{STAGE_LABELS[stage]}。直接回车执行：")
                node.publish_stage(request_id, stage)
                stage_event = node.wait_for_event(
                    request_id,
                    {"stage_complete", "failed", "stage_rejected"},
                    300.0,
                )
                if stage_event.get("event") != "stage_complete":
                    print(
                        f"第{stage}步失败，任务停止："
                        f"{stage_event.get('reason', stage_event)}"
                    )
                    task_failed = True
                    break
                print(
                    f"第{stage}步完成：实际 {float(stage_event.get('wall_ms', 0.0)) / 1000.0:.3f}s，"
                    f"规划时长 {float(stage_event.get('planned_duration_s', 0.0)):.3f}s"
                )
            if not task_failed:
                print(f"{task.code} 七步执行完成，可以发送下一任务。")
    except KeyboardInterrupt:
        print("\n任务线程退出。")
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
