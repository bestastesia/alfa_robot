#!/usr/bin/python3
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node


class FakePlcCliTaskPublisher(Node):
    def __init__(self) -> None:
        super().__init__('fake_plc_cli_task_publisher')
        self.driver_dir = Path(str(self.declare_parameter(
            'driver_dir', '/mnt/mydisk/ALFA/alfa_robot_ec/tools/alfa_robot_plc_driver').value))
        self.home_targets = str(self.declare_parameter(
            'home_targets', '1:0,2:0,3:0,4:0,5:0,6:0,7:0,8:0,9:0,10:0,11:0,12:0').value)
        self.target1_targets = str(self.declare_parameter(
            'target1_targets', '1:-28,2:51,3:38,4:30,5:81,6:83,7:28,8:-49,9:-35,10:-28,11:-79,12:-85').value)
        self.target2_targets = str(self.declare_parameter(
            'target2_targets', '1:16,2:15,3:-119,4:-160,5:46,6:-104,7:33,8:-7,9:111,10:56,11:46,12:40').value)
        self.home_velocity = float(self.declare_parameter('home_velocity', 15.0).value)
        self.target1_velocity = float(self.declare_parameter('target1_velocity', 60.0).value)
        self.target2_velocity = float(self.declare_parameter('target2_velocity', 15.0).value)
        legacy_velocity = float(self.declare_parameter('velocity', 0.0).value)  # legacy, ignored
        if abs(legacy_velocity) > 1e-9:
            self.get_logger().warn('Parameter velocity is ignored now; use home_velocity/target1_velocity/target2_velocity')
        self.python = str(self.declare_parameter('python', sys.executable).value)
        self.sequence = [
            ('TARGET1', self.target1_targets, self.target1_velocity),
            ('HOME', self.home_targets, self.home_velocity),
            ('TARGET2', self.target2_targets, self.target2_velocity),
            ('HOME', self.home_targets, self.home_velocity),
        ]
        self.sequence_index = 0
        self.command_index = 0
        self.get_logger().warn('FAKE PLC CLI sequence publisher ready. Startup sends HOME once; Enter cycles TARGET1 -> HOME -> TARGET2 -> HOME.')
        self.get_logger().info(f'HOME    vel={self.home_velocity:g}: {self.home_targets}')
        self.get_logger().info(f'TARGET1 vel={self.target1_velocity:g}: {self.target1_targets}')
        self.get_logger().info(f'TARGET2 vel={self.target2_velocity:g}: {self.target2_targets}')

    def run_targets(self, label: str, targets: str, velocity: float) -> int:
        cmd = [
            self.python,
            '-m', 'alfa_robot_plc_driver.cli',
            'move-abs',
            '--targets', targets,
            '--vel', f'{velocity:g}',
            '--yes-write',
        ]
        self.command_index += 1
        self.get_logger().warn(f'[{self.command_index}] Executing PLC CLI move-abs -> {label} vel={velocity:g}')
        self.get_logger().info(' '.join(cmd))
        result = subprocess.run(cmd, cwd=str(self.driver_dir), text=True)
        if result.returncode == 0:
            self.get_logger().info(f'[{self.command_index}] PLC CLI move-abs finished: {label}')
        else:
            self.get_logger().error(f'[{self.command_index}] PLC CLI move-abs failed with code {result.returncode}: {label}')
        return result.returncode

    def run_startup_home(self) -> int:
        return self.run_targets('HOME(startup)', self.home_targets, self.home_velocity)

    def run_next(self) -> int:
        label, targets, velocity = self.sequence[self.sequence_index]
        code = self.run_targets(label, targets, velocity)
        if code == 0:
            self.sequence_index = (self.sequence_index + 1) % len(self.sequence)
        return code


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FakePlcCliTaskPublisher()
    code = 0
    try:
        code = node.run_startup_home()
        while rclpy.ok():
            try:
                line = input()
            except EOFError:
                node.get_logger().warn('stdin closed; exiting fake PLC CLI publisher')
                break
            if line.strip().lower() in {'q', 'quit', 'exit'}:
                node.get_logger().info('exit requested')
                break
            code = node.run_next()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == '__main__':
    main()
