#!/usr/bin/env python3
"""
RMD 电机 PID 整定上位机

集成化 GUI 工具，用于调整瓴控科技 RMD 电机内部三环 PID 参数。
功能: PID 读写、电机控制、实时波形、阶跃测试、调试记录。

依赖: pip install PyQt5 matplotlib numpy

用法:
  python3 rmd_tuner.py
"""

import csv
import os
import socket
import struct
import sys
import time
from collections import deque
from datetime import datetime
from typing import Optional

import matplotlib
matplotlib.use("Qt5Agg")

import matplotlib.font_manager as fm
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import numpy as np

from PyQt5.QtCore import (
    QObject, QThread, pyqtSignal, pyqtSlot, QMutex, QMutexLocker,
    QTimer, Qt,
)
from PyQt5.QtGui import QFont, QFontDatabase, QColor
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QComboBox, QSpinBox,
    QTableWidget, QTableWidgetItem, QCheckBox, QRadioButton,
    QButtonGroup, QSplitter, QMessageBox, QShortcut, QFileDialog,
    QHeaderView, QDoubleSpinBox,
)


# ============================================================
# 字体配置
# ============================================================

def _setup_matplotlib_fonts():
    """配置 matplotlib 中文字体 (可在 QApplication 之前调用)"""
    zh_font = None
    for candidate in ["SimHei", "Droid Sans Fallback", "Noto Sans CJK SC",
                       "WenQuanYi Micro Hei"]:
        if any(f.name == candidate for f in fm.fontManager.ttflist):
            zh_font = candidate
            break

    matplotlib.rcParams["font.sans-serif"] = (
        [zh_font, "DejaVu Sans"] if zh_font else ["DejaVu Sans"]
    )
    matplotlib.rcParams["axes.unicode_minus"] = False


def _setup_qt_fonts():
    """加载 Qt 字体 (必须在 QApplication 创建之后调用)"""
    font_path = os.path.expanduser("~/.local/share/fonts/黑体.ttf")
    if os.path.isfile(font_path):
        QFontDatabase.addApplicationFont(font_path)


# ============================================================
# SocketCAN 底层
# ============================================================

CAN_RAW = 1
CAN_FORMAT = "<IB3x8s"  # can_id(4) + dlc(1) + pad(3) + data(8) = 16 bytes


def create_can_socket(interface: str, timeout: float = 0.5) -> socket.socket:
    """创建并绑定 SocketCAN 原始套接字"""
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
    sock.settimeout(timeout)
    sock.bind((interface,))
    return sock


def send_frame(sock: socket.socket, can_id: int, data: bytes) -> None:
    """发送一帧 CAN 数据"""
    padded = data.ljust(8, b"\x00")
    frame = struct.pack(CAN_FORMAT, can_id, 8, padded)
    sock.send(frame)


def recv_frame(sock: socket.socket) -> tuple:
    """接收一帧, 返回 (can_id, data_bytes)"""
    raw = sock.recv(16)
    if len(raw) != 16:
        raise OSError(f"Short CAN frame: {len(raw)} bytes")
    can_id, dlc, data = struct.unpack(CAN_FORMAT, raw)
    can_id &= 0x7FF
    return can_id, data[:dlc]


def _drain_rx(sock: socket.socket):
    """排空 socket 接收缓冲区中的残留帧"""
    while True:
        try:
            sock.recv(16)
        except (socket.timeout, OSError):
            break


def send_and_recv(
    sock: socket.socket,
    motor_id: int,
    data: bytes,
    expected_cmd: Optional[int] = None,
    retries: int = 10,
    drain_first: bool = False,
) -> Optional[bytes]:
    """发送命令并等待对应电机回复 (rx_id = 0x140 + motor_id)"""
    tx_id = 0x140 + motor_id
    rx_id = 0x140 + motor_id

    if drain_first:
        _drain_rx(sock)

    send_frame(sock, tx_id, data)

    for _ in range(retries):
        try:
            rid, rdata = recv_frame(sock)
            if rid != rx_id:
                continue
            if expected_cmd is None or rdata[0] == expected_cmd:
                return rdata
        except (socket.timeout, OSError, struct.error):
            continue

    return None


# ============================================================
# 协议命令封装
# ============================================================

PARAM_ID_ANGLE_PID = 0x0A
PARAM_ID_SPEED_PID = 0x0B
PARAM_ID_CURRENT_PID = 0x0C
PARAM_ID_TORQUE_LIMIT = 0x1E
PARAM_ID_SPEED_LIMIT = 0x20

LOOP_LABEL = {
    PARAM_ID_ANGLE_PID: "角度环",
    PARAM_ID_SPEED_PID: "速度环",
    PARAM_ID_CURRENT_PID: "电流环",
}

ERROR_BITS = {
    0: "低电压保护",
    1: "高电压保护",
    2: "驱动过温",
    3: "电机过温",
    4: "电机过流",
    5: "电机短路",
    6: "电机堵转",
    7: "输入信号丢失超时",
}


def read_control_param(sock: socket.socket, motor_id: int, param_id: int) -> Optional[bytes]:
    """0xC0 读取控制参数, 返回 DATA[2..7] 共 6 字节"""
    cmd = bytes([0xC0, param_id, 0, 0, 0, 0, 0, 0])
    resp = send_and_recv(sock, motor_id, cmd, expected_cmd=0xC0, drain_first=True)
    if resp is None or resp[1] != param_id:
        return None
    return resp[2:8]


def write_control_param(
    sock: socket.socket, motor_id: int, param_id: int, param_bytes: bytes,
) -> Optional[bytes]:
    """0xC1 写入控制参数到 RAM"""
    payload = bytes([0xC1, param_id]) + param_bytes[:6].ljust(6, b"\x00")
    return send_and_recv(sock, motor_id, payload, expected_cmd=0xC1, drain_first=True)


def parse_pid(raw6: bytes) -> dict:
    """解析 6 字节 PID: Kp(u16) Ki(u16) Kd(u16), 小端"""
    kp = struct.unpack_from("<H", raw6, 0)[0]
    ki = struct.unpack_from("<H", raw6, 2)[0]
    kd = struct.unpack_from("<H", raw6, 4)[0]
    return {"kp": kp, "ki": ki, "kd": kd}


def encode_pid(kp: int, ki: int, kd: int) -> bytes:
    """编码 PID 参数为 6 字节"""
    return struct.pack("<HHH", kp, ki, kd)


def read_status1(sock: socket.socket, motor_id: int) -> Optional[dict]:
    """0x9A 读取电机状态1: 温度、电压、电流、状态、错误"""
    cmd = bytes([0x9A, 0, 0, 0, 0, 0, 0, 0])
    resp = send_and_recv(sock, motor_id, cmd, expected_cmd=0x9A)
    if resp is None:
        return None
    return {
        "temperature_c": struct.unpack_from("<b", resp, 1)[0],
        "voltage_v": struct.unpack_from("<H", resp, 2)[0] * 0.01,
        "current_a": struct.unpack_from("<h", resp, 4)[0] * 0.01,
        "motor_state": resp[6],
        "error_state": resp[7],
    }


def read_status2_direct(sock: socket.socket, motor_id: int) -> Optional[dict]:
    """0x9C 读取电机状态2 (rx_id = 0x140 + motor_id)"""
    tx_id = 0x140 + motor_id
    rx_id = 0x140 + motor_id
    send_frame(sock, tx_id, bytes([0x9C, 0, 0, 0, 0, 0, 0, 0]))

    for _ in range(3):
        try:
            rid, rdata = recv_frame(sock)
            if rid == rx_id and len(rdata) >= 8 and rdata[0] == 0x9C:
                return {
                    "temperature_c": struct.unpack_from("<b", rdata, 1)[0],
                    "iq_raw": struct.unpack_from("<h", rdata, 2)[0],
                    "speed_dps": struct.unpack_from("<h", rdata, 4)[0],
                    "encoder": struct.unpack_from("<H", rdata, 6)[0],
                }
        except (socket.timeout, OSError, struct.error):
            continue
    return None


def read_multi_turn_angle(sock: socket.socket, motor_id: int) -> Optional[float]:
    """0x92 多圈角度 (rx_id = 0x140 + motor_id), 返回角度 (0.01°/LSB)"""
    tx_id = 0x140 + motor_id
    rx_id = 0x140 + motor_id
    send_frame(sock, tx_id, bytes([0x92, 0, 0, 0, 0, 0, 0, 0]))

    for _ in range(3):
        try:
            rid, rdata = recv_frame(sock)
            if rid == rx_id and len(rdata) >= 8 and rdata[0] == 0x92:
                return int.from_bytes(rdata[1:8], "little", signed=True) * 0.01
        except (socket.timeout, OSError, struct.error):
            continue
    return None


def motor_run(sock: socket.socket, motor_id: int) -> bool:
    """0x88 电机运行"""
    return send_and_recv(sock, motor_id, bytes([0x88, 0, 0, 0, 0, 0, 0, 0]),
                         expected_cmd=0x88) is not None


def motor_stop(sock: socket.socket, motor_id: int) -> bool:
    """0x81 电机停止"""
    return send_and_recv(sock, motor_id, bytes([0x81, 0, 0, 0, 0, 0, 0, 0]),
                         expected_cmd=0x81) is not None


def motor_off(sock: socket.socket, motor_id: int) -> bool:
    """0x80 电机关闭"""
    return send_and_recv(sock, motor_id, bytes([0x80, 0, 0, 0, 0, 0, 0, 0]),
                         expected_cmd=0x80) is not None


def read_torque_limit(sock: socket.socket, motor_id: int) -> Optional[int]:
    """读取最大力矩电流限制 (ParamID 0x1E)"""
    raw = read_control_param(sock, motor_id, PARAM_ID_TORQUE_LIMIT)
    if raw is None:
        return None
    return struct.unpack_from("<h", raw, 2)[0]


def read_speed_limit(sock: socket.socket, motor_id: int) -> Optional[int]:
    """读取最大速度限制 (ParamID 0x20)"""
    raw = read_control_param(sock, motor_id, PARAM_ID_SPEED_LIMIT)
    if raw is None:
        return None
    return struct.unpack_from("<i", raw, 2)[0]


def format_error_state(error_state: int) -> str:
    if error_state == 0:
        return "正常"
    errors = []
    for bit, desc in ERROR_BITS.items():
        if error_state & (1 << bit):
            errors.append(desc)
    return ", ".join(errors)


def send_position_command(
    sock: socket.socket, motor_id: int,
    angle_deg_motor_side: float, max_speed_dps: int,
) -> Optional[bytes]:
    """0xA4 多圈位置命令 (angle 单位 0.01°/LSB 电机侧)"""
    angle_lsb = int(angle_deg_motor_side * 100)
    speed_lsb = max_speed_dps & 0xFFFF
    data = bytes([0xA4, 0x00,
                  speed_lsb & 0xFF, (speed_lsb >> 8) & 0xFF,
                  angle_lsb & 0xFF, (angle_lsb >> 8) & 0xFF,
                  (angle_lsb >> 16) & 0xFF, (angle_lsb >> 24) & 0xFF])
    return send_and_recv(sock, motor_id, data, expected_cmd=0xA4)


def send_speed_command(
    sock: socket.socket, motor_id: int, speed_dps: int,
) -> Optional[bytes]:
    """0xA2 速度闭环命令"""
    speed_val = struct.pack("<i", speed_dps * 100)  # 0.01dps/LSB
    data = bytes([0xA2, 0x00, 0x00, 0x00]) + speed_val
    return send_and_recv(sock, motor_id, data, expected_cmd=0xA2)


def brake_control(sock: socket.socket, motor_id: int, action: int) -> Optional[int]:
    """0x8C 抱闸控制: action=0x00 锁定, 0x01 释放, 0x10 读取状态
    返回抱闸状态字节 (0x00=锁定, 0x01=释放), 失败返回 None"""
    data = bytes([0x8C, action, 0, 0, 0, 0, 0, 0])
    resp = send_and_recv(sock, motor_id, data, expected_cmd=0x8C)
    if resp is None:
        return None
    return resp[1]


# ============================================================
# DataBuffer — 线程安全环形缓冲区
# ============================================================

class DataBuffer:
    """QMutex 保护的环形缓冲区"""

    def __init__(self, max_points: int = 20000):
        self._mutex = QMutex()
        self._max = max_points
        self.timestamps = deque(maxlen=max_points)
        self.speed = deque(maxlen=max_points)
        self.position = deque(maxlen=max_points)
        self.current = deque(maxlen=max_points)

    def append(self, t: float, spd: float, pos: float, cur: float):
        locker = QMutexLocker(self._mutex)
        self.timestamps.append(t)
        self.speed.append(spd)
        self.position.append(pos)
        self.current.append(cur)

    def snapshot(self):
        """返回 (ts, speed, position, current) numpy 数组快照"""
        locker = QMutexLocker(self._mutex)
        n = len(self.timestamps)
        if n == 0:
            empty = np.array([])
            return empty, empty, empty, empty
        ts = np.array(self.timestamps)
        spd = np.array(self.speed)
        pos = np.array(self.position)
        cur = np.array(self.current)
        return ts, spd, pos, cur

    def clear(self):
        locker = QMutexLocker(self._mutex)
        self.timestamps.clear()
        self.speed.clear()
        self.position.clear()
        self.current.clear()


# ============================================================
# CanWorker — 运行在 QThread 的 CAN 操作
# ============================================================

class CanWorker(QObject):
    """所有 CAN 操作在此线程执行"""

    connected = pyqtSignal(bool, str)        # (success, message)
    pid_read = pyqtSignal(int, dict)         # (param_id, {kp, ki, kd})
    pid_write_ok = pyqtSignal(int, dict)     # (param_id, {kp, ki, kd})
    status_updated = pyqtSignal(dict)        # merged status dict
    error_occurred = pyqtSignal(str)         # error message
    step_done = pyqtSignal()
    brake_state = pyqtSignal(int)            # 抱闸状态: 0=锁定, 1=释放

    def __init__(self, data_buffer: DataBuffer):
        super().__init__()
        self._sock: Optional[socket.socket] = None
        self._can_iface = ""
        self._motor_id = 1
        self._data_buf = data_buffer
        self._start_time = 0.0
        self._gear_ratio = 36
        self._poll_count = 0
        self._slow_poll_divisor = 5000  # status1 每 N 次快轮询读一次
        # 采集掩码: 由 GUI 动态设置
        self._read_speed = True    # status2 (速度+电流)
        self._read_position = True # 多圈角度 (0x92)
        # 采样率统计
        self._rate_count = 0
        self._rate_time = 0.0

    @pyqtSlot(str, int)
    def do_connect(self, iface: str, motor_id: int):
        try:
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
            self._sock = create_can_socket(iface, timeout=0.005)
            self._can_iface = iface
            self._motor_id = motor_id
            self._start_time = time.monotonic()
            self._data_buf.clear()
            self.connected.emit(True, f"已连接 {iface} 电机{motor_id}")
        except OSError as e:
            self.connected.emit(False, f"连接失败: {e}")

    @pyqtSlot()
    def do_disconnect(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None

    @pyqtSlot(int)
    def do_read_pid(self, param_id: int):
        if self._sock is None:
            self.error_occurred.emit("未连接")
            return
        raw = read_control_param(self._sock, self._motor_id, param_id)
        if raw is None:
            self.error_occurred.emit(f"读取 {LOOP_LABEL.get(param_id, '?')} PID 失败")
            return
        self.pid_read.emit(param_id, parse_pid(raw))

    @pyqtSlot(int, int, int, int)
    def do_write_pid(self, param_id: int, kp: int, ki: int, kd: int):
        if self._sock is None:
            self.error_occurred.emit("未连接")
            return
        payload = encode_pid(kp, ki, kd)
        resp = write_control_param(self._sock, self._motor_id, param_id, payload)
        if resp is None:
            self.error_occurred.emit(f"写入 {LOOP_LABEL.get(param_id, '?')} PID 失败")
            return
        # 回读验证
        time.sleep(0.05)
        raw = read_control_param(self._sock, self._motor_id, param_id)
        if raw is not None:
            verified = parse_pid(raw)
            self.pid_write_ok.emit(param_id, verified)
        else:
            self.pid_write_ok.emit(param_id, {"kp": kp, "ki": ki, "kd": kd})

    @pyqtSlot(str)
    def do_motor_ctrl(self, action: str):
        if self._sock is None:
            self.error_occurred.emit("未连接")
            return
        funcs = {"run": motor_run, "stop": motor_stop, "off": motor_off}
        labels = {"run": "运行", "stop": "停止", "off": "关闭"}
        fn = funcs.get(action)
        if fn is None:
            return
        ok = fn(self._sock, self._motor_id)
        if not ok:
            self.error_occurred.emit(f"电机{labels[action]}命令失败")

    @pyqtSlot()
    def poll_status(self):
        """快轮询: 按需读取 status2/角度; 慢轮询: status1"""
        if self._sock is None:
            return

        self._poll_count += 1
        result = {}

        # 慢轮询: status1 (温度/电压/错误), 默认 ~50s 一次
        if self._poll_count % self._slow_poll_divisor == 0:
            s1 = read_status1(self._sock, self._motor_id)
            if s1:
                result.update(s1)

        # 快轮询: 按勾选状态决定读哪些
        if self._read_speed:
            s2 = read_status2_direct(self._sock, self._motor_id)
            if s2:
                result.update(s2)

        if self._read_position:
            angle = read_multi_turn_angle(self._sock, self._motor_id)
            if angle is not None:
                result["angle_deg"] = angle
                result["position_output"] = angle / self._gear_ratio

        # 写入数据缓冲区
        t = time.monotonic() - self._start_time
        spd = result.get("speed_dps", float("nan"))
        pos = result.get("position_output", float("nan"))
        cur_raw = result.get("iq_raw", float("nan"))
        cur = float(cur_raw) * 0.01 if cur_raw == cur_raw else float("nan")
        self._data_buf.append(t, float(spd), float(pos), float(cur))

        # 采样率统计
        self._rate_count += 1
        now = time.monotonic()
        dt = now - self._rate_time
        if dt >= 1.0:
            result["poll_hz"] = self._rate_count / dt
            self._rate_count = 0
            self._rate_time = now

        if result:
            self.status_updated.emit(result)

    @pyqtSlot(str, float, int)
    def do_step_test(self, mode: str, amplitude: float, max_speed: int):
        """阶跃测试"""
        if self._sock is None:
            self.error_occurred.emit("未连接")
            return

        if mode == "position":
            current_angle = read_multi_turn_angle(self._sock, self._motor_id)
            if current_angle is None:
                self.error_occurred.emit("读取当前角度失败")
                return
            target = current_angle + amplitude * self._gear_ratio
            send_position_command(
                self._sock, self._motor_id, target, max_speed,
            )
            self.step_done.emit()
        elif mode == "speed":
            send_speed_command(self._sock, self._motor_id, int(amplitude))
            # Non-blocking: emit step_done immediately, speed stop scheduled by MainWindow
            self.step_done.emit()

    @pyqtSlot()
    def do_speed_stop(self):
        """Stop speed command (called after delay by MainWindow)"""
        if self._sock is None:
            return
        send_speed_command(self._sock, self._motor_id, 0)

    @pyqtSlot(int)
    def do_brake_ctrl(self, action: int):
        """抱闸控制: 0=锁定, 1=释放, 0x10=读取"""
        if self._sock is None:
            self.error_occurred.emit("未连接")
            return
        result = brake_control(self._sock, self._motor_id, action)
        if result is None:
            self.error_occurred.emit("抱闸命令失败")
            return
        self.brake_state.emit(result)


# ============================================================
# PidLoopWidget — 单个环路的 PID 控件
# ============================================================

class PidLoopWidget(QGroupBox):

    read_requested = pyqtSignal(int)         # param_id
    write_requested = pyqtSignal(int, int, int, int)  # param_id, kp, ki, kd

    def __init__(self, label: str, param_id: int, parent=None):
        super().__init__(label, parent)
        self._param_id = param_id

        layout = QVBoxLayout(self)

        # Kp
        row_kp = QHBoxLayout()
        row_kp.addWidget(QLabel("Kp"))
        self.sp_kp = QSpinBox()
        self.sp_kp.setRange(0, 65535)
        row_kp.addWidget(self.sp_kp)
        layout.addLayout(row_kp)

        # Ki
        row_ki = QHBoxLayout()
        row_ki.addWidget(QLabel("Ki"))
        self.sp_ki = QSpinBox()
        self.sp_ki.setRange(0, 65535)
        row_ki.addWidget(self.sp_ki)
        layout.addLayout(row_ki)

        # Kd
        row_kd = QHBoxLayout()
        row_kd.addWidget(QLabel("Kd"))
        self.sp_kd = QSpinBox()
        self.sp_kd.setRange(0, 65535)
        row_kd.addWidget(self.sp_kd)
        layout.addLayout(row_kd)

        # 按钮
        btn_row = QHBoxLayout()
        btn_read = QPushButton("读取")
        btn_write = QPushButton("写入RAM")
        btn_read.clicked.connect(lambda: self.read_requested.emit(self._param_id))
        btn_write.clicked.connect(self._on_write)
        btn_row.addWidget(btn_read)
        btn_row.addWidget(btn_write)
        layout.addLayout(btn_row)

    def _on_write(self):
        self.write_requested.emit(
            self._param_id,
            self.sp_kp.value(),
            self.sp_ki.value(),
            self.sp_kd.value(),
        )

    def set_values(self, kp: int, ki: int, kd: int):
        self.sp_kp.setValue(kp)
        self.sp_ki.setValue(ki)
        self.sp_kd.setValue(kd)

    def get_values(self) -> dict:
        return {
            "kp": self.sp_kp.value(),
            "ki": self.sp_ki.value(),
            "kd": self.sp_kd.value(),
        }


# ============================================================
# ConnectionPanel — 顶栏
# ============================================================

class ConnectionPanel(QWidget):

    connect_requested = pyqtSignal(str, int)
    disconnect_requested = pyqtSignal()
    estop_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        layout.addWidget(QLabel("CAN:"))
        self.cmb_can = QComboBox()
        self.cmb_can.setEditable(True)
        self.cmb_can.addItems(["can0", "can1", "vcan0"])
        layout.addWidget(self.cmb_can)

        layout.addWidget(QLabel("电机ID:"))
        self.sp_id = QSpinBox()
        self.sp_id.setRange(1, 32)
        self.sp_id.setValue(1)
        layout.addWidget(self.sp_id)

        self.btn_connect = QPushButton("连接")
        self.btn_connect.clicked.connect(self._on_connect)
        layout.addWidget(self.btn_connect)

        self.lbl_status = QLabel("●未连接")
        self.lbl_status.setStyleSheet("color: gray; font-weight: bold;")
        layout.addWidget(self.lbl_status)

        layout.addStretch()

        self.btn_estop = QPushButton("紧急停止 ESC")
        self.btn_estop.setStyleSheet(
            "QPushButton { background-color: #D32F2F; color: white; "
            "font-weight: bold; padding: 6px 16px; border-radius: 4px; }"
            "QPushButton:hover { background-color: #B71C1C; }"
        )
        self.btn_estop.clicked.connect(self.estop_requested.emit)
        layout.addWidget(self.btn_estop)

        self._connected = False

    def _on_connect(self):
        if self._connected:
            self.disconnect_requested.emit()
        else:
            self.connect_requested.emit(
                self.cmb_can.currentText(), self.sp_id.value()
            )

    def set_connected(self, ok: bool, msg: str):
        self._connected = ok
        if ok:
            self.lbl_status.setText("●已连接")
            self.lbl_status.setStyleSheet("color: #4CAF50; font-weight: bold;")
            self.btn_connect.setText("断开")
            self.cmb_can.setEnabled(False)
            self.sp_id.setEnabled(False)
        else:
            self.lbl_status.setText(f"●{msg}")
            self.lbl_status.setStyleSheet("color: red; font-weight: bold;")
            self.btn_connect.setText("连接")
            self.cmb_can.setEnabled(True)
            self.sp_id.setEnabled(True)

    def set_disconnected(self):
        self._connected = False
        self.lbl_status.setText("●未连接")
        self.lbl_status.setStyleSheet("color: gray; font-weight: bold;")
        self.btn_connect.setText("连接")
        self.cmb_can.setEnabled(True)
        self.sp_id.setEnabled(True)


# ============================================================
# PidPanel — 三环 PID + 备份/恢复
# ============================================================

class PidPanel(QWidget):

    read_pid = pyqtSignal(int)
    write_pid = pyqtSignal(int, int, int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)

        self.loops = {}
        for param_id, label in LOOP_LABEL.items():
            w = PidLoopWidget(f"{label} (0x{param_id:02X})", param_id)
            w.read_requested.connect(self.read_pid.emit)
            w.write_requested.connect(self.write_pid.emit)
            self.loops[param_id] = w
            layout.addWidget(w)

        btn_row = QHBoxLayout()
        btn_all = QPushButton("全部读取")
        btn_all.clicked.connect(self._read_all)
        btn_row.addWidget(btn_all)
        layout.addLayout(btn_row)

        btn_row2 = QHBoxLayout()
        btn_backup = QPushButton("备份出厂")
        btn_restore = QPushButton("恢复出厂")
        btn_backup.clicked.connect(self._backup)
        btn_restore.clicked.connect(self._restore)
        btn_row2.addWidget(btn_backup)
        btn_row2.addWidget(btn_restore)
        layout.addLayout(btn_row2)

        layout.addStretch()

        self.factory_values: dict = {}

    def _read_all(self):
        for param_id in LOOP_LABEL:
            self.read_pid.emit(param_id)

    def _backup(self):
        for param_id, w in self.loops.items():
            self.factory_values[param_id] = w.get_values()
        QMessageBox.information(self, "备份", "出厂参数已备份到内存")

    def _restore(self):
        if not self.factory_values:
            QMessageBox.warning(self, "恢复", "请先备份出厂参数")
            return
        for param_id, vals in self.factory_values.items():
            self.write_pid.emit(param_id, vals["kp"], vals["ki"], vals["kd"])

    def update_pid(self, param_id: int, vals: dict):
        w = self.loops.get(param_id)
        if w:
            w.set_values(vals["kp"], vals["ki"], vals["kd"])


# ============================================================
# MotorControlPanel — 运行/停止/关闭 + 状态
# ============================================================

class MotorControlPanel(QWidget):

    motor_ctrl = pyqtSignal(str)
    brake_ctrl = pyqtSignal(int)             # 0=锁定, 1=释放

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)

        # 控制按钮
        btn_row = QHBoxLayout()
        for action, label, color in [
            ("run", "运行", "#4CAF50"),
            ("stop", "停止", "#FF9800"),
            ("off", "关闭", "#F44336"),
        ]:
            btn = QPushButton(label)
            btn.setStyleSheet(
                f"QPushButton {{ background-color: {color}; color: white; "
                f"padding: 4px 12px; border-radius: 3px; }}"
            )
            btn.clicked.connect(lambda checked, a=action: self.motor_ctrl.emit(a))
            btn_row.addWidget(btn)
        layout.addLayout(btn_row)

        # 抱闸控制按钮
        brake_row = QHBoxLayout()
        self.btn_brake_lock = QPushButton("抱闸锁定")
        self.btn_brake_lock.setStyleSheet(
            "QPushButton { background-color: #795548; color: white; "
            "padding: 4px 12px; border-radius: 3px; }"
            "QPushButton:hover { background-color: #5D4037; }"
        )
        self.btn_brake_lock.clicked.connect(lambda: self.brake_ctrl.emit(0x00))
        brake_row.addWidget(self.btn_brake_lock)

        self.btn_brake_release = QPushButton("抱闸释放")
        self.btn_brake_release.setStyleSheet(
            "QPushButton { background-color: #607D8B; color: white; "
            "padding: 4px 12px; border-radius: 3px; }"
            "QPushButton:hover { background-color: #455A64; }"
        )
        self.btn_brake_release.clicked.connect(lambda: self.brake_ctrl.emit(0x01))
        brake_row.addWidget(self.btn_brake_release)
        layout.addLayout(brake_row)

        # 抱闸状态标签
        self.lbl_brake = QLabel("抱闸: --")
        layout.addWidget(self.lbl_brake)

        # 状态标签
        self.lbl_temp = QLabel("温度: --")
        self.lbl_voltage = QLabel("电压: --")
        self.lbl_current = QLabel("电流: --")
        self.lbl_speed = QLabel("转速: --")
        self.lbl_state = QLabel("状态: --")
        self.lbl_error = QLabel("错误: --")
        self.lbl_rate = QLabel("采样: -- Hz")
        self.lbl_rate.setStyleSheet("color: #666;")

        for lbl in [self.lbl_temp, self.lbl_voltage, self.lbl_current,
                     self.lbl_speed, self.lbl_state, self.lbl_error,
                     self.lbl_rate]:
            layout.addWidget(lbl)

    def update_status(self, data: dict):
        if "temperature_c" in data:
            t = data["temperature_c"]
            self.lbl_temp.setText(f"温度: {t}°C")

        if "voltage_v" in data:
            self.lbl_voltage.setText(f"电压: {data['voltage_v']:.1f}V")

        if "current_a" in data:
            self.lbl_current.setText(f"电流: {data['current_a']:.2f}A")

        if "speed_dps" in data:
            self.lbl_speed.setText(f"转速: {data['speed_dps']}dps")

        if "motor_state" in data:
            ms = data["motor_state"]
            state_str = "关闭" if ms == 0x00 else "运行"
            self.lbl_state.setText(f"状态: {state_str}")

        if "error_state" in data:
            es = data["error_state"]
            self.lbl_error.setText(f"错误: {format_error_state(es)}")
            if es != 0:
                self.lbl_error.setStyleSheet("color: red; font-weight: bold;")
            else:
                self.lbl_error.setStyleSheet("")

        if "poll_hz" in data:
            hz = data["poll_hz"]
            self.lbl_rate.setText(f"采样: {hz:.0f} Hz")

    def update_brake_state(self, state: int):
        if state == 0x00:
            self.lbl_brake.setText("抱闸: 锁定")
            self.lbl_brake.setStyleSheet("color: #795548; font-weight: bold;")
        elif state == 0x01:
            self.lbl_brake.setText("抱闸: 释放")
            self.lbl_brake.setStyleSheet("color: #607D8B; font-weight: bold;")
        else:
            self.lbl_brake.setText(f"抱闸: 未知(0x{state:02X})")
            self.lbl_brake.setStyleSheet("")


# ============================================================
# WaveformPanel — matplotlib 嵌入实时波形
# ============================================================

class WaveformPanel(QWidget):

    # (need_speed_or_current, need_position)
    signals_changed = pyqtSignal(bool, bool)

    def __init__(self, data_buffer: DataBuffer, parent=None):
        super().__init__(parent)
        self._buf = data_buffer
        self._paused = False
        self._auto_scroll = True
        self._window_sec = 10.0

        # 框选放大状态
        self._drag_start = None   # (ax, x0, y0) 起点
        self._drag_rect = None    # matplotlib Rectangle patch

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # matplotlib figure — 子图动态重建
        self.fig = Figure(figsize=(8, 5), dpi=100, facecolor="#FAFAFA")
        self.canvas = FigureCanvasQTAgg(self.fig)
        layout.addWidget(self.canvas, stretch=1)

        # axes / lines 字典, 按信号名索引
        self._axes = {}    # {"speed": ax, "position": ax, "current": ax}
        self._lines = {}   # {"speed": line, ...}

        # 信号配置
        self._sig_cfg = {
            "speed":    {"ylabel": "转速 (dps)",  "color": "#2196F3"},
            "position": {"ylabel": "位置 (deg)",  "color": "#4CAF50"},
            "current":  {"ylabel": "转矩电流 (A)",  "color": "#FF5722"},
        }

        # 事件连接
        self.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.canvas.mpl_connect("button_press_event", self._on_press)
        self.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.canvas.mpl_connect("button_release_event", self._on_release)

        # 控制按钮行
        ctrl_row = QHBoxLayout()

        self.btn_pause = QPushButton("暂停")
        self.btn_pause.clicked.connect(self._toggle_pause)
        ctrl_row.addWidget(self.btn_pause)

        btn_auto = QPushButton("自适应")
        btn_auto.clicked.connect(self._auto_fit)
        ctrl_row.addWidget(btn_auto)

        btn_export = QPushButton("导出CSV")
        btn_export.clicked.connect(self._export_csv)
        ctrl_row.addWidget(btn_export)

        layout.addLayout(ctrl_row)

        # 信号勾选
        chk_row = QHBoxLayout()
        self.chk_speed = QCheckBox("速度")
        self.chk_speed.setChecked(True)
        self.chk_pos = QCheckBox("位置")
        self.chk_pos.setChecked(True)
        self.chk_cur = QCheckBox("电流")
        self.chk_cur.setChecked(True)
        chk_row.addWidget(self.chk_speed)
        chk_row.addWidget(self.chk_pos)
        chk_row.addWidget(self.chk_cur)
        chk_row.addStretch()
        self.chk_cursor = QCheckBox("Y游标")
        self.chk_cursor.setChecked(False)
        chk_row.addWidget(self.chk_cursor)
        layout.addLayout(chk_row)

        # Y游标状态
        self._cursor_lines = {}   # {ax: hline}
        self._cursor_texts = {}   # {ax: text}
        self.chk_cursor.stateChanged.connect(self._on_cursor_toggled)

        # 勾选变化 → 重建子图 + 通知采集掩码
        self.chk_speed.stateChanged.connect(self._on_signals_changed)
        self.chk_pos.stateChanged.connect(self._on_signals_changed)
        self.chk_cur.stateChanged.connect(self._on_signals_changed)

        # 初始构建子图
        self._rebuild_axes()

    def _on_cursor_toggled(self):
        """Y游标开关"""
        if not self.chk_cursor.isChecked():
            self._remove_cursors()
            self.canvas.draw_idle()

    def _remove_cursors(self):
        for hline in self._cursor_lines.values():
            hline.remove()
        for txt in self._cursor_texts.values():
            txt.remove()
        self._cursor_lines.clear()
        self._cursor_texts.clear()

    def _checked_signals(self) -> list:
        """返回当前勾选的信号名列表"""
        sigs = []
        if self.chk_speed.isChecked():
            sigs.append("speed")
        if self.chk_pos.isChecked():
            sigs.append("position")
        if self.chk_cur.isChecked():
            sigs.append("current")
        return sigs

    def _rebuild_axes(self):
        """根据勾选状态重建子图"""
        self.fig.clear()
        self._axes.clear()
        self._lines.clear()
        self._cursor_lines.clear()
        self._cursor_texts.clear()

        sigs = self._checked_signals()
        n = len(sigs)
        if n == 0:
            self.canvas.draw_idle()
            return

        for i, sig in enumerate(sigs):
            ax = self.fig.add_subplot(n, 1, i + 1)
            cfg = self._sig_cfg[sig]
            ax.set_ylabel(cfg["ylabel"], fontsize=9)
            ax.grid(True, alpha=0.3, linestyle="--")
            ax.set_facecolor("#FFFFFF")
            ax.tick_params(labelsize=8)
            if i < n - 1:
                ax.set_xticklabels([])
            else:
                ax.set_xlabel("时间 (s)", fontsize=9)
            line, = ax.plot([], [], color=cfg["color"], linewidth=1.2)
            self._axes[sig] = ax
            self._lines[sig] = line

        self.fig.tight_layout(pad=1.5)
        self.canvas.draw_idle()

    def _on_signals_changed(self):
        self._rebuild_axes()
        need_status2 = self.chk_speed.isChecked() or self.chk_cur.isChecked()
        need_position = self.chk_pos.isChecked()
        self.signals_changed.emit(need_status2, need_position)

    def _toggle_pause(self):
        self._paused = not self._paused
        self.btn_pause.setText("恢复" if self._paused else "暂停")

    def _auto_fit(self):
        self._auto_scroll = True

    # ---- 滚轮缩放 ----

    def _on_scroll(self, event):
        ax = event.inaxes
        if ax is None:
            return
        self._auto_scroll = False
        factor = 0.8 if event.button == "up" else 1.25

        xlim = ax.get_xlim()
        if event.xdata is not None:
            w = (xlim[1] - xlim[0]) * factor
            r = (event.xdata - xlim[0]) / (xlim[1] - xlim[0])
            ax.set_xlim(event.xdata - w * r, event.xdata + w * (1 - r))

        ylim = ax.get_ylim()
        if event.ydata is not None:
            h = (ylim[1] - ylim[0]) * factor
            r = (event.ydata - ylim[0]) / (ylim[1] - ylim[0])
            ax.set_ylim(event.ydata - h * r, event.ydata + h * (1 - r))

        self.canvas.draw_idle()

    # ---- 鼠标左键框选放大 ----

    def _on_press(self, event):
        if event.button != 1 or event.inaxes is None:
            return
        self._drag_start = (event.inaxes, event.xdata, event.ydata)
        # 画选区矩形
        from matplotlib.patches import Rectangle
        self._drag_rect = Rectangle(
            (event.xdata, event.ydata), 0, 0,
            linewidth=1, edgecolor="#1976D2", facecolor="#1976D2",
            alpha=0.15, linestyle="--",
        )
        event.inaxes.add_patch(self._drag_rect)

    def _on_motion(self, event):
        # 框选拖拽
        if self._drag_start is not None and self._drag_rect is not None:
            ax0, x0, y0 = self._drag_start
            if event.inaxes == ax0 and event.xdata is not None:
                self._drag_rect.set_x(min(x0, event.xdata))
                self._drag_rect.set_y(min(y0, event.ydata))
                self._drag_rect.set_width(abs(event.xdata - x0))
                self._drag_rect.set_height(abs(event.ydata - y0))
                self.canvas.draw_idle()
            return

        # Y游标
        if not self.chk_cursor.isChecked():
            return
        if event.inaxes is None or event.ydata is None:
            return

        ax = event.inaxes
        if ax not in self._axes.values():
            return

        # 更新或创建该 ax 的游标线
        if ax in self._cursor_lines:
            self._cursor_lines[ax].set_ydata([event.ydata, event.ydata])
            self._cursor_texts[ax].set_position((ax.get_xlim()[1], event.ydata))
            self._cursor_texts[ax].set_text(f" {event.ydata:.2f}")
        else:
            hline = ax.axhline(event.ydata, color="#E91E63", linewidth=0.8,
                               linestyle="--", alpha=0.7)
            txt = ax.text(ax.get_xlim()[1], event.ydata, f" {event.ydata:.2f}",
                          fontsize=8, color="#E91E63", va="bottom",
                          ha="right", fontweight="bold")
            self._cursor_lines[ax] = hline
            self._cursor_texts[ax] = txt

        # 清理不在当前 ax 的游标
        for other_ax in list(self._cursor_lines.keys()):
            if other_ax is not ax:
                self._cursor_lines.pop(other_ax).remove()
                self._cursor_texts.pop(other_ax).remove()

        self.canvas.draw_idle()

    def _on_release(self, event):
        if self._drag_start is None:
            return
        ax0, x0, y0 = self._drag_start
        # 清理矩形
        if self._drag_rect is not None:
            self._drag_rect.remove()
            self._drag_rect = None

        self._drag_start = None

        if event.button != 1 or event.inaxes != ax0:
            self.canvas.draw_idle()
            return
        if event.xdata is None or event.ydata is None:
            self.canvas.draw_idle()
            return

        # 最小拖拽距离 (像素), 防止点击误触
        dx_px = abs(event.x - self.canvas.figure.dpi * abs(event.xdata - x0))
        x_lo, x_hi = sorted([x0, event.xdata])
        y_lo, y_hi = sorted([y0, event.ydata])

        # 选区太小则忽略
        xlim = ax0.get_xlim()
        ylim = ax0.get_ylim()
        if (x_hi - x_lo) < (xlim[1] - xlim[0]) * 0.01:
            self.canvas.draw_idle()
            return
        if (y_hi - y_lo) < (ylim[1] - ylim[0]) * 0.01:
            self.canvas.draw_idle()
            return

        self._auto_scroll = False
        ax0.set_xlim(x_lo, x_hi)
        ax0.set_ylim(y_lo, y_hi)
        self.canvas.draw_idle()

    # ---- 实时更新 ----

    def update_plot(self):
        if self._paused:
            return

        ts, spd, pos, cur = self._buf.snapshot()
        if len(ts) < 2:
            return

        data_map = {"speed": spd, "position": pos, "current": cur}

        for sig, line in self._lines.items():
            line.set_data(ts, data_map[sig])

        if self._auto_scroll:
            t_now = ts[-1]
            t_start = max(0.0, t_now - self._window_sec)
            for sig, ax in self._axes.items():
                ax.set_xlim(t_start, t_now + self._window_sec * 0.02)
                arr = data_map[sig]
                mask = ts >= t_start
                visible = arr[mask]
                finite = visible[np.isfinite(visible)]
                if len(finite) > 0:
                    lo, hi = finite.min(), finite.max()
                    margin = max(abs(hi - lo) * 0.1, 1.0)
                    ax.set_ylim(lo - margin, hi + margin)

        self.canvas.draw_idle()

    def _export_csv(self):
        ts, spd, pos, cur = self._buf.snapshot()
        if len(ts) == 0:
            QMessageBox.information(self, "导出", "无数据可导出")
            return

        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"waveform_{ts_str}.csv"
        path, _ = QFileDialog.getSaveFileName(
            self, "导出波形数据", default_name, "CSV files (*.csv)"
        )
        if not path:
            return

        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_s", "speed_dps", "position_deg", "current_raw"])
            for i in range(len(ts)):
                writer.writerow([
                    f"{ts[i]:.4f}",
                    f"{spd[i]:.2f}" if np.isfinite(spd[i]) else "",
                    f"{pos[i]:.4f}" if np.isfinite(pos[i]) else "",
                    f"{cur[i]:.2f}" if np.isfinite(cur[i]) else "",
                ])

        QMessageBox.information(self, "导出", f"已导出 {len(ts)} 条数据到\n{path}")


# ============================================================
# StepTestPanel — 阶跃测试
# ============================================================

class StepTestPanel(QGroupBox):

    step_requested = pyqtSignal(str, float, int)  # mode, amplitude, max_speed

    def __init__(self, parent=None):
        super().__init__("阶跃测试", parent)
        layout = QVBoxLayout(self)

        # 模式选择
        mode_row = QHBoxLayout()
        self.rb_pos = QRadioButton("位置阶跃")
        self.rb_spd = QRadioButton("速度阶跃")
        self.rb_pos.setChecked(True)
        self._bg = QButtonGroup(self)
        self._bg.addButton(self.rb_pos)
        self._bg.addButton(self.rb_spd)
        mode_row.addWidget(self.rb_pos)
        mode_row.addWidget(self.rb_spd)
        layout.addLayout(mode_row)

        # 参数
        param_row = QHBoxLayout()
        param_row.addWidget(QLabel("幅度:"))
        self.sp_amplitude = QDoubleSpinBox()
        self.sp_amplitude.setRange(-3600, 3600)
        self.sp_amplitude.setValue(30.0)
        self.sp_amplitude.setDecimals(1)
        param_row.addWidget(self.sp_amplitude)

        param_row.addWidget(QLabel("最大速度:"))
        self.sp_speed = QSpinBox()
        self.sp_speed.setRange(1, 10000)
        self.sp_speed.setValue(100)
        param_row.addWidget(self.sp_speed)
        layout.addLayout(param_row)

        self.btn_exec = QPushButton("执行")
        self.btn_exec.clicked.connect(self._on_exec)
        layout.addWidget(self.btn_exec)

    def _on_exec(self):
        mode = "position" if self.rb_pos.isChecked() else "speed"
        self.btn_exec.setEnabled(False)
        self.btn_exec.setText("执行中...")
        self.step_requested.emit(
            mode,
            self.sp_amplitude.value(),
            self.sp_speed.value(),
        )

    def on_step_done(self):
        self.btn_exec.setEnabled(True)
        self.btn_exec.setText("执行")


# ============================================================
# TuningLogPanel — 调试记录表格
# ============================================================

class TuningLogPanel(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)

        lbl = QLabel("调试记录")
        lbl.setStyleSheet("font-weight: bold;")
        layout.addWidget(lbl)

        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(
            ["#", "时间", "环路", "Kp", "Ki", "Kd"]
        )
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setColumnWidth(0, 30)
        layout.addWidget(self.table)

        btn_export = QPushButton("导出日志CSV")
        btn_export.clicked.connect(self._export)
        layout.addWidget(btn_export)

        self._seq = 0

    def add_entry(self, param_id: int, vals: dict):
        self._seq += 1
        row = self.table.rowCount()
        self.table.insertRow(row)

        items = [
            str(self._seq),
            datetime.now().strftime("%H:%M:%S"),
            LOOP_LABEL.get(param_id, "?"),
            str(vals["kp"]),
            str(vals["ki"]),
            str(vals["kd"]),
        ]

        for col, text in enumerate(items):
            item = QTableWidgetItem(text)
            if col < 3:
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            self.table.setItem(row, col, item)

        self.table.scrollToBottom()

    def _export(self):
        if self.table.rowCount() == 0:
            QMessageBox.information(self, "导出", "无记录可导出")
            return

        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        path, _ = QFileDialog.getSaveFileName(
            self, "导出日志", f"tuning_log_{ts_str}.csv", "CSV files (*.csv)"
        )
        if not path:
            return

        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            headers = [
                self.table.horizontalHeaderItem(c).text()
                for c in range(self.table.columnCount())
            ]
            writer.writerow(headers)
            for r in range(self.table.rowCount()):
                row_data = []
                for c in range(self.table.columnCount()):
                    item = self.table.item(r, c)
                    row_data.append(item.text() if item else "")
                writer.writerow(row_data)

        QMessageBox.information(self, "导出", f"已导出到\n{path}")


# ============================================================
# MainWindow — 组装所有面板
# ============================================================

class MainWindow(QMainWindow):

    # Signals for cross-thread invocation of worker slots
    _sig_connect = pyqtSignal(str, int)
    _sig_disconnect = pyqtSignal()
    _sig_read_pid = pyqtSignal(int)
    _sig_write_pid = pyqtSignal(int, int, int, int)
    _sig_motor_ctrl = pyqtSignal(str)
    _sig_poll = pyqtSignal()
    _sig_step = pyqtSignal(str, float, int)
    _sig_speed_stop = pyqtSignal()
    _sig_brake_ctrl = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("RMD 电机 PID 整定上位机")
        self.resize(1200, 800)

        self._estop_sock: Optional[socket.socket] = None
        self._connected = False
        self._step_mode = ""

        # 数据缓冲区
        self._data_buf = DataBuffer()

        # Worker + Thread
        self._worker = CanWorker(self._data_buf)
        self._thread = QThread()
        self._worker.moveToThread(self._thread)
        self._thread.start()

        # 连接 worker 信号
        self._sig_connect.connect(self._worker.do_connect)
        self._sig_disconnect.connect(self._worker.do_disconnect)
        self._sig_read_pid.connect(self._worker.do_read_pid)
        self._sig_write_pid.connect(self._worker.do_write_pid)
        self._sig_motor_ctrl.connect(self._worker.do_motor_ctrl)
        self._sig_poll.connect(self._worker.poll_status)
        self._sig_step.connect(self._worker.do_step_test)
        self._sig_speed_stop.connect(self._worker.do_speed_stop)
        self._sig_brake_ctrl.connect(self._worker.do_brake_ctrl)

        self._worker.connected.connect(self._on_connected)
        self._worker.pid_read.connect(self._on_pid_read)
        self._worker.pid_write_ok.connect(self._on_pid_write_ok)
        self._worker.status_updated.connect(self._on_status_updated)
        self._worker.error_occurred.connect(self._on_error)
        self._worker.step_done.connect(self._on_step_done)
        self._worker.brake_state.connect(self._on_brake_state)

        self._build_ui()
        self._setup_timers()

        # ESC 快捷键
        shortcut = QShortcut(Qt.Key_Escape, self)
        shortcut.activated.connect(self._emergency_stop)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(4, 4, 4, 4)

        # -- 顶栏 --
        self.conn_panel = ConnectionPanel()
        self.conn_panel.connect_requested.connect(
            lambda iface, mid: self._sig_connect.emit(iface, mid)
        )
        self.conn_panel.disconnect_requested.connect(self._do_disconnect)
        self.conn_panel.estop_requested.connect(self._emergency_stop)
        main_layout.addWidget(self.conn_panel)

        # -- 外层垂直 splitter: [主体 | 调试记录] --
        outer_splitter = QSplitter(Qt.Vertical)

        # -- 主体水平 splitter: [左侧 | 右侧] --
        h_splitter = QSplitter(Qt.Horizontal)

        # 左侧: PID + 电机控制 (垂直 splitter)
        left_splitter = QSplitter(Qt.Vertical)

        self.pid_panel = PidPanel()
        self.pid_panel.read_pid.connect(self._on_read_pid_request)
        self.pid_panel.write_pid.connect(self._on_write_pid_request)
        left_splitter.addWidget(self.pid_panel)

        self.motor_panel = MotorControlPanel()
        self.motor_panel.motor_ctrl.connect(self._sig_motor_ctrl.emit)
        self.motor_panel.brake_ctrl.connect(self._on_brake_request)
        left_splitter.addWidget(self.motor_panel)

        left_splitter.setStretchFactor(0, 3)
        left_splitter.setStretchFactor(1, 1)
        h_splitter.addWidget(left_splitter)

        # 右侧: 波形 + 阶跃测试 (垂直 splitter)
        right_splitter = QSplitter(Qt.Vertical)

        self.waveform_panel = WaveformPanel(self._data_buf)
        self.waveform_panel.signals_changed.connect(self._on_signals_changed)
        right_splitter.addWidget(self.waveform_panel)

        self.step_panel = StepTestPanel()
        self.step_panel.step_requested.connect(self._on_step_request)
        right_splitter.addWidget(self.step_panel)

        right_splitter.setStretchFactor(0, 5)
        right_splitter.setStretchFactor(1, 1)
        h_splitter.addWidget(right_splitter)

        h_splitter.setStretchFactor(0, 1)
        h_splitter.setStretchFactor(1, 3)
        outer_splitter.addWidget(h_splitter)

        # -- 底部调试记录 --
        self.log_panel = TuningLogPanel()
        outer_splitter.addWidget(self.log_panel)

        outer_splitter.setStretchFactor(0, 4)
        outer_splitter.setStretchFactor(1, 1)
        main_layout.addWidget(outer_splitter, stretch=1)

    def _setup_timers(self):
        # 状态轮询 5ms (~200Hz max), 实际受 CAN 响应延迟限制
        self._status_timer = QTimer(self)
        self._status_timer.setInterval(5)
        self._status_timer.timeout.connect(self._sig_poll.emit)

        # 波形更新 50ms
        self._wave_timer = QTimer(self)
        self._wave_timer.setInterval(50)
        self._wave_timer.timeout.connect(self.waveform_panel.update_plot)

    # ---- Pause/resume polling for multi-command ops ----

    def _pause_polling(self):
        self._status_timer.stop()

    def _resume_polling(self):
        if self._connected:
            self._status_timer.start()

    # ---- PID read/write with polling paused ----

    def _on_read_pid_request(self, param_id: int):
        self._pause_polling()
        # 延迟 100ms 发出读取，等 worker 队列中积压的 poll 信号消化掉
        QTimer.singleShot(100, lambda: self._sig_read_pid.emit(param_id))
        QTimer.singleShot(800, self._resume_polling)

    def _on_write_pid_request(self, param_id: int, kp: int, ki: int, kd: int):
        self._pause_polling()
        QTimer.singleShot(100, lambda: self._sig_write_pid.emit(param_id, kp, ki, kd))
        QTimer.singleShot(800, self._resume_polling)

    # ---- Step test with polling paused for speed, delayed stop ----

    def _on_step_request(self, mode: str, amplitude: float, max_speed: int):
        self._step_mode = mode
        self._sig_step.emit(mode, amplitude, max_speed)

    def _on_signals_changed(self, need_status2: bool, need_position: bool):
        """勾选框变化 → 更新 worker 采集掩码，取消不需要的 CAN 读取"""
        self._worker._read_speed = need_status2
        self._worker._read_position = need_position

    # ---- Worker 回调 ----

    def _on_connected(self, ok: bool, msg: str):
        self._connected = ok
        self.conn_panel.set_connected(ok, msg)
        if ok:
            self._status_timer.start()
            self._wave_timer.start()
            # 创建紧急停止专用 socket
            try:
                iface = self.conn_panel.cmb_can.currentText()
                self._estop_sock = create_can_socket(iface, timeout=0.1)
            except OSError:
                self._estop_sock = None
        else:
            self.statusBar().showMessage(msg, 5000)

    def _do_disconnect(self):
        self._status_timer.stop()
        self._wave_timer.stop()
        self._sig_disconnect.emit()
        self._connected = False
        self.conn_panel.set_disconnected()
        if self._estop_sock:
            try:
                self._estop_sock.close()
            except Exception:
                pass
            self._estop_sock = None

    def _on_pid_read(self, param_id: int, vals: dict):
        self.pid_panel.update_pid(param_id, vals)
        self.statusBar().showMessage(
            f"读取 {LOOP_LABEL.get(param_id, '?')}: "
            f"Kp={vals['kp']} Ki={vals['ki']} Kd={vals['kd']}", 3000
        )

    def _on_pid_write_ok(self, param_id: int, vals: dict):
        self.pid_panel.update_pid(param_id, vals)
        self.log_panel.add_entry(param_id, vals)
        self.statusBar().showMessage(
            f"写入 {LOOP_LABEL.get(param_id, '?')}: "
            f"Kp={vals['kp']} Ki={vals['ki']} Kd={vals['kd']} -> 成功", 3000
        )

    def _on_status_updated(self, data: dict):
        self.motor_panel.update_status(data)

    def _on_error(self, msg: str):
        self.statusBar().showMessage(f"错误: {msg}", 5000)

    def _on_step_done(self):
        if self._step_mode == "speed":
            # Schedule speed=0 after 3 seconds (non-blocking)
            QTimer.singleShot(3000, self._sig_speed_stop.emit)
            self.statusBar().showMessage("速度阶跃中... 3秒后自动停止", 3000)
        else:
            self.step_panel.on_step_done()
            self.statusBar().showMessage("阶跃测试完成", 3000)
        # Reset for speed step done callback
        if self._step_mode == "speed":
            QTimer.singleShot(3200, self._on_speed_step_finished)

    def _on_speed_step_finished(self):
        self.step_panel.on_step_done()
        self.statusBar().showMessage("速度阶跃完成", 3000)

    def _on_brake_request(self, action: int):
        self._pause_polling()
        QTimer.singleShot(100, lambda: self._sig_brake_ctrl.emit(action))
        QTimer.singleShot(800, self._resume_polling)

    def _on_brake_state(self, state: int):
        self.motor_panel.update_brake_state(state)
        label = "锁定" if state == 0x00 else "释放"
        self.statusBar().showMessage(f"抱闸状态: {label}", 3000)

    def _emergency_stop(self):
        """紧急停止 — 在 GUI 线程直接发送 0x80"""
        motor_id = self.conn_panel.sp_id.value()

        if self._estop_sock is not None:
            try:
                tx_id = 0x140 + motor_id
                send_frame(self._estop_sock, tx_id, bytes([0x80, 0, 0, 0, 0, 0, 0, 0]))
            except Exception:
                pass

        # 也通过 worker 发送
        self._sig_motor_ctrl.emit("off")
        self.statusBar().showMessage("紧急停止!", 5000)

    def closeEvent(self, event):
        self._status_timer.stop()
        self._wave_timer.stop()
        self._sig_motor_ctrl.emit("off")
        self._sig_disconnect.emit()
        self._thread.quit()
        self._thread.wait(4000)
        if self._estop_sock:
            try:
                self._estop_sock.close()
            except Exception:
                pass
        event.accept()


# ============================================================
# 主入口
# ============================================================

def main():
    _setup_matplotlib_fonts()
    app = QApplication(sys.argv)
    _setup_qt_fonts()

    # QSS 全局样式
    app.setStyleSheet("""
        QMainWindow { background: #F5F5F5; }
        QGroupBox {
            font-weight: bold;
            border: 1px solid #BDBDBD;
            border-radius: 4px;
            margin-top: 8px;
            padding-top: 12px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 8px;
            padding: 0 4px;
        }
        QSpinBox, QDoubleSpinBox, QComboBox {
            padding: 2px 4px;
            min-height: 22px;
        }
        QPushButton {
            padding: 4px 10px;
            border-radius: 3px;
        }
        QTableWidget {
            gridline-color: #E0E0E0;
            alternate-background-color: #FAFAFA;
        }
    """)

    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
