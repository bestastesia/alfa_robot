#!/usr/bin/env python3
"""
RMD 电机实时波形监控工具

实时读取并显示电机转速、位置、电流波形。
支持鼠标缩放、平移、一键最优显示、暂停/恢复、数据导出。

依赖: pip install matplotlib numpy

用法:
  python3 rmd_monitor.py --can can1 --id 4,5,6
  python3 rmd_monitor.py --can can0 --id 1 --signals speed,position,current
  python3 rmd_monitor.py --can can0 --id 1 --interval 20 --window 30

键盘快捷键:
  空格    暂停/恢复采集
  a       自动缩放 (最优显示)
  s       保存数据到 CSV
  q       退出
"""

import argparse
import csv
import socket
import struct
import sys
import threading
import time
from collections import deque
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")

# ---- 字体配置：优先使用系统已安装的中文字体 ----
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

_ZH_FONT = None
for _candidate in ["SimHei", "Droid Sans Fallback", "Noto Sans CJK SC",
                    "Noto Sans CJK JP", "WenQuanYi Micro Hei"]:
    if any(f.name == _candidate for f in fm.fontManager.ttflist):
        _ZH_FONT = _candidate
        break

if _ZH_FONT:
    plt.rcParams["font.sans-serif"] = [_ZH_FONT, "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
else:
    print("WARNING: 未找到中文字体，中文将显示为方框")

import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import numpy as np


# ============================================================
# SocketCAN
# ============================================================

CAN_RAW = 1
CAN_FORMAT = "<IB3x8s"


def create_can_socket(interface: str, timeout: float = 0.05) -> socket.socket:
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
    sock.settimeout(timeout)
    sock.bind((interface,))
    return sock


def send_frame(sock: socket.socket, can_id: int, data: bytes) -> None:
    padded = data.ljust(8, b"\x00")
    frame = struct.pack(CAN_FORMAT, can_id, 8, padded)
    sock.send(frame)


def recv_frame(sock: socket.socket):
    raw = sock.recv(16)
    can_id, dlc, data = struct.unpack(CAN_FORMAT, raw)
    can_id &= 0x7FF
    return can_id, data[:dlc]


def read_status2(sock: socket.socket, motor_id: int):
    """0x9C: 温度, 转矩电流, 转速(dps), 编码器"""
    tx_id = 0x140 + motor_id
    rx_id = 0x140 + motor_id
    send_frame(sock, tx_id, bytes([0x9C, 0, 0, 0, 0, 0, 0, 0]))

    for _ in range(10):
        try:
            rid, rdata = recv_frame(sock)
            if rid == rx_id and rdata[0] == 0x9C:
                return {
                    "temperature": struct.unpack_from("<b", rdata, 1)[0],
                    "iq": struct.unpack_from("<h", rdata, 2)[0],
                    "speed_dps": struct.unpack_from("<h", rdata, 4)[0],
                    "encoder": struct.unpack_from("<H", rdata, 6)[0],
                }
        except socket.timeout:
            continue
    return None


def read_multi_turn_angle(sock: socket.socket, motor_id: int):
    """0x92: 多圈角度 (0.01 deg/LSB)"""
    tx_id = 0x140 + motor_id
    rx_id = 0x140 + motor_id
    send_frame(sock, tx_id, bytes([0x92, 0, 0, 0, 0, 0, 0, 0]))

    for _ in range(10):
        try:
            rid, rdata = recv_frame(sock)
            if rid == rx_id and rdata[0] == 0x92:
                return int.from_bytes(rdata[1:8], "little", signed=True) * 0.01
        except socket.timeout:
            continue
    return None


# ============================================================
# 数据采集线程
# ============================================================

class MotorDataCollector:
    def __init__(self, can_interface: str, motor_ids: list, signals: list,
                 max_points: int = 10000, gear_ratio: int = 36):
        self.can_interface = can_interface
        self.motor_ids = motor_ids
        self.signals = signals
        self.max_points = max_points
        self.gear_ratio = gear_ratio

        self.timestamps = deque(maxlen=max_points)
        self.data = {
            mid: {sig: deque(maxlen=max_points) for sig in signals}
            for mid in motor_ids
        }

        self.paused = False
        self.running = True
        self.start_time = time.monotonic()
        self.sample_count = 0
        self.error_count = 0
        self.lock = threading.Lock()
        self.sock = None

        # 预分配快照缓冲区，避免每帧分配
        self._snap_ts = np.zeros(max_points)
        self._snap_data = {
            mid: {sig: np.zeros(max_points) for sig in signals}
            for mid in motor_ids
        }
        self._snap_len = 0

    def start(self, interval_ms: int):
        self.interval = interval_ms / 1000.0
        self.thread = threading.Thread(target=self._collect_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if hasattr(self, "thread") and self.thread.is_alive():
            self.thread.join(timeout=2)

    def toggle_pause(self):
        self.paused = not self.paused
        return self.paused

    def _collect_loop(self):
        try:
            self.sock = create_can_socket(self.can_interface)
        except OSError as e:
            print(f"CAN 打开失败: {e}")
            self.running = False
            return

        while self.running:
            if self.paused:
                time.sleep(0.05)
                continue

            t = time.monotonic() - self.start_time

            with self.lock:
                self.timestamps.append(t)
                for mid in self.motor_ids:
                    sample = self._read_motor(mid)
                    if sample is None:
                        self.error_count += 1
                        for sig in self.signals:
                            self.data[mid][sig].append(float("nan"))
                        continue
                    self.sample_count += 1
                    for sig in self.signals:
                        self.data[mid][sig].append(sample.get(sig, float("nan")))

            time.sleep(self.interval)

        if self.sock:
            self.sock.close()

    def _read_motor(self, motor_id: int):
        s2 = read_status2(self.sock, motor_id)
        if s2 is None:
            return None
        result = {
            "speed": s2["speed_dps"],
            "current": s2["iq"],
            "temperature": s2["temperature"],
            "encoder": s2["encoder"],
        }
        if "position" in self.signals:
            angle = read_multi_turn_angle(self.sock, motor_id)
            result["position"] = (angle / self.gear_ratio) if angle is not None else float("nan")
        return result

    def get_snapshot(self):
        """线程安全获取数据快照，复用预分配缓冲区"""
        with self.lock:
            n = len(self.timestamps)
            if n == 0:
                self._snap_len = 0
                return self._snap_ts[:0], self._snap_data, 0

            # 复用缓冲区，只拷贝有效数据
            for i, v in enumerate(self.timestamps):
                self._snap_ts[i] = v
            for mid in self.motor_ids:
                for sig in self.signals:
                    src = self.data[mid][sig]
                    buf = self._snap_data[mid][sig]
                    for i, v in enumerate(src):
                        buf[i] = v

            self._snap_len = n
            return self._snap_ts, self._snap_data, n

    def export_csv(self, filename: str):
        ts_buf, data_buf, n = self.get_snapshot()
        if n == 0:
            print("无数据可导出")
            return

        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            header = ["time_s"]
            for mid in self.motor_ids:
                for sig in self.signals:
                    header.append(f"motor{mid}_{sig}")
            writer.writerow(header)

            for i in range(n):
                row = [f"{ts_buf[i]:.4f}"]
                for mid in self.motor_ids:
                    for sig in self.signals:
                        v = data_buf[mid][sig][i]
                        row.append(f"{v:.2f}" if np.isfinite(v) else "")
                writer.writerow(row)

        print(f"数据已导出: {filename} ({n} 条)")


# ============================================================
# 信号配置
# ============================================================

SIGNAL_CONFIG = {
    "speed": {
        "label": "转速 (dps)",
        "color_cycle": ["#2196F3", "#1565C0", "#42A5F5", "#0D47A1"],
    },
    "position": {
        "label": "位置 (deg, 输出侧)",
        "color_cycle": ["#4CAF50", "#2E7D32", "#66BB6A", "#1B5E20"],
    },
    "current": {
        "label": "转矩电流 (raw)",
        "color_cycle": ["#FF5722", "#D84315", "#FF8A65", "#BF360C"],
    },
    "temperature": {
        "label": "温度 (C)",
        "color_cycle": ["#9C27B0", "#6A1B9A", "#BA68C8", "#4A148C"],
    },
}


# ============================================================
# 实时绑图
# ============================================================

class RealtimePlotter:
    def __init__(self, collector: MotorDataCollector, window_sec: float = 10.0):
        self.collector = collector
        self.window_sec = window_sec
        self.auto_scroll = True
        self._y_update_counter = 0  # Y 轴缩放节流计数

        signals = collector.signals
        motor_ids = collector.motor_ids
        n_plots = len(signals)

        # 创建图形
        self.fig = plt.figure(figsize=(14, 3.5 * n_plots + 1.2), facecolor="#FAFAFA")
        self.fig.canvas.manager.set_window_title("RMD Motor Monitor")

        gs = gridspec.GridSpec(n_plots + 1, 1, height_ratios=[1] * n_plots + [0.08],
                               hspace=0.35, left=0.08, right=0.95, top=0.93, bottom=0.05)

        self.axes = []
        self.lines = {}

        for i, sig in enumerate(signals):
            ax = self.fig.add_subplot(gs[i])
            cfg = SIGNAL_CONFIG.get(sig, {"label": sig, "color_cycle": ["#333"]})
            ax.set_ylabel(cfg["label"], fontsize=11)
            ax.grid(True, alpha=0.3, linestyle="--")
            ax.set_facecolor("#FFFFFF")
            ax.tick_params(labelsize=9)

            if i < n_plots - 1:
                ax.set_xticklabels([])
            else:
                ax.set_xlabel("时间 (s)", fontsize=10)

            for j, mid in enumerate(motor_ids):
                color = cfg["color_cycle"][j % len(cfg["color_cycle"])]
                lbl = f"Motor {mid}" if len(motor_ids) > 1 else None
                line, = ax.plot([], [], color=color, linewidth=1.2, label=lbl)
                self.lines[(mid, sig)] = line

            if len(motor_ids) > 1:
                ax.legend(loc="upper left", fontsize=8, framealpha=0.7)

            self.axes.append(ax)

        # 标题
        ids_str = ",".join(str(m) for m in motor_ids)
        self.title = self.fig.suptitle(
            f"RMD 电机监控 | CAN: {collector.can_interface} | 电机: {ids_str} | 运行中",
            fontsize=12, fontweight="bold",
        )

        # 底部按钮 (纯文字，不用 emoji)
        btn_gs = gs[n_plots].subgridspec(1, 5, wspace=0.3)

        ax_pause = self.fig.add_subplot(btn_gs[0])
        self.btn_pause = Button(ax_pause, "|| 暂停", color="#E3F2FD", hovercolor="#BBDEFB")
        self.btn_pause.on_clicked(self._on_pause)
        self.btn_pause.label.set_fontsize(10)

        ax_autofit = self.fig.add_subplot(btn_gs[1])
        self.btn_autofit = Button(ax_autofit, "[A] 最优显示", color="#E8F5E9", hovercolor="#C8E6C9")
        self.btn_autofit.on_clicked(self._on_autofit)
        self.btn_autofit.label.set_fontsize(10)

        ax_scroll = self.fig.add_subplot(btn_gs[2])
        self.btn_scroll = Button(ax_scroll, "< 全局视图", color="#FFF3E0", hovercolor="#FFE0B2")
        self.btn_scroll.on_clicked(self._on_toggle_scroll)
        self.btn_scroll.label.set_fontsize(10)

        ax_export = self.fig.add_subplot(btn_gs[3])
        self.btn_export = Button(ax_export, "[S] 导出CSV", color="#F3E5F5", hovercolor="#E1BEE7")
        self.btn_export.on_clicked(self._on_export)
        self.btn_export.label.set_fontsize(10)

        ax_clear = self.fig.add_subplot(btn_gs[4])
        self.btn_clear = Button(ax_clear, "[X] 清空", color="#FFEBEE", hovercolor="#FFCDD2")
        self.btn_clear.on_clicked(self._on_clear)
        self.btn_clear.label.set_fontsize(10)

        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

        self.status_text = self.fig.text(
            0.5, 0.005, "", fontsize=8, ha="center", color="#666",
        )

    # ---- 回调 ----

    def _on_pause(self, event=None):
        paused = self.collector.toggle_pause()
        self.btn_pause.label.set_text("> 恢复" if paused else "|| 暂停")
        state = "已暂停" if paused else "运行中"
        ids_str = ",".join(str(m) for m in self.collector.motor_ids)
        self.title.set_text(
            f"RMD 电机监控 | CAN: {self.collector.can_interface} | 电机: {ids_str} | {state}"
        )

    def _on_autofit(self, event=None):
        ts_buf, data_buf, n = self.collector.get_snapshot()
        if n < 2:
            return
        ts = ts_buf[:n]

        for i, sig in enumerate(self.collector.signals):
            ax = self.axes[i]
            ax.set_xlim(ts[0], ts[-1])
            ymin, ymax = self._calc_ylim(data_buf, sig, n, 0, n)
            ax.set_ylim(ymin, ymax)

        self.auto_scroll = False
        self.btn_scroll.label.set_text("< 全局 *")

    def _on_toggle_scroll(self, event=None):
        self.auto_scroll = not self.auto_scroll
        if self.auto_scroll:
            self.btn_scroll.label.set_text("< 滚动窗口")
        else:
            self.btn_scroll.label.set_text("< 全局 *")

    def _on_export(self, event=None):
        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        ids_str = "_".join(str(m) for m in self.collector.motor_ids)
        self.collector.export_csv(f"motor_{ids_str}_{ts_str}.csv")

    def _on_clear(self, event=None):
        with self.collector.lock:
            self.collector.timestamps.clear()
            for mid in self.collector.motor_ids:
                for sig in self.collector.signals:
                    self.collector.data[mid][sig].clear()
            self.collector.start_time = time.monotonic()
            self.collector.sample_count = 0
            self.collector.error_count = 0

    def _on_key(self, event):
        if event.key == " ":
            self._on_pause()
        elif event.key == "a":
            self._on_autofit()
        elif event.key == "s":
            self._on_export()
        elif event.key == "q":
            self.collector.stop()
            plt.close("all")

    # ---- 计算辅助 ----

    def _calc_ylim(self, data_buf, sig, n, start_idx, end_idx):
        """计算 Y 轴范围，返回 (ymin, ymax)"""
        lo, hi = float("inf"), float("-inf")
        for mid in self.collector.motor_ids:
            arr = data_buf[mid][sig][start_idx:end_idx]
            finite = arr[np.isfinite(arr)]
            if len(finite) > 0:
                lo = min(lo, finite.min())
                hi = max(hi, finite.max())
        if lo > hi:
            return -1.0, 1.0
        margin = max(abs(hi - lo) * 0.1, 1.0)
        return lo - margin, hi + margin

    # ---- 动画更新 ----

    def update(self, frame):
        ts_buf, data_buf, n = self.collector.get_snapshot()
        if n < 2:
            return list(self.lines.values())

        self._y_update_counter += 1
        do_y_update = (self._y_update_counter % 5 == 0)  # 每 5 帧更新一次 Y 轴

        for i, sig in enumerate(self.collector.signals):
            ax = self.axes[i]

            for mid in self.collector.motor_ids:
                self.lines[(mid, sig)].set_data(ts_buf[:n], data_buf[mid][sig][:n])

            if self.auto_scroll:
                t_now = ts_buf[n - 1]
                t_start = max(0.0, t_now - self.window_sec)
                ax.set_xlim(t_start, t_now + self.window_sec * 0.02)

                if do_y_update:
                    # 找到窗口内的起始索引 (二分查找)
                    start_idx = np.searchsorted(ts_buf[:n], t_start)
                    ymin, ymax = self._calc_ylim(data_buf, sig, n, start_idx, n)
                    ax.set_ylim(ymin, ymax)

        # 状态栏 (也节流)
        if do_y_update:
            elapsed = ts_buf[n - 1]
            rate = self.collector.sample_count / max(elapsed, 0.01)
            self.status_text.set_text(
                f"采样: {self.collector.sample_count} | "
                f"错误: {self.collector.error_count} | "
                f"速率: {rate:.1f} Hz | "
                f"时间: {elapsed:.1f}s | "
                f"[空格]暂停 [a]最优显示 [s]导出 [q]退出 | 滚轮缩放, 拖拽平移"
            )

        return list(self.lines.values())

    def run(self, interval_ms: int = 50):
        self.anim = FuncAnimation(
            self.fig,
            self.update,
            interval=interval_ms,
            blit=True,
            cache_frame_data=False,
        )

        for ax in self.axes:
            ax.set_navigate(True)
            _enable_scroll_zoom(ax, self)

        plt.show()


def _enable_scroll_zoom(ax, plotter):
    """鼠标滚轮缩放 + 自动关闭 auto_scroll"""
    def on_scroll(event):
        if event.inaxes != ax:
            return

        # 用户手动缩放时关闭自动滚动
        plotter.auto_scroll = False
        plotter.btn_scroll.label.set_text("< 全局 *")

        factor = 0.8 if event.button == "up" else 1.25

        xlim = ax.get_xlim()
        xd = event.xdata
        if xd is not None:
            w = (xlim[1] - xlim[0]) * factor
            r = (xd - xlim[0]) / (xlim[1] - xlim[0])
            ax.set_xlim(xd - w * r, xd + w * (1 - r))

        ylim = ax.get_ylim()
        yd = event.ydata
        if yd is not None:
            h = (ylim[1] - ylim[0]) * factor
            r = (yd - ylim[0]) / (ylim[1] - ylim[0])
            ax.set_ylim(yd - h * r, yd + h * (1 - r))

    ax.figure.canvas.mpl_connect("scroll_event", on_scroll)


# ============================================================
# 主入口
# ============================================================

def parse_ids(s: str) -> list:
    result = []
    for part in s.split(","):
        part = part.strip()
        if "-" in part:
            a, b = part.split("-", 1)
            result.extend(range(int(a), int(b) + 1))
        else:
            result.append(int(part))
    return result


def main():
    parser = argparse.ArgumentParser(
        description="RMD 电机实时波形监控",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
示例:
  %(prog)s --can can0 --id 1
  %(prog)s --can can0 --id 1,2,3 --signals speed,current
  %(prog)s --can can0 --id 1 --signals speed,position --window 20 --interval 30
""",
    )
    parser.add_argument("--can", default="can1", help="CAN 接口 (默认: can1)")
    parser.add_argument("--id", default="4,5,6", help="电机ID, 如 1,2,3 或 1-6")
    parser.add_argument("--signals", default="speed",
                        help="信号: speed,position,current,temperature (默认: speed)")
    parser.add_argument("--interval", type=int, default=20,
                        help="采样间隔 ms (默认: 20)")
    parser.add_argument("--window", type=float, default=10.0,
                        help="显示时间窗口 秒 (默认: 10)")
    parser.add_argument("--max-points", type=int, default=20000,
                        help="最大缓存点数 (默认: 20000)")
    parser.add_argument("--gear-ratio", type=int, default=36,
                        help="减速比 (默认: 36)")

    args = parser.parse_args()

    motor_ids = parse_ids(args.id)
    signals = [s.strip() for s in args.signals.split(",")]

    valid_signals = set(SIGNAL_CONFIG.keys())
    for sig in signals:
        if sig not in valid_signals:
            print(f"未知信号: {sig}, 可选: {', '.join(valid_signals)}")
            sys.exit(1)

    print(f"CAN 接口:   {args.can}")
    print(f"电机 ID:    {motor_ids}")
    print(f"监控信号:   {signals}")
    print(f"采样间隔:   {args.interval} ms ({1000 / args.interval:.0f} Hz)")
    print(f"显示窗口:   {args.window} s")
    if _ZH_FONT:
        print(f"中文字体:   {_ZH_FONT}")
    print()

    collector = MotorDataCollector(
        can_interface=args.can,
        motor_ids=motor_ids,
        signals=signals,
        max_points=args.max_points,
        gear_ratio=args.gear_ratio,
    )

    collector.start(interval_ms=args.interval)
    time.sleep(0.2)
    if not collector.running:
        print("采集线程启动失败，请检查 CAN 接口")
        sys.exit(1)

    plotter = RealtimePlotter(collector, window_sec=args.window)

    try:
        plotter.run(interval_ms=max(args.interval, 33))
    except KeyboardInterrupt:
        pass
    finally:
        collector.stop()
        print(f"\n采集结束: {collector.sample_count} 样本, {collector.error_count} 错误")


if __name__ == "__main__":
    main()
