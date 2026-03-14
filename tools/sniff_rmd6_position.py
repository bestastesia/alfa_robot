#!/usr/bin/env python3
"""
Sniff CAN1 bus for RMD motor ID6 position commands (0xA4),
decode to degrees, log to CSV, and plot waveform.

Usage:
  # Record (Ctrl+C to stop and plot):
  sudo python3 sniff_rmd6_position.py

  # Plot existing CSV:
  python3 sniff_rmd6_position.py --plot rmd6_position_log.csv

  # Custom interface / motor ID:
  sudo python3 sniff_rmd6_position.py --interface can1 --motor-id 6
"""

import argparse
import csv
import os
import signal
import socket
import struct
import sys
import time

# CAN constants
CAN_RAW = 1
CAN_FMT = "=IB3x8s"  # can_id(4) + can_dlc(1) + pad(3) + data(8) = 16 bytes
CAN_FRAME_SIZE = 16
SOL_CAN_RAW = 101
CAN_RAW_FILTER = 1

GEAR_RATIO = 36
RMD_BASE_ID = 0x140
CMD_POSITION = 0xA4


def open_can_socket(interface: str) -> socket.socket:
    """Open a raw CAN socket with hardware filter."""
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
    sock.bind((interface,))
    return sock


def set_can_filter(sock: socket.socket, can_id: int):
    """Set hardware-level CAN filter to only receive specific ID."""
    # struct can_filter { canid_t can_id; canid_t can_mask; }
    filt = struct.pack("=II", can_id, 0x7FF)
    sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, filt)


def decode_position_deg(data: bytes) -> float:
    """Decode 0xA4 position command frame to degrees (output-shaft)."""
    # data[0] = 0xA4 (cmd byte, already in frame from sendMotorCommand)
    # data[1:3] = max_speed_dps (uint16 LE) — we skip this
    # data[3:7] = angle_control (int32 LE) = angle_deg * 100 * GEAR_RATIO
    if len(data) < 7 or data[0] != CMD_POSITION:
        return None
    angle_control = struct.unpack_from("<i", data, 3)[0]
    angle_deg = angle_control / (100.0 * GEAR_RATIO)
    return angle_deg


def record(interface: str, motor_id: int, output_csv: str):
    """Record position commands to CSV until Ctrl+C."""
    can_id = RMD_BASE_ID + motor_id
    sock = open_can_socket(interface)
    set_can_filter(sock, can_id)
    sock.settimeout(1.0)

    records = []
    t0 = time.monotonic()

    print(f"Listening on {interface} for CAN ID 0x{can_id:03X} (motor {motor_id}) ...")
    print("Press Ctrl+C to stop and plot.\n")

    stopped = False

    def on_signal(sig, frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, on_signal)

    try:
        while not stopped:
            try:
                frame = sock.recv(CAN_FRAME_SIZE)
            except socket.timeout:
                continue
            except OSError:
                break

            recv_id, dlc, data = struct.unpack(CAN_FMT, frame)
            recv_id &= 0x7FF
            if recv_id != can_id or dlc < 7:
                continue

            deg = decode_position_deg(data)
            if deg is None:
                continue

            elapsed = time.monotonic() - t0
            records.append((elapsed, deg))
            print(f"\r  t={elapsed:8.3f}s  angle={deg:10.3f}°", end="", flush=True)
    finally:
        sock.close()

    print(f"\n\nCaptured {len(records)} frames.")

    if not records:
        print("No data captured.")
        return

    # Write CSV
    with open(output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "angle_deg"])
        writer.writerows(records)
    print(f"Saved to {output_csv}")

    plot_csv(output_csv)


def plot_csv(csv_path: str):
    """Plot angle waveform from CSV."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed. Install with: pip3 install matplotlib")
        print(f"You can still view the data in {csv_path}")
        return

    times, angles = [], []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row["time_s"]))
            angles.append(float(row["angle_deg"]))

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(times, angles, linewidth=0.8, color="#2196F3")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (°)")
    ax.set_title(f"RMD Motor Position Command — {os.path.basename(csv_path)}")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    png_path = csv_path.replace(".csv", ".png")
    fig.savefig(png_path, dpi=150)
    print(f"Plot saved to {png_path}")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Sniff RMD motor position commands on CAN bus")
    parser.add_argument("--interface", default="can1", help="CAN interface (default: can1)")
    parser.add_argument("--motor-id", type=int, default=6, help="RMD motor ID (default: 6)")
    parser.add_argument("--output", default="rmd6_position_log.csv", help="Output CSV file")
    parser.add_argument("--plot", metavar="CSV", help="Plot existing CSV instead of recording")
    args = parser.parse_args()

    if args.plot:
        plot_csv(args.plot)
    else:
        record(args.interface, args.motor_id, args.output)


if __name__ == "__main__":
    main()
