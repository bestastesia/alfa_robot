#!/usr/bin/env python3
"""Modbus trajectory-queue test client for the dual-arm PLC V0.2 interface.

This file is intentionally independent from ROS.  It talks directly to the PLC
Holding Register map documented in SEV-7:

  Modbus双臂轨迹接口 V0.2-工程对齐版

The goal is to validate prepare/write/commit/start and basic 12-axis
synchronised trajectory execution before the same protocol is moved into the
ROS PLC bridge.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Sequence


AXIS_COUNT = 12
POINTS_PER_PACKET = 5
REGS_PER_AXIS = 2
REGS_PER_POINT = AXIS_COUNT * REGS_PER_AXIS
POINT_RAW_REG_COUNT = POINTS_PER_PACKET * REGS_PER_POINT
MAX_POINTS = 240

DEFAULT_HOST = "192.168.1.88"
DEFAULT_PORT = 502
DEFAULT_UNIT_ID = 1

REG_PREPARE_TOGGLE = 0
REG_WRITE_TOGGLE = 1
REG_COMMIT_TOGGLE = 2
REG_START_TOGGLE = 3
REG_STOP_TOGGLE = 4
REG_RESET_TOGGLE = 5
REG_TARGET_BANK = 6

REG_PLC_READY = 100
REG_MOTION_READY = 101
REG_RECEIVING = 102
REG_RUNNING = 103
REG_DONE = 104
REG_FAULT = 105
REG_ERROR_CODE = 106
REG_ACTIVE_BANK = 107
REG_WRITE_BANK = 108
REG_BANK_A_READY = 109
REG_BANK_B_READY = 110
REG_ACK_PREPARE = 111
REG_ACK_WRITE = 112
REG_ACK_COMMIT = 113
REG_ACK_START = 114
REG_ACK_JOB_ID_H = 115
REG_ACK_JOB_ID_L = 116
REG_ACK_PACKET_SEQ_H = 117
REG_ACK_PACKET_SEQ_L = 118
REG_LAST_ERROR_PACKET_SEQ_H = 119
REG_LAST_ERROR_PACKET_SEQ_L = 120
REG_RECEIVED_POINT_COUNT = 121
REG_TOTAL_POINT_COUNT_STS = 122
REG_EXECUTED_LINE = 123

REG_HEADER_BASE = 200
REG_POINT_RAW_BASE = 240
HEADER_REG_COUNT = 11


ERROR_NAMES = {
    0: "NO_ERROR",
    1: "NOT_PREPARED",
    2: "BANK_IS_ACTIVE",
    3: "JOB_ID_MISMATCH",
    4: "BANK_MISMATCH",
    5: "POINT_COUNT_INVALID",
    6: "PACKET_RANGE_INVALID",
    8: "DUPLICATE_CONFLICT",
    9: "MISSING_POINT",
    15: "PTHC_ERROR",
}


class PlcQueueError(RuntimeError):
    pass


class ModbusFunction(IntEnum):
    READ_HOLDING_REGISTERS = 3
    WRITE_SINGLE_REGISTER = 6
    WRITE_MULTIPLE_REGISTERS = 16


@dataclass(frozen=True)
class QueueStatus:
    plc_ready: int
    motion_ready: int
    receiving: int
    running: int
    done: int
    fault: int
    error_code: int
    active_bank: int
    write_bank: int
    bank_a_ready: int
    bank_b_ready: int
    ack_prepare: int
    ack_write: int
    ack_commit: int
    ack_start: int
    ack_job_id: int
    ack_packet_seq: int
    last_error_packet_seq: int
    received_point_count: int
    total_point_count: int
    executed_line: int

    @property
    def error_name(self) -> str:
        return ERROR_NAMES.get(self.error_code, f"UNKNOWN_{self.error_code}")

    def bank_ready(self, bank: int) -> int:
        return self.bank_a_ready if bank == 0 else self.bank_b_ready

    def compact(self) -> str:
        return (
            f"ready={self.plc_ready} motionReady={self.motion_ready} "
            f"receiving={self.receiving} running={self.running} done={self.done} "
            f"fault={self.fault} err={self.error_code}({self.error_name}) "
            f"activeBank={self.active_bank} writeBank={self.write_bank} "
            f"bankAReady={self.bank_a_ready} bankBReady={self.bank_b_ready} "
            f"ackPrep={self.ack_prepare} ackWrite={self.ack_write} "
            f"ackCommit={self.ack_commit} ackStart={self.ack_start} "
            f"ackJob={self.ack_job_id} ackPkt={self.ack_packet_seq} "
            f"recv={self.received_point_count}/{self.total_point_count} "
            f"line={self.executed_line}"
        )


class ModbusTcpClient:
    def __init__(self, host: str, port: int, unit_id: int, timeout_s: float):
        self.host = host
        self.port = port
        self.unit_id = unit_id
        self.timeout_s = timeout_s
        self._transaction_id = 0
        self._sock: socket.socket | None = None

    def __enter__(self) -> "ModbusTcpClient":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> None:
        if self._sock is not None:
            return
        self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        self._sock.settimeout(self.timeout_s)

    def close(self) -> None:
        if self._sock is None:
            return
        self._sock.close()
        self._sock = None

    def read_holding_registers(self, address: int, count: int) -> list[int]:
        if count < 1:
            raise ValueError("count must be positive")
        pdu = struct.pack(">BHH", ModbusFunction.READ_HOLDING_REGISTERS, address, count)
        response = self._request(pdu)
        byte_count = response[1]
        expected = count * 2
        if byte_count != expected:
            raise PlcQueueError(f"unexpected Modbus byte count at {address}: got {byte_count}, expected {expected}")
        return list(struct.unpack(">" + "H" * count, response[2 : 2 + byte_count]))

    def write_register(self, address: int, value: int) -> None:
        self._request(struct.pack(">BHH", ModbusFunction.WRITE_SINGLE_REGISTER, address, value & 0xFFFF))

    def write_registers(self, address: int, values: Sequence[int]) -> None:
        if not values:
            raise ValueError("values must not be empty")
        words = [value & 0xFFFF for value in values]
        pdu = struct.pack(">BHHB", ModbusFunction.WRITE_MULTIPLE_REGISTERS, address, len(words), len(words) * 2)
        pdu += struct.pack(">" + "H" * len(words), *words)
        self._request(pdu)

    def _request(self, pdu: bytes) -> bytes:
        self.open()
        assert self._sock is not None
        self._transaction_id = (self._transaction_id + 1) & 0xFFFF
        if self._transaction_id == 0:
            self._transaction_id = 1
        packet = struct.pack(">HHHB", self._transaction_id, 0, len(pdu) + 1, self.unit_id) + pdu
        self._sock.sendall(packet)
        response = self._sock.recv(4096)
        if len(response) < 8:
            raise PlcQueueError(f"short Modbus response: {response.hex()}")
        rx_transaction_id, protocol_id, length = struct.unpack(">HHH", response[:6])
        if rx_transaction_id != self._transaction_id:
            raise PlcQueueError(f"transaction mismatch: got {rx_transaction_id}, expected {self._transaction_id}")
        if protocol_id != 0:
            raise PlcQueueError(f"unexpected Modbus protocol id: {protocol_id}")
        if len(response) < 6 + length:
            raise PlcQueueError(f"incomplete Modbus response: got {len(response)} bytes, expected {6 + length}")
        unit_id = response[6]
        if unit_id != self.unit_id:
            raise PlcQueueError(f"unit id mismatch: got {unit_id}, expected {self.unit_id}")
        function = response[7]
        if function & 0x80:
            code = response[8] if len(response) > 8 else None
            raise PlcQueueError(f"Modbus exception code={code} raw={response.hex()}")
        return response[7 : 6 + length]


def split_udint(value: int) -> list[int]:
    value &= 0xFFFFFFFF
    return [(value >> 16) & 0xFFFF, value & 0xFFFF]


def join_udint(high: int, low: int) -> int:
    return ((high & 0xFFFF) << 16) | (low & 0xFFFF)


def dint_to_regs(value: int) -> list[int]:
    value &= 0xFFFFFFFF
    return [(value >> 16) & 0xFFFF, value & 0xFFFF]


def deg_to_mdeg(value_deg: float) -> int:
    return int(round(value_deg * 1000.0))


def mdeg_to_deg(value_mdeg: int) -> float:
    return value_mdeg / 1000.0


def next_toggle(value: int) -> int:
    value = (value + 1) & 0xFFFF
    return 1 if value == 0 else value


def parse_axis_values(text: str, *, default: float = 0.0) -> list[float]:
    values = [default] * AXIS_COUNT
    if not text:
        return values
    parts = [part.strip() for part in text.replace(";", ",").split(",") if part.strip()]
    if len(parts) == AXIS_COUNT and all(":" not in part for part in parts):
        return [float(part) for part in parts]
    for part in parts:
        if ":" not in part:
            raise ValueError(f"expected AXIS:VALUE or 12 comma-separated values, got {part!r}")
        axis_text, value_text = part.split(":", 1)
        axis = int(axis_text)
        if axis < 1 or axis > AXIS_COUNT:
            raise ValueError(f"axis must be 1..12, got {axis}")
        values[axis - 1] = float(value_text)
    return values


def format_point(point: Sequence[float]) -> str:
    return ", ".join(f"A{axis}={value:+.3f}" for axis, value in enumerate(point, start=1))


def max_segment_delta(points: Sequence[Sequence[float]]) -> tuple[float, int, int]:
    best = 0.0
    best_segment = 0
    best_axis = 0
    for segment_index, (left, right) in enumerate(zip(points, points[1:]), start=1):
        for axis, (start, end) in enumerate(zip(left, right), start=1):
            delta = abs(end - start)
            if delta > best:
                best = delta
                best_segment = segment_index
                best_axis = axis
    return best, best_segment, best_axis


def pack_point(point_deg: Sequence[float]) -> list[int]:
    if len(point_deg) != AXIS_COUNT:
        raise ValueError(f"point must have {AXIS_COUNT} axes")
    registers: list[int] = []
    for value_deg in point_deg:
        registers.extend(dint_to_regs(deg_to_mdeg(value_deg)))
    return registers


def pack_packet_points(points_deg: Sequence[Sequence[float]]) -> list[int]:
    if len(points_deg) > POINTS_PER_PACKET:
        raise ValueError(f"packet supports at most {POINTS_PER_PACKET} points")
    registers: list[int] = []
    for point in points_deg:
        registers.extend(pack_point(point))
    registers.extend([0] * (POINT_RAW_REG_COUNT - len(registers)))
    return registers


def make_header(
    *,
    job_id: int,
    packet_seq: int,
    first_point_seq: int,
    packet_point_count: int,
    total_point_count: int,
    cycle_ms: int,
    target_bank: int,
    crc32: int = 0,
) -> list[int]:
    return [
        *split_udint(job_id),
        *split_udint(packet_seq),
        first_point_seq & 0xFFFF,
        packet_point_count & 0xFFFF,
        total_point_count & 0xFFFF,
        cycle_ms & 0xFFFF,
        *split_udint(crc32),
        target_bank & 0xFFFF,
    ]


class TrajectoryQueueClient:
    def __init__(self, modbus: ModbusTcpClient, *, poll_interval_s: float = 0.05, require_ack_job_id: bool = False):
        self.modbus = modbus
        self.poll_interval_s = poll_interval_s
        self.require_ack_job_id = require_ack_job_id

    def read_status(self) -> QueueStatus:
        words = self.modbus.read_holding_registers(REG_PLC_READY, 24)
        return QueueStatus(
            plc_ready=words[0],
            motion_ready=words[1],
            receiving=words[2],
            running=words[3],
            done=words[4],
            fault=words[5],
            error_code=words[6],
            active_bank=words[7],
            write_bank=words[8],
            bank_a_ready=words[9],
            bank_b_ready=words[10],
            ack_prepare=words[11],
            ack_write=words[12],
            ack_commit=words[13],
            ack_start=words[14],
            ack_job_id=join_udint(words[15], words[16]),
            ack_packet_seq=join_udint(words[17], words[18]),
            last_error_packet_seq=join_udint(words[19], words[20]),
            received_point_count=words[21],
            total_point_count=words[22],
            executed_line=words[23],
        )

    def choose_target_bank(self) -> int:
        status = self.read_status()
        return 1 if status.active_bank == 0 else 0

    def upload_and_start(
        self,
        points_deg: Sequence[Sequence[float]],
        *,
        job_id: int,
        cycle_ms: int,
        target_bank: int | None = None,
        start: bool,
        timeout_s: float,
        verbose: bool,
        commit: bool = True,
    ) -> QueueStatus:
        self._validate_points(points_deg, cycle_ms)
        status = self.read_status()
        if status.fault:
            raise PlcQueueError(f"PLC fault before upload: {status.compact()}")
        if status.running:
            raise PlcQueueError(f"PLC is running; refusing upload/start: {status.compact()}")
        if target_bank is None:
            target_bank = 1 if status.active_bank == 0 else 0
        if target_bank not in (0, 1):
            raise ValueError("target_bank must be 0 or 1")
        if verbose:
            print(f"Initial status: {status.compact()}")
            print(f"Selected targetBank={target_bank}, jobId={job_id}, points={len(points_deg)}, cycleMs={cycle_ms}")

        self.prepare(job_id=job_id, total_point_count=len(points_deg), cycle_ms=cycle_ms, target_bank=target_bank, timeout_s=timeout_s)
        if verbose:
            print("prepare ACK ok")

        for packet_index, first in enumerate(range(0, len(points_deg), POINTS_PER_PACKET), start=0):
            packet = points_deg[first : first + POINTS_PER_PACKET]
            packet_seq = packet_index
            self.write_packet(
                packet,
                job_id=job_id,
                packet_seq=packet_seq,
                first_point_seq=first,
                total_point_count=len(points_deg),
                cycle_ms=cycle_ms,
                target_bank=target_bank,
                timeout_s=timeout_s,
            )
            if verbose:
                status = self.read_status()
                print(
                    f"packet {packet_index:02d} ACK ok: first={first}, count={len(packet)}, "
                    f"received={status.received_point_count}/{len(points_deg)}"
                )

        if not commit:
            return self.read_status()

        self.commit(target_bank=target_bank, timeout_s=timeout_s)
        status = self.read_status()
        if verbose:
            print(f"commit ACK ok: {status.compact()}")
        if not start:
            return status

        start_value = self.start(target_bank=target_bank, timeout_s=timeout_s)
        if verbose:
            print("start ACK ok; monitoring execution...")
        return self.wait_done(
            expected_start_toggle=start_value,
            timeout_s=max(timeout_s, len(points_deg) * cycle_ms / 1000.0 + 10.0),
            verbose=verbose,
        )

    def prepare(self, *, job_id: int, total_point_count: int, cycle_ms: int, target_bank: int, timeout_s: float) -> None:
        self.modbus.write_register(REG_TARGET_BANK, target_bank)
        header = make_header(
            job_id=job_id,
            packet_seq=0,
            first_point_seq=0,
            packet_point_count=0,
            total_point_count=total_point_count,
            cycle_ms=cycle_ms,
            target_bank=target_bank,
        )
        self.modbus.write_registers(REG_HEADER_BASE, header)
        prepare_value = next_toggle(self.modbus.read_holding_registers(REG_PREPARE_TOGGLE, 1)[0])
        self.modbus.write_register(REG_PREPARE_TOGGLE, prepare_value)
        self._wait_for(
            lambda status: status.ack_prepare == prepare_value and status.error_code == 0,
            timeout_s,
            f"ackPrepareToggle={prepare_value}",
        )

    def write_packet(
        self,
        packet_points_deg: Sequence[Sequence[float]],
        *,
        job_id: int,
        packet_seq: int,
        first_point_seq: int,
        total_point_count: int,
        cycle_ms: int,
        target_bank: int,
        timeout_s: float,
    ) -> None:
        header = make_header(
            job_id=job_id,
            packet_seq=packet_seq,
            first_point_seq=first_point_seq,
            packet_point_count=len(packet_points_deg),
            total_point_count=total_point_count,
            cycle_ms=cycle_ms,
            target_bank=target_bank,
            crc32=0,
        )
        self.modbus.write_registers(REG_HEADER_BASE, header)
        self.modbus.write_registers(REG_POINT_RAW_BASE, pack_packet_points(packet_points_deg))
        write_value = next_toggle(self.modbus.read_holding_registers(REG_WRITE_TOGGLE, 1)[0])
        self.modbus.write_register(REG_WRITE_TOGGLE, write_value)
        self._wait_for(
            lambda status: (
                status.ack_write == write_value
                and (not self.require_ack_job_id or status.ack_job_id == job_id)
                and status.ack_packet_seq == packet_seq
                and status.received_point_count >= first_point_seq + len(packet_points_deg)
                and status.error_code == 0
            ),
            timeout_s,
            f"ackWriteToggle={write_value}, ackJobId={job_id}, ackPacketSeq={packet_seq}",
        )

    def commit(self, *, target_bank: int, timeout_s: float) -> None:
        commit_value = next_toggle(self.modbus.read_holding_registers(REG_COMMIT_TOGGLE, 1)[0])
        self.modbus.write_register(REG_COMMIT_TOGGLE, commit_value)
        self._wait_for(
            lambda status: (
                status.ack_commit == commit_value
                and status.motion_ready == 1
                and status.bank_ready(target_bank) == 1
                and status.error_code == 0
            ),
            timeout_s,
            f"ackCommitToggle={commit_value}, bankReady[{target_bank}]=1",
        )

    def start(self, *, target_bank: int, timeout_s: float) -> int:
        self.modbus.write_register(REG_TARGET_BANK, target_bank)
        start_value = next_toggle(self.modbus.read_holding_registers(REG_START_TOGGLE, 1)[0])
        self.modbus.write_register(REG_START_TOGGLE, start_value)
        self._wait_for(
            lambda status: status.ack_start == start_value and status.error_code == 0,
            timeout_s,
            f"ackStartToggle={start_value}",
        )
        return start_value

    def wait_done(self, *, expected_start_toggle: int, timeout_s: float, verbose: bool) -> QueueStatus:
        started_at = time.monotonic()
        last_line = None
        while time.monotonic() - started_at <= timeout_s:
            status = self.read_status()
            if status.fault or status.error_code:
                raise PlcQueueError(f"PLC execution fault: {status.compact()}")
            if verbose and status.executed_line != last_line:
                print(f"exec: t={time.monotonic() - started_at:.2f}s {status.compact()}")
                last_line = status.executed_line
            if status.done and status.ack_start == expected_start_toggle:
                return status
            time.sleep(self.poll_interval_s)
        raise PlcQueueError(f"timeout waiting for done: {self.read_status().compact()}")

    def reset(self) -> None:
        reset_value = next_toggle(self.modbus.read_holding_registers(REG_RESET_TOGGLE, 1)[0])
        self.modbus.write_register(REG_RESET_TOGGLE, reset_value)

    def stop(self) -> None:
        stop_value = next_toggle(self.modbus.read_holding_registers(REG_STOP_TOGGLE, 1)[0])
        self.modbus.write_register(REG_STOP_TOGGLE, stop_value)

    def _wait_for(self, predicate, timeout_s: float, description: str) -> QueueStatus:
        started_at = time.monotonic()
        last_status: QueueStatus | None = None
        while time.monotonic() - started_at <= timeout_s:
            status = self.read_status()
            last_status = status
            if status.error_code:
                raise PlcQueueError(f"PLC error while waiting for {description}: {status.compact()}")
            if status.fault:
                raise PlcQueueError(f"PLC fault while waiting for {description}: {status.compact()}")
            if predicate(status):
                return status
            time.sleep(self.poll_interval_s)
        detail = last_status.compact() if last_status is not None else "no status"
        raise PlcQueueError(f"timeout waiting for {description}: {detail}")

    @staticmethod
    def _validate_points(points_deg: Sequence[Sequence[float]], cycle_ms: int) -> None:
        if not points_deg:
            raise ValueError("trajectory must contain at least one point")
        if len(points_deg) > MAX_POINTS:
            raise ValueError(f"trajectory supports at most {MAX_POINTS} points, got {len(points_deg)}")
        if cycle_ms <= 0 or cycle_ms > 60000:
            raise ValueError("cycle_ms must be in 1..60000")
        for index, point in enumerate(points_deg):
            if len(point) != AXIS_COUNT:
                raise ValueError(f"point {index} must have {AXIS_COUNT} axes")
            for value in point:
                if not math.isfinite(value):
                    raise ValueError(f"point {index} contains non-finite value: {value}")


def interpolate(left: Sequence[float], right: Sequence[float], ratio: float) -> list[float]:
    return [start + (end - start) * ratio for start, end in zip(left, right)]


def make_hold_points(base_deg: Sequence[float], count: int) -> list[list[float]]:
    return [list(base_deg) for _ in range(count)]


def make_single_axis_points(base_deg: Sequence[float], *, axis: int, delta_deg: float, count: int) -> list[list[float]]:
    points: list[list[float]] = []
    for index in range(count):
        ratio = index / (count - 1) if count > 1 else 0.0
        point = list(base_deg)
        point[axis - 1] = base_deg[axis - 1] + delta_deg * ratio
        points.append(point)
    return points


def make_sync_wave_points(
    base_deg: Sequence[float],
    *,
    count: int,
    amplitude_deg: float,
    cycles: float,
    phase_step_deg: float,
) -> list[list[float]]:
    points: list[list[float]] = []
    for point_index in range(count):
        ratio = point_index / (count - 1) if count > 1 else 0.0
        point: list[float] = []
        for axis_index, base in enumerate(base_deg):
            phase = math.radians(phase_step_deg * axis_index)
            value = base + amplitude_deg * math.sin(2.0 * math.pi * cycles * ratio + phase)
            point.append(value)
        points.append(point)
    return points


def make_mixed_delta_points(
    base_deg: Sequence[float],
    *,
    count: int,
    small_delta_deg: float,
    large_delta_deg: float,
) -> list[list[float]]:
    """Generate a 12-axis trajectory with deliberately uneven segment/axis deltas.

    This tests two things:
    1. Some consecutive trajectory points are close and some are far apart.
    2. In the same segment, some axes move a lot while others barely move.

    The profile remains continuous: waypoints are interpolated, not jumped.
    """

    waypoint_offsets = [
        [0.0] * AXIS_COUNT,
        [
            large_delta_deg,
            small_delta_deg,
            -large_delta_deg * 0.65,
            small_delta_deg * 0.5,
            large_delta_deg * 0.45,
            -small_delta_deg,
            -large_delta_deg * 0.35,
            small_delta_deg * 1.4,
            large_delta_deg * 0.25,
            -small_delta_deg * 0.8,
            large_delta_deg * 0.15,
            small_delta_deg * 0.2,
        ],
        [
            large_delta_deg * 0.15,
            -large_delta_deg * 0.45,
            -small_delta_deg,
            large_delta_deg * 0.75,
            small_delta_deg * 0.6,
            large_delta_deg * 0.35,
            -small_delta_deg * 0.3,
            -large_delta_deg * 0.7,
            small_delta_deg,
            large_delta_deg * 0.55,
            -large_delta_deg * 0.25,
            small_delta_deg * 0.4,
        ],
        [
            -small_delta_deg,
            -small_delta_deg * 0.5,
            small_delta_deg * 0.3,
            -small_delta_deg,
            small_delta_deg,
            -large_delta_deg * 0.45,
            large_delta_deg * 0.55,
            -small_delta_deg,
            -large_delta_deg * 0.5,
            small_delta_deg * 0.7,
            large_delta_deg * 0.35,
            -small_delta_deg * 0.4,
        ],
        [0.0] * AXIS_COUNT,
    ]
    waypoints = [[base + offset for base, offset in zip(base_deg, offsets)] for offsets in waypoint_offsets]
    segment_count = len(waypoints) - 1
    points: list[list[float]] = []
    for index in range(count):
        global_ratio = index / (count - 1) if count > 1 else 0.0
        segment_float = global_ratio * segment_count
        segment = min(int(segment_float), segment_count - 1)
        local_ratio = segment_float - segment
        points.append(interpolate(waypoints[segment], waypoints[segment + 1], local_ratio))
    return points


def load_csv_points(path: str) -> list[list[float]]:
    points: list[list[float]] = []
    with open(path, "r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            values = [float(part.strip()) for part in stripped.split(",") if part.strip()]
            if len(values) != AXIS_COUNT:
                raise ValueError(f"{path}:{line_number}: expected {AXIS_COUNT} comma-separated values, got {len(values)}")
            points.append(values)
    return points


def write_csv_points(path: str, points: Sequence[Sequence[float]]) -> None:
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("# A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12 in deg\n")
        for point in points:
            stream.write(",".join(f"{value:.6f}" for value in point) + "\n")


def build_points(args: argparse.Namespace) -> list[list[float]]:
    base = parse_axis_values(args.base)
    if args.csv:
        return load_csv_points(args.csv)
    if args.scenario == "hold":
        return make_hold_points(base, args.points)
    if args.scenario == "single-axis":
        return make_single_axis_points(base, axis=args.axis, delta_deg=args.delta, count=args.points)
    if args.scenario == "sync-wave":
        return make_sync_wave_points(
            base,
            count=args.points,
            amplitude_deg=args.amplitude,
            cycles=args.cycles,
            phase_step_deg=args.phase_step,
        )
    if args.scenario == "mixed-delta":
        return make_mixed_delta_points(
            base,
            count=args.points,
            small_delta_deg=args.small_delta,
            large_delta_deg=args.large_delta,
        )
    raise ValueError(f"unsupported scenario: {args.scenario}")


def print_plan(points: Sequence[Sequence[float]], cycle_ms: int) -> None:
    duration_s = max(0.0, (len(points) - 1) * cycle_ms / 1000.0)
    max_delta, segment, axis = max_segment_delta(points)
    print(f"Trajectory: points={len(points)}, cycleMs={cycle_ms}, duration≈{duration_s:.3f}s")
    print(f"First point: {format_point(points[0])}")
    print(f"Last  point: {format_point(points[-1])}")
    print(f"Max point-to-point axis delta: {max_delta:.3f} deg at segment {segment}, axis {axis}")


def confirm_or_exit(args: argparse.Namespace, message: str) -> None:
    if args.yes_write:
        return
    print(message)
    input("确认机械安全、人员远离、可以写 PLC 后按 Enter；Ctrl+C 取消...")


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Test PLC V0.2 Modbus trajectory queue protocol without ROS.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--unit-id", type=int, default=DEFAULT_UNIT_ID)
    parser.add_argument("--timeout-s", type=float, default=3.0)
    parser.add_argument("--poll-s", type=float, default=0.05)
    parser.add_argument("--job-id", type=int, default=None, help="Defaults to current epoch seconds & 0xffffffff")
    parser.add_argument("--cycle-ms", type=int, default=100)
    parser.add_argument("--target-bank", type=int, choices=[0, 1], default=None)
    parser.add_argument(
        "--base",
        default="",
        help=(
            "Safe current 12-axis base positions in deg. Use either 12 comma-separated values "
            "or AXIS:VALUE pairs. Unspecified axes default to 0; for real motion, fill all 12 axes."
        ),
    )
    parser.add_argument("--points", type=int, default=20)
    parser.add_argument("--csv", default="", help="Load explicit 12-axis points from CSV, one point per line in deg")
    parser.add_argument("--export-csv", default="", help="Write generated points to CSV and exit unless --upload is set")
    parser.add_argument("--scenario", choices=["hold", "single-axis", "sync-wave", "mixed-delta"], default="hold")
    parser.add_argument("--axis", type=int, default=1, help="Axis for single-axis scenario")
    parser.add_argument("--delta", type=float, default=0.3, help="Final delta deg for single-axis scenario")
    parser.add_argument("--amplitude", type=float, default=0.2, help="Amplitude deg for sync-wave scenario")
    parser.add_argument("--cycles", type=float, default=1.0, help="Wave cycles for sync-wave scenario")
    parser.add_argument("--phase-step", type=float, default=15.0, help="Per-axis phase shift deg for sync-wave scenario")
    parser.add_argument("--small-delta", type=float, default=0.1, help="Small axis delta deg for mixed-delta scenario")
    parser.add_argument("--large-delta", type=float, default=0.6, help="Large axis delta deg for mixed-delta scenario")
    parser.add_argument("--status", action="store_true", help="Only read and print queue status")
    parser.add_argument("--reset", action="store_true", help="Send resetToggle before other actions")
    parser.add_argument("--stop", action="store_true", help="Send stopToggle before other actions")
    parser.add_argument("--upload", action="store_true", help="Upload trajectory to PLC")
    parser.add_argument("--write-only", action="store_true", help="Upload packets after prepare, but do not commit or start")
    parser.add_argument("--commit-only", action="store_true", help="Upload and commit but do not start")
    parser.add_argument("--commit-existing", action="store_true", help="Only trigger commit for an already received bank")
    parser.add_argument("--start-existing", action="store_true", help="Only trigger start for an already committed bank")
    parser.add_argument("--require-ack-job-id", action="store_true", help="Also require ackJobId to match job id when waiting packet ACK")
    parser.add_argument("--yes-write", action="store_true", help="Do not prompt before writing PLC")
    parser.add_argument("--verbose", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = make_parser().parse_args(argv)
    if args.points < 1 or args.points > MAX_POINTS:
        raise ValueError(f"--points must be 1..{MAX_POINTS}")
    if args.axis < 1 or args.axis > AXIS_COUNT:
        raise ValueError("--axis must be 1..12")
    write_like = args.upload
    if write_like and not args.csv and not args.base:
        raise ValueError(
            "--upload requires --base with all 12 current safe axis angles, unless --csv provides explicit points"
        )
    if args.write_only and not args.upload:
        raise ValueError("--write-only requires --upload")
    if args.commit_only and not args.upload:
        raise ValueError("--commit-only requires --upload")
    if sum(bool(flag) for flag in (args.commit_only, args.write_only)) > 1:
        raise ValueError("--commit-only and --write-only are mutually exclusive")

    points: list[list[float]] = []
    if args.upload or args.export_csv or not (args.status or args.reset or args.stop or args.commit_existing or args.start_existing):
        points = build_points(args)
        print_plan(points, args.cycle_ms)
        if args.export_csv:
            write_csv_points(args.export_csv, points)
            print(f"Wrote CSV: {args.export_csv}")
            if not args.upload:
                return 0

    if not (args.status or args.reset or args.stop or args.upload or args.commit_existing or args.start_existing):
        print("未指定 --upload/--status/--reset/--stop，只生成/展示轨迹，不连接 PLC。")
        return 0
    job_id = args.job_id if args.job_id is not None else int(time.time()) & 0xFFFFFFFF

    with ModbusTcpClient(args.host, args.port, args.unit_id, args.timeout_s) as modbus:
        client = TrajectoryQueueClient(
            modbus,
            poll_interval_s=args.poll_s,
            require_ack_job_id=args.require_ack_job_id,
        )
        if args.status:
            print(client.read_status().compact())
            return 0
        if args.reset:
            confirm_or_exit(args, "即将写 resetToggle。")
            client.reset()
            print("resetToggle sent")
        if args.stop:
            confirm_or_exit(args, "即将写 stopToggle。")
            client.stop()
            print("stopToggle sent")
        if args.commit_existing:
            target_bank = args.target_bank
            if target_bank is None:
                status = client.read_status()
                target_bank = status.write_bank if status.write_bank in (0, 1) else client.choose_target_bank()
            confirm_or_exit(args, f"即将只触发 commitToggle，targetBank={target_bank}。")
            client.modbus.write_register(REG_TARGET_BANK, target_bank)
            client.commit(target_bank=target_bank, timeout_s=args.timeout_s)
            print(f"Commit existing OK: {client.read_status().compact()}")
            return 0
        if args.start_existing:
            target_bank = args.target_bank
            if target_bank is None:
                status = client.read_status()
                if status.bank_a_ready and not status.bank_b_ready:
                    target_bank = 0
                elif status.bank_b_ready and not status.bank_a_ready:
                    target_bank = 1
                else:
                    target_bank = 1 if status.active_bank == 0 else 0
            confirm_or_exit(args, f"即将只触发 startToggle，targetBank={target_bank}。")
            start_value = client.start(target_bank=target_bank, timeout_s=args.timeout_s)
            final_status = client.wait_done(
                expected_start_toggle=start_value,
                timeout_s=max(args.timeout_s, 10.0),
                verbose=args.verbose,
            )
            print(f"Start existing final status: {final_status.compact()}")
            return 0
        if not args.upload:
            return 0
        confirm_or_exit(
            args,
            (
                f"即将上传轨迹到 PLC: scenario={args.scenario}, points={len(points)}, "
                f"cycleMs={args.cycle_ms}, commit={not args.write_only}, start={not args.commit_only and not args.write_only}"
            ),
        )
        final_status = client.upload_and_start(
            points,
            job_id=job_id,
            cycle_ms=args.cycle_ms,
            target_bank=args.target_bank,
            start=not args.commit_only and not args.write_only,
            timeout_s=args.timeout_s,
            verbose=args.verbose,
            commit=not args.write_only,
        )
        print(f"Final status: {final_status.compact()}")
        return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nCancelled by user", file=sys.stderr)
        raise SystemExit(130)
