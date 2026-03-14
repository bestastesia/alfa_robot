#!/usr/bin/env python3
"""
RMD 电机调试工具 — 基于瓴控科技 CAN 通讯协议 V2.36

支持功能:
  - 读取/写入 PID 参数 (角度环、速度环、电流环)
  - 读取电机状态 (温度、电压、电流、转速、编码器)
  - 读取/写入最大力矩电流限制、最大速度限制
  - 电机运行/停止/关闭控制

协议要点:
  命令帧 CAN ID: 0x140 + motor_id (1~32)
  回复帧 CAN ID: 0x140 + motor_id (1~32)  (注: 协议文档写的 0x180 实测不对)
  DLC: 8 字节, 标准帧

用法:
  python3 rmd_debug.py --can can0 --id 1 read-pid
  python3 rmd_debug.py --can can0 --id 1 write-pid --loop angle --kp 100 --ki 50 --kd 30
  python3 rmd_debug.py --can can0 --id 1 status
"""

import argparse
import socket
import struct
import sys
import time

# ============================================================
# SocketCAN 底层
# ============================================================

CAN_RAW = 1
CAN_FORMAT = "<IB3x8s"  # can_id(4) + dlc(1) + pad(3) + data(8) = 16 bytes
SIOCGIFINDEX = 0x8933


def create_can_socket(interface: str, timeout: float = 0.5) -> socket.socket:
    """创建并绑定 SocketCAN 原始套接字"""
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, CAN_RAW)
    sock.settimeout(timeout)
    sock.bind((interface,))
    return sock


def send_frame(sock: socket.socket, can_id: int, data: bytes) -> None:
    """发送一帧 CAN 数据"""
    dlc = len(data)
    padded = data.ljust(8, b"\x00")
    frame = struct.pack(CAN_FORMAT, can_id, dlc, padded)
    sock.send(frame)


def recv_frame(sock: socket.socket) -> tuple:
    """接收一帧, 返回 (can_id, data_bytes) 或抛出 timeout"""
    raw = sock.recv(16)
    can_id, dlc, data = struct.unpack(CAN_FORMAT, raw)
    can_id &= 0x7FF  # 标准帧 11bit
    return can_id, data[:dlc]


def send_and_recv(
    sock: socket.socket,
    motor_id: int,
    data: bytes,
    expected_cmd: int | None = None,
    retries: int = 5,
) -> bytes | None:
    """发送命令并等待对应电机的回复"""
    tx_id = 0x140 + motor_id
    rx_id = 0x140 + motor_id

    send_frame(sock, tx_id, data)

    for _ in range(retries):
        try:
            rid, rdata = recv_frame(sock)
            if rid == rx_id:
                if expected_cmd is None or rdata[0] == expected_cmd:
                    return rdata
        except socket.timeout:
            continue

    return None


# ============================================================
# 协议命令封装
# ============================================================

# --- 读取控制参数 (0xC0) ---
PARAM_ID_ANGLE_PID = 0x0A
PARAM_ID_SPEED_PID = 0x0B
PARAM_ID_CURRENT_PID = 0x0C
PARAM_ID_TORQUE_LIMIT = 0x1E
PARAM_ID_SPEED_LIMIT = 0x20

LOOP_NAME_MAP = {
    "angle": PARAM_ID_ANGLE_PID,
    "speed": PARAM_ID_SPEED_PID,
    "current": PARAM_ID_CURRENT_PID,
}

LOOP_LABEL = {
    PARAM_ID_ANGLE_PID: "角度环 (angle)",
    PARAM_ID_SPEED_PID: "速度环 (speed)",
    PARAM_ID_CURRENT_PID: "电流环 (current)",
}


def read_control_param(sock: socket.socket, motor_id: int, param_id: int) -> bytes | None:
    """0xC0 读取控制参数, 返回 DATA[2..7] 共 6 字节"""
    cmd = bytes([0xC0, param_id, 0, 0, 0, 0, 0, 0])
    resp = send_and_recv(sock, motor_id, cmd, expected_cmd=0xC0)
    if resp is None:
        return None
    if resp[1] != param_id:
        return None
    return resp[2:8]


def write_control_param(
    sock: socket.socket, motor_id: int, param_id: int, param_bytes: bytes
) -> bytes | None:
    """0xC1 写入控制参数到 RAM, param_bytes 长度 6"""
    payload = bytes([0xC1, param_id]) + param_bytes[:6].ljust(6, b"\x00")
    resp = send_and_recv(sock, motor_id, payload, expected_cmd=0xC1)
    return resp


def parse_pid(raw6: bytes) -> dict:
    """解析 6 字节 PID: Kp(u16) Ki(u16) Kd(u16), 小端"""
    kp = struct.unpack_from("<H", raw6, 0)[0]
    ki = struct.unpack_from("<H", raw6, 2)[0]
    kd = struct.unpack_from("<H", raw6, 4)[0]
    return {"kp": kp, "ki": ki, "kd": kd}


def encode_pid(kp: int, ki: int, kd: int) -> bytes:
    """编码 PID 参数为 6 字节"""
    return struct.pack("<HHH", kp, ki, kd)


# --- 电机状态 (0x9A / 0x9C) ---

def read_status1(sock: socket.socket, motor_id: int) -> dict | None:
    """0x9A 读取电机状态1: 温度、电压、电流、状态、错误"""
    cmd = bytes([0x9A, 0, 0, 0, 0, 0, 0, 0])
    resp = send_and_recv(sock, motor_id, cmd, expected_cmd=0x9A)
    if resp is None:
        return None
    temperature = struct.unpack_from("<b", resp, 1)[0]
    voltage = struct.unpack_from("<H", resp, 2)[0] * 0.01
    current = struct.unpack_from("<h", resp, 4)[0] * 0.01
    motor_state = resp[6]
    error_state = resp[7]
    return {
        "temperature_c": temperature,
        "voltage_v": voltage,
        "current_a": current,
        "motor_state": motor_state,
        "error_state": error_state,
    }


def read_status2(sock: socket.socket, motor_id: int) -> dict | None:
    """0x9C 读取电机状态2: 温度、转矩电流、转速、编码器"""
    cmd = bytes([0x9C, 0, 0, 0, 0, 0, 0, 0])
    resp = send_and_recv(sock, motor_id, cmd, expected_cmd=0x9C)
    if resp is None:
        return None
    temperature = struct.unpack_from("<b", resp, 1)[0]
    iq = struct.unpack_from("<h", resp, 2)[0]
    speed = struct.unpack_from("<h", resp, 4)[0]  # 1 dps/LSB
    encoder = struct.unpack_from("<H", resp, 6)[0]
    return {
        "temperature_c": temperature,
        "iq_raw": iq,
        "speed_dps": speed,
        "encoder": encoder,
    }


# --- 电机控制 ---

def motor_run(sock: socket.socket, motor_id: int) -> bool:
    """0x88 电机运行"""
    cmd = bytes([0x88, 0, 0, 0, 0, 0, 0, 0])
    return send_and_recv(sock, motor_id, cmd) is not None


def motor_stop(sock: socket.socket, motor_id: int) -> bool:
    """0x81 电机停止"""
    cmd = bytes([0x81, 0, 0, 0, 0, 0, 0, 0])
    return send_and_recv(sock, motor_id, cmd) is not None


def motor_off(sock: socket.socket, motor_id: int) -> bool:
    """0x80 电机关闭"""
    cmd = bytes([0x80, 0, 0, 0, 0, 0, 0, 0])
    return send_and_recv(sock, motor_id, cmd) is not None


# --- 限制参数 ---

def read_torque_limit(sock: socket.socket, motor_id: int) -> int | None:
    """读取最大力矩电流限制 (ParamID 0x1E)"""
    raw = read_control_param(sock, motor_id, PARAM_ID_TORQUE_LIMIT)
    if raw is None:
        return None
    return struct.unpack_from("<h", raw, 2)[0]  # Byte3-4


def read_speed_limit(sock: socket.socket, motor_id: int) -> int | None:
    """读取最大速度限制 (ParamID 0x20)"""
    raw = read_control_param(sock, motor_id, PARAM_ID_SPEED_LIMIT)
    if raw is None:
        return None
    return struct.unpack_from("<i", raw, 2)[0]  # Byte3-6


# ============================================================
# 错误标志解析
# ============================================================

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


def format_error_state(error_state: int) -> str:
    if error_state == 0:
        return "正常"
    errors = []
    for bit, desc in ERROR_BITS.items():
        if error_state & (1 << bit):
            errors.append(desc)
    return ", ".join(errors)


# ============================================================
# CLI 子命令
# ============================================================

def cmd_read_pid(sock: socket.socket, motor_id: int, args: argparse.Namespace) -> None:
    """读取所有三环 PID"""
    print(f"=== 电机 {motor_id} PID 参数 ===\n")
    for param_id, label in LOOP_LABEL.items():
        raw = read_control_param(sock, motor_id, param_id)
        if raw is None:
            print(f"  {label}: 读取失败")
            continue
        pid = parse_pid(raw)
        print(f"  {label}:  Kp={pid['kp']}  Ki={pid['ki']}  Kd={pid['kd']}")
    print()


def cmd_write_pid(sock: socket.socket, motor_id: int, args: argparse.Namespace) -> None:
    """写入指定环的 PID (到 RAM, 掉电丢失)"""
    loop_name = args.loop
    param_id = LOOP_NAME_MAP.get(loop_name)
    if param_id is None:
        print(f"未知环路: {loop_name}, 可选: angle, speed, current")
        return

    kp, ki, kd = args.kp, args.ki, args.kd
    label = LOOP_LABEL[param_id]

    # 先读取当前值
    raw = read_control_param(sock, motor_id, param_id)
    if raw is not None:
        old = parse_pid(raw)
        print(f"当前 {label}: Kp={old['kp']}  Ki={old['ki']}  Kd={old['kd']}")

    # 写入新值
    payload = encode_pid(kp, ki, kd)
    resp = write_control_param(sock, motor_id, param_id, payload)
    if resp is None:
        print("写入失败!")
        return

    print(f"写入 {label}: Kp={kp}  Ki={ki}  Kd={kd}  → 成功 (RAM, 掉电丢失)")

    # 回读验证
    time.sleep(0.05)
    raw = read_control_param(sock, motor_id, param_id)
    if raw is not None:
        verify = parse_pid(raw)
        print(f"回读 {label}: Kp={verify['kp']}  Ki={verify['ki']}  Kd={verify['kd']}")
        if verify["kp"] == kp and verify["ki"] == ki and verify["kd"] == kd:
            print("验证通过")
        else:
            print("警告: 回读值与写入值不一致!")


def cmd_status(sock: socket.socket, motor_id: int, args: argparse.Namespace) -> None:
    """读取电机状态"""
    print(f"=== 电机 {motor_id} 状态 ===\n")

    s1 = read_status1(sock, motor_id)
    if s1:
        state_str = "运行" if s1["motor_state"] == 0x00 else "关闭"
        print(f"  温度:     {s1['temperature_c']} °C")
        print(f"  母线电压: {s1['voltage_v']:.2f} V")
        print(f"  母线电流: {s1['current_a']:.2f} A")
        print(f"  电机状态: 0x{s1['motor_state']:02X} ({state_str})")
        print(f"  错误标志: 0x{s1['error_state']:02X} ({format_error_state(s1['error_state'])})")
    else:
        print("  状态1 (0x9A): 读取失败")

    print()

    s2 = read_status2(sock, motor_id)
    if s2:
        print(f"  转矩电流: {s2['iq_raw']}")
        print(f"  转速:     {s2['speed_dps']} dps")
        print(f"  编码器:   {s2['encoder']}")
    else:
        print("  状态2 (0x9C): 读取失败")

    print()

    # 读取限制参数
    tl = read_torque_limit(sock, motor_id)
    sl = read_speed_limit(sock, motor_id)
    if tl is not None:
        print(f"  最大力矩电流限制: {tl}")
    if sl is not None:
        print(f"  最大速度限制:     {sl}")

    print()


def cmd_motor_ctrl(sock: socket.socket, motor_id: int, args: argparse.Namespace) -> None:
    """电机运行/停止/关闭"""
    action = args.action
    funcs = {"run": motor_run, "stop": motor_stop, "off": motor_off}
    labels = {"run": "运行 (0x88)", "stop": "停止 (0x81)", "off": "关闭 (0x80)"}
    ok = funcs[action](sock, motor_id)
    print(f"电机 {motor_id} {labels[action]}: {'成功' if ok else '失败'}")


def cmd_read_all_pid(sock: socket.socket, motor_id: int, args: argparse.Namespace) -> None:
    """批量读取多个电机的 PID"""
    ids = parse_motor_ids(args.ids)
    can_iface = args.can

    print(f"=== 批量读取 PID (CAN: {can_iface}) ===\n")
    print(f"{'电机ID':>6}  {'环路':<16}  {'Kp':>6}  {'Ki':>6}  {'Kd':>6}")
    print("-" * 56)

    for mid in ids:
        for param_id, label in LOOP_LABEL.items():
            raw = read_control_param(sock, mid, param_id)
            if raw is None:
                print(f"{mid:>6}  {label:<16}  {'读取失败':>6}")
                continue
            pid = parse_pid(raw)
            print(f"{mid:>6}  {label:<16}  {pid['kp']:>6}  {pid['ki']:>6}  {pid['kd']:>6}")
        time.sleep(0.01)

    print()


def parse_motor_ids(ids_str: str) -> list:
    """解析电机 ID 字符串, 支持逗号分隔和范围: '1,2,3' 或 '1-6'"""
    result = []
    for part in ids_str.split(","):
        part = part.strip()
        if "-" in part:
            start, end = part.split("-", 1)
            result.extend(range(int(start), int(end) + 1))
        else:
            result.append(int(part))
    return result


# ============================================================
# 主入口
# ============================================================

def main():
    parser = argparse.ArgumentParser(
        description="RMD 电机调试工具 (瓴控科技 CAN 协议 V2.36)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
示例:
  # 读取电机1的PID
  %(prog)s --can can0 --id 1 read-pid

  # 写入角度环 PID (RAM, 掉电丢失)
  %(prog)s --can can0 --id 1 write-pid --loop angle --kp 100 --ki 50 --kd 30

  # 读取电机状态
  %(prog)s --can can0 --id 1 status

  # 批量读取多个电机PID
  %(prog)s --can can0 read-all-pid --ids 1-6

  # 电机控制
  %(prog)s --can can0 --id 1 motor run
  %(prog)s --can can0 --id 1 motor stop
  %(prog)s --can can0 --id 1 motor off
""",
    )
    parser.add_argument("--can", default="can0", help="CAN 接口名 (默认: can0)")
    parser.add_argument("--id", type=int, default=1, help="电机 ID 1~32 (默认: 1)")

    sub = parser.add_subparsers(dest="command", help="子命令")

    # read-pid
    sub.add_parser("read-pid", help="读取三环 PID 参数")

    # write-pid
    wp = sub.add_parser("write-pid", help="写入 PID 参数到 RAM")
    wp.add_argument("--loop", required=True, choices=["angle", "speed", "current"],
                     help="环路: angle(角度环), speed(速度环), current(电流环)")
    wp.add_argument("--kp", type=int, required=True, help="Kp (uint16, 0~65535)")
    wp.add_argument("--ki", type=int, required=True, help="Ki (uint16, 0~65535)")
    wp.add_argument("--kd", type=int, required=True, help="Kd (uint16, 0~65535)")

    # status
    sub.add_parser("status", help="读取电机状态")

    # motor control
    mc = sub.add_parser("motor", help="电机控制 (run/stop/off)")
    mc.add_argument("action", choices=["run", "stop", "off"], help="run=运行, stop=停止, off=关闭")

    # read-all-pid (batch)
    rap = sub.add_parser("read-all-pid", help="批量读取多个电机 PID")
    rap.add_argument("--ids", required=True, help="电机ID列表, 如 '1-6' 或 '1,2,3'")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        sys.exit(1)

    try:
        sock = create_can_socket(args.can)
    except OSError as e:
        print(f"无法打开 CAN 接口 '{args.can}': {e}")
        print("请确认接口已启动: sudo ip link set can0 up type can bitrate 1000000")
        sys.exit(1)

    try:
        dispatch = {
            "read-pid": cmd_read_pid,
            "write-pid": cmd_write_pid,
            "status": cmd_status,
            "motor": cmd_motor_ctrl,
            "read-all-pid": lambda s, m, a: cmd_read_all_pid(s, m, a),
        }
        dispatch[args.command](sock, args.id, args)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
