from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import time
from zoneinfo import ZoneInfo

import numpy as np

from ipc_left_prefix.coordinate_mapping import motor_to_model, EXECUTION_REVISION, POLICY
from probe_touch_ik import ROOT, ProbeTouchIK


def measured_angles(status):
    age = status.get('feedback_age_s')
    values = status.get('feedback_deg')
    if age is None or not 0 <= age <= .5 or values is None:
        raise ValueError('反馈过期，不能记录或使用旧姿态')
    if status.get('busy'):
        raise ValueError('机械臂正在执行，等待停稳再记录')
    if status.get('coordinate_revision') != EXECUTION_REVISION:
        raise ValueError('工控机后端坐标版本不一致，请先同步')
    raw = np.asarray(values, dtype=float)
    if raw.shape != (7,) or not np.isfinite(raw).all():
        raise ValueError('反馈必须包含左7轴有限角度')
    return raw


def record_contact(request_status, solver, path, point_id, target):
    point_id = point_id.strip()
    if not point_id or len(point_id) > 64:
        raise ValueError('固定点编号需要1～64个字符')
    first = request_status()
    before = measured_angles(first)
    clock = time.monotonic()
    time.sleep(.25)
    second = request_status()
    raw = measured_angles(second)
    interval = time.monotonic() - clock
    if interval > 1. or float(np.max(np.abs(raw - before))) / interval > .5:
        raise ValueError('尚未停稳或反馈读取太慢，记录未保存')
    model = motor_to_model(np.deg2rad(raw))
    tip = solver.forward(model)
    joint7 = solver.model.get_transform('left_link7', solver.model.base_link).copy()
    carriage = solver.model.get_transform('arm_carriage', solver.model.base_link).copy()
    urdf = ROOT / 'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf'
    sample = dict(schema_version=1, timestamp=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        point_id=point_id, measurement='user_clicked_record_after_manual_contact; no contact sensor',
        raw_motor_deg=raw.tolist(), model_joint_deg=np.rad2deg(model).tolist(),
        feedback_age_s=second['feedback_age_s'], stability_interval_s=interval,
        observed_max_joint_speed_deg_s=float(np.max(np.abs(raw - before))) / interval,
        coordinate_revision=EXECUTION_REVISION, motor_minus_model_deg=POLICY['motor_minus_model_deg'],
        updown_reference_m=-.3, tcp_in_left_link7_m=[0.,0.,.070], frame_id=solver.model.base_link,
        joint7_in_footprint=joint7.tolist(), carriage_in_footprint=carriage.tolist(),
        nominal_tip_in_footprint=tip.tolist(), requested_target_tcp=np.asarray(target).tolist(),
        urdf_path=str(urdf), urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
        note='FK位置仅为当前模型计算值；真实约束是同point_id样本由人工触碰同一固定点。')
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(sample, ensure_ascii=False) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    return sample
