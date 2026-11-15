import json
from pathlib import Path

import numpy as np


POLICY = json.loads((Path(__file__).parent / 'coordinate_policy.json').read_text())
EXECUTION_REVISION = POLICY['required_coordinate_revision']
MOTOR_MINUS_MODEL_RAD = np.deg2rad(POLICY['motor_minus_model_deg'])


def calibration_summary():
    correction = -np.asarray(POLICY['motor_minus_model_deg'])
    return ('**当前联调补偿：** ' + POLICY.get('calibration_name', '软件角度校准')
        + '\n\n模型角＝电控角＋以下修正（J1～J7，度）：`'
        + str(np.round(correction, 6).tolist())
        + '`\n\n发送反向换算，模型全零对应电控角（度）：`'
        + str(np.round(POLICY['motor_minus_model_deg'], 6).tolist())
        + '`\n\nTCP仍为Joint7＋Z 70mm，URDF几何不变；J1/J7未新增补偿不代表已标定无误差。')


def feedback_summary(degrees):
    model = np.rad2deg(motor_to_model(np.deg2rad(degrees)))
    return ('\n\n实测电控角J1～J7（度）：`' + str(np.round(degrees, 4).tolist())
        + '`\n\n补偿后模型角（度）：`' + str(np.round(model, 4).tolist()) + '`')


def motor_to_model(values):
    values = np.asarray(values, dtype=float)
    if values.shape[-1:] != (7,) or not np.isfinite(values).all():
        raise ValueError('需要七轴有限数值')
    return values - MOTOR_MINUS_MODEL_RAD


def model_to_motor(values):
    values = np.asarray(values, dtype=float)
    if values.shape[-1:] != (7,) or not np.isfinite(values).all():
        raise ValueError('需要七轴有限数值')
    return values + MOTOR_MINUS_MODEL_RAD
