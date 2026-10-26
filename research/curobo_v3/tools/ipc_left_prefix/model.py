import hashlib
import json
import math
from pathlib import Path


JOINT_NAMES=[f'left_joint{index}' for index in range(1,8)]
MOTOR_SIGNS=[1,1,1,1,1,1,1]
MODEL_LIMITS_DEG=[[-180,180],[-90,120],[-180,180],[-145,145],[-180,180],[-110,110],[-180,180]]


def motor_positions(model_positions):
    return [float(value)*sign for value,sign in zip(model_positions,MOTOR_SIGNS)]


def read_library(path):
    library=json.loads(Path(path).read_text())
    if library['joint_names']!=JOINT_NAMES or library['motor_signs']!=MOTOR_SIGNS:
        raise ValueError('Unsupported joint names or coordinate mapping')
    for record in library['tasks'].values():
        rows=record['model_positions']
        if len(rows)<2 or any(len(row)!=7 for row in rows):
            raise ValueError('Every recorded point must contain exactly seven left joints')
        if record['phases'][-1]!='contact_approach' or len(record['phases'])!=len(rows):
            raise ValueError('Recording must end at contact approach')
        if not set(record['phases'])<={'home','base_pick_reposition','approach','contact_approach'}:
            raise ValueError('Forbidden post-contact phase in recording')
        for row in rows:
            for value,(lower,upper) in zip(row,MODEL_LIMITS_DEG):
                if not math.isfinite(value) or not math.radians(lower)-1e-6<=value<=math.radians(upper)+1e-6:
                    raise ValueError('Recorded joint is outside model limits')
    return library


def export_prefix(source):
    result=json.loads(source.read_text())
    if not result.get('success') or not result.get('contact_roll_policy'):
        raise ValueError('Use a successful recording with the corrected contact roll policy')
    boundary=result['phases'].index('attach')
    phases=result['phases'][:boundary]
    names=result['joint_names']
    indices=[names.index(name) for name in JOINT_NAMES]
    rows=[[float(row[index]) for index in indices] for row in result['frames'][:boundary]]
    if phases[-1]!='contact_approach':
        raise ValueError('Last movement before attach is not contact approach')
    if max(abs(rows[-1][joint]-result['frames'][boundary][index]) for joint,index in enumerate(indices))>1e-6:
        raise ValueError('Recorded prefix does not reach the contact configuration')
    updown=[row[names.index('updown')] for row in result['frames'][:boundary]]
    right=[names.index(f'right_joint{index}') for index in range(1,8)]
    return dict(task=result['task'],source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                model_positions=rows,phases=phases,source_end_frame=boundary-1,
                updown_reference=dict(start_m=updown[0],end_m=updown[-1],minimum_m=min(updown),maximum_m=max(updown)),
                right_reference_max_motion_deg=math.degrees(max(abs(row[index]-result['frames'][0][index]) for row in result['frames'][:boundary] for index in right)),
                end_policy='Before attach; no vacuum, extraction, transport, placement or navigation commands')


def select_path(record,feedback,direction,tolerance_deg=.5):
    rows=[motor_positions(row) for row in record['model_positions']]
    tolerance=math.radians(tolerance_deg)
    if direction=='forward':
        if max(abs(value-reference) for value,reference in zip(feedback,rows[0]))>tolerance:
            raise ValueError('当前左臂不在记录的初始姿态，请先用原控制台调整到初始姿态')
        return [list(feedback)]+rows
    if direction!='home':
        raise ValueError('Only forward and home commands are permitted')
    distances=[max(abs(value-reference) for value,reference in zip(feedback,row)) for row in rows]
    nearest=min(range(len(rows)),key=distances.__getitem__)
    if distances[nearest]>tolerance:
        raise ValueError('当前位置偏离已记录路径，不能自动沿原轨迹返回')
    return [list(feedback)]+rows[nearest::-1]


def compact_path(rows,tolerance=2e-7):
    rows=[list(row) for row in rows]
    if not rows or any(len(row)!=7 or not all(math.isfinite(value) for value in row) for row in rows):
        raise ValueError('Expected finite seven-axis positions')
    keep=[0];anchor=0
    while anchor<len(rows)-1:
        end=anchor+1
        while end+1<len(rows):
            candidate=end+1
            delta=[b-a for a,b in zip(rows[anchor],rows[candidate])]
            length=sum(value*value for value in delta)
            previous=0.
            valid=True
            for index in range(anchor+1,candidate):
                offset=[b-a for a,b in zip(rows[anchor],rows[index])]
                fraction=sum(a*b for a,b in zip(offset,delta))/length if length>1e-20 else 0.
                error=max(abs(a-fraction*b) for a,b in zip(offset,delta))
                if error>tolerance or fraction<previous-1e-8 or fraction>1+1e-8:
                    valid=False;break
                previous=fraction
            if not valid:
                break
            end=candidate
        keep.append(end);anchor=end
    return [rows[index] for index in keep]


def time_parameterize(rows,speed_deg_s=15.,acceleration_deg_s2=30.,jerk_deg_s3=150.,sample_period=.02,max_duration_s=60.,max_points=200):
    if not .1<=speed_deg_s<=15.:
        raise ValueError('Speed must be between 0.1 and 15 deg/s')
    rows=compact_path(rows)
    velocity=math.radians(speed_deg_s)
    acceleration=math.radians(acceleration_deg_s2)
    jerk=math.radians(jerk_deg_s3)
    if acceleration<=0 or jerk<=0 or sample_period<=0:
        raise ValueError('Timing limits must be positive')
    points=[dict(time_s=.1,positions=list(rows[0]),velocities=[0.]*7,accelerations=[0.]*7)]
    elapsed=.1
    for start,end in zip(rows,rows[1:]):
        delta=[target-origin for origin,target in zip(start,end)]
        distance=max(abs(value) for value in delta)
        if distance<1e-10:
            continue
        duration=max(1.875*distance/velocity,math.sqrt(5.773503*distance/acceleration),(60*distance/jerk)**(1/3),sample_period)
        elapsed+=duration
        points.append(dict(time_s=elapsed,positions=list(end),velocities=[0.]*7,accelerations=[0.]*7))
    validate_timing(points,speed_deg_s,acceleration_deg_s2,jerk_deg_s3,max_duration_s,max_points)
    return points


def validate_timing(points,speed_deg_s=15.,acceleration_deg_s2=30.,jerk_deg_s3=150.,max_duration_s=60.,max_points=200):
    if not points or len(points)>max_points:
        raise ValueError(f'轨迹点数超过{max_points}点硬上限，拒绝发送')
    if points[-1]['time_s']>max_duration_s or not math.isfinite(points[-1]['time_s']):
        raise ValueError(f"预计运行{points[-1]['time_s']:.1f}秒，超过{max_duration_s:.1f}秒硬上限，拒绝发送")
    for point in points:
        if len(point['positions'])!=7 or not all(math.isfinite(value) for value in point['positions']):
            raise ValueError('Invalid position data')
        if point['velocities']!=[0.]*7 or point['accelerations']!=[0.]*7:
            raise ValueError('Compact trajectory requires zero derivatives at each knot')
    for start,end in zip(points,points[1:]):
        duration=end['time_s']-start['time_s']
        if duration<=0:
            raise ValueError('Trajectory times must increase')
        distance=max(abs(b-a) for a,b in zip(start['positions'],end['positions']))
        if 1.875*distance/duration>math.radians(min(speed_deg_s,15.))+1e-9:
            raise ValueError('五次插值峰值速度超限')
        if 5.773503*distance/duration**2>math.radians(acceleration_deg_s2)+1e-9:
            raise ValueError('五次插值峰值加速度超限')
        if 60*distance/duration**3>math.radians(jerk_deg_s3)+1e-9:
            raise ValueError('五次插值峰值加加速度超限')
