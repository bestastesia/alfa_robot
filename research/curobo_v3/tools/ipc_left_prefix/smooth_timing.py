import math
import hashlib
import json

import numpy as np
from scipy.interpolate import BPoly,PPoly,make_interp_spline
from scipy.ndimage import gaussian_filter1d


def rt_curves(points):
    times=np.array([point['time_s']for point in points])
    return [PPoly.from_bernstein_basis(BPoly.from_derivatives(times,
        [[point[field][joint]for field in ('positions','velocities','accelerations')]for point in points]))for joint in range(7)]


def exact_peaks(points):
    curves=rt_curves(points)
    times=np.array([point['time_s']for point in points])
    peaks=[]
    for order in (1,2,3):
        maximum=0.
        for curve in curves:
            roots=curve.derivative(order+1).roots(extrapolate=False)
            stamps=np.r_[times,roots[np.isfinite(roots)]]
            maximum=max(maximum,float(np.abs(curve.derivative(order)(stamps)).max()))
        peaks.append(math.degrees(maximum))
    return peaks


def validate_preview(payload,max_duration_s=60.):
    unsigned={key:value for key,value in payload.items()if key!='preview_id'}
    identity=hashlib.sha256(json.dumps(unsigned,sort_keys=True).encode()).hexdigest()
    if payload.get('preview_id')!=identity:
        raise ValueError('预览内容校验失败，禁止发布')
    points=payload['points'];report=payload['report']
    if not report.get('collision_checked')or not report.get('left_task'):
        raise ValueError('该预览没有通过参考碰撞检查，或是仅右臂任务，禁止发布')
    if report.get('collision_passed') is False and not (payload.get('task')=='probe_touch' and report.get('collision_advisory') is True):
        raise ValueError('参考碰撞未通过，且不是已授权仅提示的测试杆任务')
    if not 2<=len(points)<=1000:
        raise ValueError('Action必须2～1000点')
    duration=points[-1]['time_s']
    if duration>max_duration_s or duration<=0:
        raise ValueError(f'轨迹{duration:.1f}秒超过{max_duration_s:.1f}秒，禁止发布')
    if any(abs(point['time_s']-index*.1)>1e-8 for index,point in enumerate(points)):
        raise ValueError('必须严格按0.1秒采样')
    for point in points:
        if any(len(point[field])!=7 or not np.isfinite(point[field]).all()for field in ('positions','velocities','accelerations')):
            raise ValueError('轨迹必须只有左7轴且数值有限')
    if any(np.max(np.abs(points[index][field]))>1e-8 for index in (0,-1)for field in ('velocities','accelerations')):
        raise ValueError('起终点速度/加速度必须为0')
    peaks=exact_peaks(points)
    if any(peak>limit+1e-5 for peak,limit in zip(peaks,(min(15.,report['speed_limit_deg_s']),30.,150.))):
        raise ValueError('RT五次插值峰值复核超限')
    return identity


def smooth_points(rows,speed_deg_s=15.,max_duration_s=60.):
    q=np.asarray(rows,dtype=float)
    if q.ndim!=2 or q.shape[1]!=7 or len(q)<2 or not np.isfinite(q).all():
        raise ValueError('Expected finite original seven-axis path')
    if not .1<=speed_deg_s<=15.:
        raise ValueError('Speed must not exceed 15 deg/s')
    ticks=np.arange(len(q))*.1
    step=.02;padding=.8
    working=np.arange(round((ticks[-1]+2*padding)/step)+1)*step
    raw=np.column_stack([np.interp(working-padding,ticks,q[:,joint])for joint in range(7)])
    filtered=gaussian_filter1d(raw,.12/step,axis=0,mode='nearest',truncate=4.)
    filtered[0]=q[0];filtered[-1]=q[-1]
    spline=make_interp_spline(working,filtered,k=5,
        bc_type=([(1,np.zeros(7)),(2,np.zeros(7))],[(1,np.zeros(7)),(2,np.zeros(7))]))
    probes=np.linspace(0,working[-1],round(working[-1]*500)+1)
    limits=[speed_deg_s,30.,150.]
    scale=max(1.,*[float(np.max(np.abs(spline(probes,order)))/math.radians(limit))**(1/order)for order,limit in enumerate(limits,1)])*1.01
    duration=math.ceil(working[-1]*scale*10)/10
    for _attempt in range(4):
        count=round(duration*10)+1
        stamps=np.arange(count)*.1
        parameter=stamps/duration*working[-1]
        factor=working[-1]/duration
        positions=spline(parameter);velocities=spline(parameter,1)*factor;accelerations=spline(parameter,2)*factor**2
        positions[0]=q[0];positions[-1]=q[-1];velocities[[0,-1]]=0.;accelerations[[0,-1]]=0.
        points=[dict(time_s=float(stamp),positions=position.tolist(),velocities=velocity.tolist(),accelerations=acceleration.tolist())for stamp,position,velocity,acceleration in zip(stamps,positions,velocities,accelerations)]
        peaks=exact_peaks(points)
        ratio=max(*[(peak/limit)**(1/order)for order,(peak,limit) in enumerate(zip(peaks,limits),1)],1.)
        if ratio<=1.+1e-7:
            break
        duration=math.ceil(duration*ratio*1.01*10)/10
    else:
        raise ValueError('Cannot verify interpolated derivative limits')
    curves=rt_curves(points)
    dense=np.linspace(0,duration,round(duration*200)+1)
    parameter=dense/duration*working[-1]-padding
    reference=np.column_stack([np.interp(parameter,ticks,q[:,joint])for joint in range(7)])
    actual=np.column_stack([curve(dense)for curve in curves])
    deviation=float(np.rad2deg(np.abs(actual-reference).max()))
    if deviation>.5:
        raise ValueError(f'局部平滑偏离{deviation:.3f}°，超过0.5°，拒绝采用')
    if count>1000:
        raise ValueError('10Hz平滑结果超过1000点')
    report=dict(points=count,duration_s=duration,interval_s=.1,
        peak_velocity_deg_s=peaks[0],peak_acceleration_deg_s2=peaks[1],peak_jerk_deg_s3=peaks[2],
        maximum_joint_deviation_deg=deviation,speed_limit_deg_s=speed_deg_s,max_duration_s=max_duration_s,
        within_duration=duration<=max_duration_s,middle_stops=int((np.max(np.abs(velocities[1:-1]),axis=1)<1e-6).sum()),
        padding_s=padding,source_duration_s=float(ticks[-1]),working_duration_s=float(working[-1]),
        method='Local time smoothing sigma0.12s with endpoint holds; RT quintic extrema checked; original endpoints fixed')
    return points,report
