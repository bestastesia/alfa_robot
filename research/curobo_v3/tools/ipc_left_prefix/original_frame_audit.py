import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.interpolate import BPoly,PPoly


def analyze(record):
    positions=np.asarray(record['model_positions'],dtype=float)
    times=np.arange(len(positions),dtype=float)*.1
    velocities=np.gradient(positions,.1,axis=0,edge_order=2)
    velocities[[0,-1]]=0.
    accelerations=np.gradient(velocities,.1,axis=0,edge_order=2)
    accelerations[[0,-1]]=0.
    curves=[];limits={};worst={}
    for order,label in [(1,'velocity'),(2,'acceleration'),(3,'jerk')]:
        maximum=0.;location=None
        for joint in range(7):
            if order==1:
                curve=PPoly.from_bernstein_basis(BPoly.from_derivatives(times,
                    np.column_stack([positions[:,joint],velocities[:,joint],accelerations[:,joint]])))
                curves.append(curve)
            curve=curves[joint]
            critical=curve.derivative(order+1).roots(extrapolate=False)
            samples=np.concatenate([times,critical[np.isfinite(critical)]])
            values=np.abs(curve.derivative(order)(samples))
            index=int(values.argmax())
            if values[index]>maximum:
                maximum=float(values[index]);stamp=float(samples[index])
                segment=min(len(times)-2,max(0,int(np.searchsorted(times,stamp,side='right')-1)))
                location=dict(joint='left_joint'+str(joint+1),time_s=stamp,
                    from_frame=segment+1,to_frame=segment+2,phase=record['phases'][segment])
        limits[label]=math.degrees(maximum);worst[label]=location
    durations=dict(velocity_acceleration=max(1.,limits['velocity']/15.,math.sqrt(limits['acceleration']/30.)),
                   velocity_acceleration_jerk=max(1.,limits['velocity']/15.,math.sqrt(limits['acceleration']/30.),(limits['jerk']/150.)**(1/3)))
    duration=float(times[-1])
    sample_times=np.linspace(0,duration,(len(times)-1)*20+1)
    interpolated=np.column_stack([curve(sample_times) for curve in curves])
    segments=np.minimum((sample_times/.1).astype(int),len(times)-2)
    fractions=(sample_times-times[segments])/.1
    delta=positions[segments+1]-positions[segments]
    fraction=np.clip(np.sum((interpolated-positions[segments])*delta,axis=1)/np.maximum(np.sum(delta*delta,axis=1),1e-20),0.,1.)
    errors=np.max(np.abs(interpolated-positions[segments]-fraction[:,None]*delta),axis=1)
    error_index=int(errors.argmax())
    lower=np.deg2rad([-180,-90,-180,-145,-180,-110,-180])
    upper=np.deg2rad([180,120,180,145,180,110,180])
    bound=np.maximum(lower-interpolated,interpolated-upper)
    return dict(points=len(positions),interval_s=.1,duration_s=duration,
        method='Original positions unchanged; central-difference velocity/acceleration; endpoint v/a zero; exact RT quintic Hermite extrema',
        peak_deg_units=limits,worst=worst,
        max_interval_path_deviation_deg=math.degrees(float(errors.max())),
        max_deviation_from_frame=int(segments[error_index])+1,
        joint_bound_violation_deg=math.degrees(max(0.,float(bound.max()))),
        middle_all_joint_zero_velocity_count=int(np.sum(np.max(np.abs(velocities[1:-1]),axis=1)<1e-9)),
        timing_options={key:dict(uniform_scale=value,duration_s=duration*value,within_60s=duration*value<=60.,
            effective_point_frequency_hz=10/value,points=len(positions)) for key,value in durations.items()}),positions,velocities,accelerations,curves


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--library',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    library=json.loads(args.library.read_text())
    reports={}
    args.output_dir.mkdir(parents=True,exist_ok=True)
    for name,record in library['tasks'].items():
        report,positions,velocities,accelerations,_curves=analyze(record)
        reports[name]=report
        (args.output_dir/(name+'.json')).write_text(json.dumps(dict(report=report,
            positions=positions.tolist(),velocities=velocities.tolist(),accelerations=accelerations.tolist()),indent=2))
        print(name,json.dumps(report,ensure_ascii=False),flush=True)
    (args.output_dir/'summary.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
