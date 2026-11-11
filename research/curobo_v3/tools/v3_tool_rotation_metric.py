import numpy as np
from scipy.spatial.transform import Rotation


def rotation_metrics(quaternions, phases):
    output={}
    for side,raw in quaternions.items():
        rotations=Rotation.from_quat(np.asarray(raw,dtype=np.float64)[:,[1,2,3,0]])
        stages={};start=0
        while start<len(phases):
            end=start+1
            while end<len(phases) and phases[end]==phases[start]:end+=1
            phase=phases[start]
            if phase not in ('base_reposition','base_return','navigation_to_pick','base_pick_reposition','home','attach','release'):
                begin=max(0,start-1)
                section=rotations[begin:end]
                accumulated=float(np.sum((section[:-1].inv()*section[1:]).magnitude())) if end-begin>1 else 0.
                necessary=float((section[0].inv()*section[-1]).magnitude())
                record=stages.setdefault(phase,dict(total_deg=0.,necessary_deg=0.,extra_deg=0.))
                record['total_deg']+=float(np.rad2deg(accumulated))
                record['necessary_deg']+=float(np.rad2deg(necessary))
                record['extra_deg']+=float(np.rad2deg(max(0.,accumulated-necessary)))
            start=end
        output[side]=dict(stages=stages,total_deg=sum(v['total_deg'] for v in stages.values()),
            extra_deg=sum(v['extra_deg'] for v in stages.values()))
    output['combined_extra_deg']=sum(output[side]['extra_deg'] for side in quaternions)
    output['combined_total_deg']=sum(output[side]['total_deg'] for side in quaternions)
    return output
