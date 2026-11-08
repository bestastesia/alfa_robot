#!/usr/bin/env python3
"""Independent acceptance gate for the modernized central door pair flow."""
import argparse
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from curobo_core.contracts import PlannerAssets
from curobo_core.door_recenter import ROOT
import yourdfpy


EXECUTION_SOURCES = (
    'tools/v3_door_recenter.py',
    'tools/curobo_core/door_recenter.py',
    'tools/curobo_core/door_sequential.py',
    'tools/curobo_core/door_sequential_validity.py',
    'tools/curobo_core/door_extract.py',
    'tools/curobo_core/torque.py',
    'tools/v3_cartesian_dual_transport.py',
    'tools/v3_cartesian_transport.py',
    'tools/v3_conveyor_cycle_planner.py',
    'tools/v3_batched_loaded_search.py',
)


def phase_rotation(model, result, phase, side):
    names=result['joint_names']
    selected=np.flatnonzero(np.asarray(result['phases'])==phase)
    if not len(selected):
        return dict(total_deg=float('inf'),net_deg=float('inf'),extra_deg=float('inf'))
    rotations=[]
    for frame in np.asarray(result['frames'])[selected]:
        values=dict(zip(names,frame))
        model.update_cfg([values.get(name,0.) for name in model.actuated_joint_names])
        rotations.append(model.get_transform(side+'_tool0',model.base_link)[:3,:3])
    values=Rotation.from_matrix(np.asarray(rotations))
    steps=(values[:-1].inv()*values[1:]).magnitude() if len(values)>1 else np.zeros(0)
    net=(values[0].inv()*values[-1]).magnitude() if len(values)>1 else 0.
    return dict(total_deg=math.degrees(float(steps.sum())),net_deg=math.degrees(float(net)),
                extra_deg=math.degrees(float(max(0.,steps.sum()-net))))


def source_audit(result):
    recorded=result.get('source_identity',{})
    mismatches=[]
    for relative in EXECUTION_SOURCES:
        path=ROOT/relative
        actual=hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
        if recorded.get(relative)!=actual:
            mismatches.append(dict(path=relative,recorded=recorded.get(relative),current=actual))
    return dict(success=not mismatches,mismatches=mismatches,checked=list(EXECUTION_SOURCES))


def lifecycle_audit(result,expected_boxes):
    events=result.get('predicted_scene_events',())
    attach=next((x for x in events if x.get('phase')=='left_attach'),None)
    releases=[x for x in events if x.get('phase')=='release' and x.get('object_id') in expected_boxes]
    placed={x.get('object_id') for x in result.get('placed_boxes',{}).values()}
    rows=result.get('attachments_by_frame',())
    held=False;stable=False;release_frame=None
    if attach is not None and {x.get('object_id') for x in releases}==expected_boxes:
        start=int(attach['frame_index']);release_frame=min(int(x['frame_index']) for x in releases)
        if 0<=start<release_frame<=len(rows):
            expected={x['object_id']:x for x in rows[start]}
            held=set(expected)==expected_boxes and all(
                {x['object_id'] for x in row}==expected_boxes for row in rows[start:release_frame])
            stable=held and all({x['object_id']:x for x in row}==expected for row in rows[start:release_frame])
    final_empty=not result.get('final_snapshot',{}).get('attachments',())
    success=placed==expected_boxes and len(releases)==2 and held and stable and final_empty
    return dict(success=success,placed_boxes=sorted(placed),release_count=len(releases),
                held_from_left_attach_until_release=held,tool_to_object_unchanged=stable,
                release_frame=release_frame,final_attachments_empty=final_empty)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--result',type=Path,required=True)
    p.add_argument('--assets-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--left-precontact-extra-max-deg',type=float,default=30.)
    p.add_argument('--right-precontact-extra-max-deg',type=float,default=105.)
    p.add_argument('--recenter-extra-max-deg',type=float,default=65.)
    p.add_argument('--motion-max-s',type=float,default=60.)
    a=p.parse_args();result=json.loads(a.result.read_text());assets=PlannerAssets();assets=replace(
        assets,**{name:a.assets_root/getattr(assets,name).relative_to(ROOT)
                  for name in ('robot_config','mobile_robot_config','urdf','named_poses','box_fit')})
    model=yourdfpy.URDF.load(assets.urdf,load_meshes=False,build_scene_graph=True)
    metrics={phase+'_'+side:phase_rotation(model,result,phase,side)
             for phase,side in [('right_precontact','right'),('left_precontact','left'),
                                ('left_recenter','left'),('right_recenter','left'),
                                ('right_recenter','right')]}
    selected=next((item for item in result.get('contact_geometry',())
                   if item.get('side')=='left' and item.get('selected')),None)
    expected_boxes={f"wall_box_{int(value):02d}" for value in result.get('task',{}).values()}
    source=source_audit(result);lifecycle=lifecycle_audit(result,expected_boxes)
    atlas=result.get('latest_compact_atlas',{});torque=result.get('torque_validation',{})
    atlas_ok=(atlas.get('atlas_rows')==2816 and atlas.get('selected')==64 and
              atlas.get('policy_radius_m')==1.05 and atlas.get('selected_radius_max_m',float('inf'))<=1.05 and
              Path(atlas.get('atlas_path','')).name=='all_solutions.npz')
    torque_ok=(bool(torque.get('success')) and bool(torque.get('geometry_frozen')) and
               torque.get('maximum_position_delta',float('inf'))<=1e-12 and
               torque.get('payload_mass_kg')==10. and torque.get('torque_limit_scale')==.9 and
               torque.get('maximum_utilization',float('inf'))<=.900001)
    checks=dict(
        task_success=bool(result.get('success')),
        box_lifecycle=lifecycle['success'],
        selected_grasp_recorded=selected is not None,
        current_execution_source=source['success'],
        latest_atlas_contract=atlas_ok,
        precontact_rotation=(metrics['right_precontact_right']['extra_deg']<=a.right_precontact_extra_max_deg and
                             metrics['left_precontact_left']['extra_deg']<=a.left_precontact_extra_max_deg),
        recenter_rotation=(metrics['left_recenter_left']['extra_deg']<=a.recenter_extra_max_deg and
                           metrics['right_recenter_left']['extra_deg']<=a.recenter_extra_max_deg and
                           metrics['right_recenter_right']['extra_deg']<=a.recenter_extra_max_deg),
        compact_envelope=(result.get('dual_recenter_verification',{}).get(
            'actual_whole_robot_payload_radius_m',float('inf'))<=1.05),
        timing=bool(result.get('time_parameterized')) and result.get('motion_duration_s',float('inf'))<=a.motion_max_s,
        torque=torque_ok,
        navigation_explicitly_unvalidated=(result.get('navigation_path_validated') is False and
                                           result.get('base_transition_policy')=='navigation arrival state switch; no base trajectory'))
    output=dict(success=all(checks.values()),checks=checks,metrics=metrics,lifecycle=lifecycle,
                source_audit=source,atlas_audit=atlas,torque_audit={k:torque.get(k) for k in (
                    'success','geometry_frozen','maximum_position_delta','payload_mass_kg',
                    'torque_limit_scale','maximum_utilization','maximum_gravity_utilization')},
                selected_left_roll_deg=None if selected is None else math.degrees(selected['roll_rad']),
                motion_duration_s=result.get('motion_duration_s'),
                compact_radius_m=result.get('dual_recenter_verification',{}).get(
                    'actual_whole_robot_payload_radius_m'),
                thresholds=dict(left_precontact_extra_max_deg=a.left_precontact_extra_max_deg,
                                right_precontact_extra_max_deg=a.right_precontact_extra_max_deg,
                                recenter_extra_max_deg=a.recenter_extra_max_deg,
                                motion_max_s=a.motion_max_s))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(output,ensure_ascii=False))
    raise SystemExit(0 if output['success'] else 1)


if __name__=='__main__':main()
