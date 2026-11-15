import copy
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import torch
from scipy.spatial.transform import Rotation

from curobo.types import JointState
from v3_guided_cycle_planner import GuidedCyclePlanner
from v3_batched_loaded_search import densify
from v3_route_ik_benchmark import sample_route
from v3_spherical_wrist_route import plan_wrist, wrist_geometry
from v3_suction_cluster_navigation import ClusterCollision
from v3_wall_ik_benchmark import ACTIVE_JOINTS


class WristGuidedCyclePlanner(GuidedCyclePlanner):
    def __init__(self, args, reference_source=None):
        super().__init__(args)
        source=Path(reference_source or '/mnt/mydisk/ALFA/alfa_robot_v3_curobo/research/curobo_v3/generated/spherical_wrist_20261022/result.json')
        source_bytes=source.read_bytes()
        self.wrist_settings=json.loads(source_bytes)
        self.wrist_settings_sha256=hashlib.sha256(source_bytes).hexdigest()
        self.wrist_geometry=wrist_geometry(args.urdf)

    def _reference(self, checker, validity, start, goal):
        started=time.perf_counter()
        scene=checker.guidance_scene
        if scene.mesh:
            raise ValueError('球腕参考暂不支持未转换场景网格')
        objects=[dict(name=item.name,dimensions=list(item.dims),position=list(item.pose[:3]),quaternion_wxyz=list(item.pose[3:])) for item in scene.cuboid or []]
        strict=validity if hasattr(validity,'targets') else getattr(validity.evaluate,'__self__',None)
        if strict is not None and hasattr(strict,'targets'):
            objects.extend(dict(name='precontact_target_'+side,dimensions=list(item['dimensions']),position=list(item['center']),quaternion_wxyz=[1.,0.,0.,0.]) for side,item in strict.targets.items())
        objects=[item for item in objects if not item['name'].startswith('robot_internal/')]
        if getattr(self,'capture_guidance_nodes',False):
            self.guidance_events[-1]['reference_scene_objects']=objects
        initial=self._tool_poses(checker,start)
        target=self._tool_poses(checker,goal)
        state=checker.kinematics.compute_kinematics(JointState.from_position(start.reshape(1,-1),joint_names=ACTIVE_JOINTS))
        spheres=state.robot_spheres.reshape(-1,4).detach().cpu().numpy()
        models={}
        for side in ('left','right'):
            indices=checker.kinematics.config.kinematics_config.get_sphere_index_from_link_name(side+'_link7').cpu().numpy()
            selected=spheres[indices]
            selected=selected[selected[:,3]>0]
            transform=np.asarray(initial[side])
            local=(selected[:,:3]-transform[:3,3])@transform[:3,:3]
            models[side]=dict(centers_tool=local.tolist(),radii=selected[:,3].tolist())
        configured_margin=self.wrist_settings['margin_m']
        minimum_gap=float('inf')
        for side,model in models.items():
            poses=np.array([np.r_[np.asarray(matrix)[:3,3],Rotation.from_matrix(np.asarray(matrix)[:3,:3]).as_quat()] for matrix in (initial[side],target[side])])
            minimum_gap=min(minimum_gap,float(ClusterCollision(model,objects,0).clearance(poses).min()))
        effective_margin=configured_margin
        if minimum_gap>0 and minimum_gap<=configured_margin:
            effective_margin=minimum_gap*.5
        reference=plan_wrist(dict(tcp_world=initial,targets=target),objects,dict(arms=models),self.wrist_geometry,
            copy.deepcopy(self.wrist_settings['limits']),offset=0.,margin=effective_margin)
        self.guidance_events[-1].update(reference_kind='spherical_wrist_c2_no_velocity_z_bias',
            reference_scene_scope='external scene only; no vehicle body; actual tool and attached payload spheres',
            reference_collision_spheres={side:len(model['radii']) for side,model in models.items()},
            reference_margin_m=effective_margin,configured_reference_margin_m=configured_margin,
            endpoint_external_gap_m=minimum_gap)
        if not reference['success']:
            raise RuntimeError(reference.get('failure','球腕参考未通过连续场景检查'))
        samples,progress=sample_route(dict(tracks=reference['paths']),128)
        reference['total_ms']=(time.perf_counter()-started)*1000
        return reference,samples,progress

    def _guided_search(self,start,goals,lower,upper,validity,budget,seed,**kwargs):
        path,stats=super()._guided_search(start,goals,lower,upper,validity,budget,seed,**kwargs)
        if path is not None:
            began=time.perf_counter()
            rows=torch.tensor(np.asarray(densify(path)),device='cuda',dtype=torch.float32)
            state=validity.checker.kinematics.compute_kinematics(JointState.from_position(rows[None],joint_names=ACTIVE_JOINTS))
            tracks={}
            for side in ('left','right'):
                pose=state.tool_poses.to_dict()[side+'_tool0']
                positions=pose.position.reshape(-1,3).detach().cpu().numpy()
                quaternion=pose.quaternion.reshape(-1,4).detach().cpu().numpy()
                rotations=Rotation.from_quat(np.roll(quaternion,-1,axis=1))
                centers=positions+rotations.apply(np.repeat(np.asarray(self.wrist_geometry['arms'][side]['center_tool'])[None],len(rows),axis=0))
                tracks[side]=dict(positions=positions.tolist(),centers=centers.tolist(),quaternions_wxyz=quaternion.tolist(),
                    total_rotation_deg=float(np.rad2deg((rotations[:-1].inv()*rotations[1:]).magnitude()).sum()),
                    wrist_length_m=float(np.linalg.norm(np.diff(centers,axis=0),axis=1).sum()))
            self.guidance_events[-1]['actual_tracks']=tracks
            self.guidance_events[-1]['actual_rotation_deg']={side:entry['total_rotation_deg'] for side,entry in tracks.items()}
            self.guidance_events[-1]['trace_metrics_ms']=(time.perf_counter()-began)*1000
        return path,stats

    def full_cycle(self,*args,**kwargs):
        result=super().full_cycle(*args,**kwargs)
        result['guided_reference_kind']='spherical_wrist_c2_no_velocity_z_bias'
        result['guided_search_policy']='Four stages: external-scene spherical-wrist C2 reference, no velocity-Z bias -> LM128x16/batch64 -> native whole-robot/payload filtering -> first finely valid Informed Connect'
        result['wrist_guidance_settings']=dict(limits=self.wrist_settings['limits'],margin_m=self.wrist_settings['margin_m'],
            geometry=self.wrist_geometry,source_sha256=self.wrist_settings_sha256,reference_scope='external scene only; native whole-robot validation unchanged')
        return result
