import hashlib
import json
from pathlib import Path
import time

import numpy as np
import torch

from model import export_prefix
from smooth_timing import smooth_points,rt_curves


class PrefixPreviewPlanner:
    def __init__(self,root):
        self.root=Path(root)
        self.planner=None
        self.directory=self.root/'generated/smooth_prefix_preview'
        self.directory.mkdir(parents=True,exist_ok=True)
        self.library=json.loads((self.root/'generated/left_prefix_control_20261018/prefixes.json').read_text())

    def load_planner(self):
        if self.planner is None:
            from v3_rezero_task_planner import RezeroTaskPlanner
            self.planner=RezeroTaskPlanner()
        return self.planner

    def catalog(self):
        from curobo_core.fixtures import tasks
        catalog=tasks()+tasks([1,0])
        for box_id in range(25):
            side='left'if box_id%5>=3 else'right'
            label=f"单臂 {'左手'if side=='left'else'右手'} {box_id:02d}(排{5-box_id//5}/列{5-box_id%5})"
            catalog.append((label,{side:box_id}))
        return catalog

    def build_source(self,boxes,progress):
        from scipy.spatial.transform import Rotation
        from v3_batched_loaded_search import GpuValidity
        from v3_wall_ik_benchmark import ACTIVE_JOINTS
        planner=self.load_planner();planner.configure_task(boxes)
        names=list(planner.mobile_robot['kinematics']['cspace']['joint_names'])
        checker=planner.checker(boxes);validity=GpuValidity(checker,check_ground=True)
        if not bool(validity.mask(torch.tensor(planner.home_values[None],device='cuda',dtype=torch.float32)).all()):
            raise ValueError('当前参考场景Home碰撞')
        frames=[planner.home_values.copy()];phases=['home']
        if len(boxes)==2:
            progress('枚举双臂接触候选，仅规划Home→接触，不规划抽离/运输')
            solved=planner.build_contacts(boxes)
            contacts=solved.js_solution.position.detach().cpu().numpy().reshape(-1,15)
            chosen=None
            for index,contact in enumerate(contacts):
                progress(f'预接触候选{index+1}/{len(contacts)}')
                path,stats=planner.plan_contact_approach(contact,checker,validity,20261003+index)
                if path is not None:
                    split=planner.contact_approach_split
                    frames.extend(path);phases.extend(['approach']*split+['contact_approach']*(len(path)-split))
                    chosen=index+1;break
            if chosen is None:
                raise ValueError('所有接触候选均未通过Home到接触路径验证')
        else:
            side=next(iter(boxes))
            progress('枚举单臂接触候选，仅规划Home→接触')
            target=planner.contact_poses_for_task(boxes)[side+'_tool0']
            matrix=np.eye(4);matrix[:3,:3]=Rotation.from_quat(np.roll(target['quaternion'],-1)).as_matrix();matrix[:3,3]=target['position']
            contacts=planner.enumerate_single(matrix,planner.home_values,checker)
            chosen=None
            for index,contact in enumerate(contacts):
                backwards,detail=planner.cartesian_single(contact,np.linspace(.005,.03,6),True)
                if backwards is None or not bool(validity.mask(torch.tensor(backwards,device='cuda',dtype=torch.float32)).all()):
                    continue
                path,stats=planner.single_path(planner.home_values,[backwards[-1]],checker,strict=planner.target_validity(checker))
                if path is not None:
                    frames.extend(path);phases.extend(['approach']*len(path))
                    frames.extend(backwards[::-1]);phases.extend(['contact_approach']*len(backwards))
                    chosen=index+1;break
            if chosen is None:
                raise ValueError('单臂没有通过Home→接触路径验证的候选')
        frames.append(np.asarray(frames[-1]));phases.append('attach')
        output=[]
        for row in frames:
            mapping=dict(zip(ACTIVE_JOINTS,row));mapping['base_x']=planner.base_advance
            output.append([float(mapping.get(name,0.))for name in names])
        return dict(success=True,task=boxes,joint_names=names,frames=output,phases=phases,
                    contact_roll_policy='RezeroTaskPlanner aligned contact',selected_candidate=chosen,
                    scope='Internal attach marker only to delimit prefix; no vacuum or post-contact planning')

    def compute(self,boxes,speed,progress):
        began=time.monotonic()
        name='_'.join(('L'if side=='left'else'R')+str(value)for side,value in boxes.items())
        cached=self.library['tasks'].get(name)
        if cached and Path(cached['source']).is_file():
            progress('读取已验证原始接触前缀，重新计算平滑与插值验收')
            source=json.loads(Path(cached['source']).read_text())
        else:
            source=self.build_source(boxes,progress)
        source_path=self.directory/(name+'_source.json');source_path.write_text(json.dumps(source))
        record=export_prefix(source_path)
        progress('计算10Hz连续时间轨迹，校验RT五次插值峰值')
        points,report=smooth_points(record['model_positions'],speed)
        planner=self.load_planner();planner.configure_task(boxes)
        from v3_wall_ik_benchmark import ACTIVE_JOINTS
        from v3_batched_loaded_search import GpuValidity
        checker=planner.checker(boxes);ordinary=GpuValidity(checker,check_ground=True);strict=planner.target_validity(checker)
        names=source['joint_names'];indices=[names.index(joint)for joint in ACTIVE_JOINTS]
        original=np.asarray(source['frames'])[:len(record['model_positions']),indices]
        ticks=np.arange(len(original))*.1
        stamps=np.linspace(0,report['duration_s'],round(report['duration_s']*200)+1)
        parameter=stamps/report['duration_s']*report['working_duration_s']-report['padding_s']
        values=np.column_stack([np.interp(parameter,ticks,original[:,index])for index in range(15)])
        values[:,1:8]=np.column_stack([curve(stamps)for curve in rt_curves(points)])
        phases=np.asarray(record['phases'])[np.clip(np.floor(parameter/.1).astype(int),0,len(original)-1)]
        progress(f'逐段检查{len(values)}个插值状态的参考场景碰撞')
        for label,validity in [('approach',strict),('other',ordinary)]:
            selected=np.where(phases=='approach')[0]if label=='approach'else np.where(phases!='approach')[0]
            for offset in range(0,len(selected),512):
                ids=selected[offset:offset+512]
                mask=validity.mask(torch.tensor(values[ids],device='cuda',dtype=torch.float32).contiguous())
                if not bool(mask.all()):
                    bad=int(ids[int(torch.where(~mask)[0][0])])
                    raise ValueError(f'平滑后参考场景碰撞：{stamps[bad]:.3f}s，禁止发布')
        report.update(collision_checked=True,collision_samples=len(values),collision_scope='Recorded reference right/updown and world; only left commands sent',
                      left_task='left'in boxes,compute_s=time.monotonic()-began)
        payload=dict(task=name,boxes=boxes,points=points,report=report,record=record)
        payload['preview_id']=hashlib.sha256(json.dumps(payload,sort_keys=True).encode()).hexdigest()
        (self.directory/(name+'_preview.json')).write_text(json.dumps(payload))
        return payload

    def compute_offline(self,boxes,speed,progress):
        name='_'.join(('L'if side=='left'else'R')+str(value)for side,value in boxes.items())
        if name not in self.library['tasks']:
            raise ValueError('该任务尚无保存的接触前缀；当前GPU显存不足，暂停新规划，不提供执行')
        record=self.library['tasks'][name]
        progress('只读预览：重新计算已保存前缀的连续10Hz时间轨迹')
        points,report=smooth_points(record['model_positions'],speed)
        report.update(collision_checked=False,collision_samples=0,left_task='left'in boxes,
                      collision_scope='Pending GPU check; offline preview only')
        payload=dict(task=name,boxes=boxes,points=points,report=report,record=record)
        payload['preview_id']=hashlib.sha256(json.dumps(payload,sort_keys=True).encode()).hexdigest()
        return payload
