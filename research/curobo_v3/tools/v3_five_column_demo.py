#!/usr/bin/env python3
"""CPU-only Viser replay for a composed five-column result."""
import argparse, json, threading, time
from pathlib import Path
import numpy as np, viser, yaml, yourdfpy
from scipy.spatial.transform import Rotation
from viser.extras import ViserUrdf
from curobo_core.adapter import pose_matrix
from curobo_core.contracts import PlannerAssets
from curobo_core.planner import FullCyclePlanner
from curobo_core.scene import Pose
from curobo_core.sequential import sequential_request

LABELS={'park_for_base_move':'列间：双臂移动安全姿态','base_translate':'列间：底盘横移','prepare_column':'列间：准备下一列',**{f'column_{i}':f'卸载第{i}列顶部两箱' for i in range(5)}}
def quat(T):return tuple(np.roll(Rotation.from_matrix(T[:3,:3]).as_quat(),1))
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--result',type=Path,required=True);p.add_argument('--port',type=int,default=8095);a=p.parse_args();result=json.loads(a.result.read_text());seed=result['seed'];planner=FullCyclePlanner(PlannerAssets());request=sequential_request(planner)
 config=yaml.safe_load(planner.args.mobile_robot_config.read_text());urdf_path=Path(config.get('robot_cfg',config)['kinematics']['urdf_path']);urdf=yourdfpy.URDF.load(urdf_path,load_meshes=True,build_scene_graph=True);server=viser.ViserServer(host='127.0.0.1',port=a.port,label='Five-column unload · untimed geometry replay');server.gui.configure_theme(control_width='large');server.scene.set_up_direction('+z');server.scene.add_grid('/ground',width=5,height=3);robot=ViserUrdf(server,urdf,root_node_name='/robot',mesh_color_override=(.45,.49,.57,1.))
 world={x.object_id:server.scene.add_box('/world/'+x.object_id,dimensions=x.dimensions_m,position=x.pose.position,wxyz=x.pose.quaternion_wxyz,color=(216,130,70) if x.object_id=='warehouse_top_door_leaf' else (87,145,165),opacity=.45 if x.object_id.startswith('wall_box_') else .08) for x in request.snapshot.objects if x.object_id!='ground' and x.object_id not in ('left_wall','right_wall')};payload={side:server.scene.add_box('/payload/'+side,dimensions=(.3,.4,.4),color=(239,146,62),visible=False) for side in ('left','right')}
 phases=result['phases']
 def attachments(index):
  active={}
  for event in result['lifecycle']:
   if event['global_frame_index']>index:continue
   if 'attachment' in event:active[event['object_id']]=event['attachment']
   if 'release' in event['phase'] or 'disappear' in event['phase']:active.pop(event['object_id'],None)
  return list(active.values())
 frames=result['frames'];slider=server.gui.add_slider('几何帧（非执行时间）',min=0,max=len(frames)-1,step=1,initial_value=0);options=[]
 for phase in phases:
  label=LABELS.get(phase,phase)
  if label not in options:options.append(label)
 jump=server.gui.add_dropdown('跳转阶段',options=options);play=server.gui.add_checkbox('播放',initial_value=False);speed=server.gui.add_slider('播放倍速',min=0.1,max=5.0,step=0.1,initial_value=1.0);status=server.gui.add_markdown('')
 planning=result.get('planning_timing',{})
 timing_rows={row['column']:row for row in planning.get('rows',[])}
 def timing_group(stage):
  if stage.startswith(('移动前','到站后','底盘','读取并准备','接入本列','列间')):return 'transition'
  if '上箱' in stage or stage.startswith('右臂'):return 'upper'
  if '下箱' in stage or stage.startswith('左臂'):return 'lower'
  return 'other'
 def stage_table(rows,total_ms):
  lines=['| 阶段 | 秒 | 占本列 | 求解方式 |','|---|---:|---:|---|']
  lines += [f"| {row['stage']} | {row['ms']/1000:.3f} | {row['ms']/total_ms*100:.1f}% | {row['method']} |" for row in rows]
  return '\n'.join(lines) if rows else '无阶段记录。'
 with server.gui.add_folder('规划耗时',expand_by_default=True):
  server.gui.add_markdown(f"**累计规划计算：{planning.get('total_ms',0.)/1000:.3f} 秒**  \n全部为规划计算墙钟，不包含GPU锁等待。  \n**执行时间未提供：轨迹未做动力学时间参数化。**")
  timing_column=server.gui.add_dropdown('选择列',options=[f"第{column}列" for column in result['order']],initial_value=f"第{result['order'][0]}列")
  column_summary=server.gui.add_markdown('')
  with server.gui.add_folder('① 列间状态衔接',expand_by_default=True):transition_table=server.gui.add_markdown('')
  with server.gui.add_folder('② 上箱流程',expand_by_default=True):upper_table=server.gui.add_markdown('')
  with server.gui.add_folder('③ 下箱流程',expand_by_default=True):lower_table=server.gui.add_markdown('')
  with server.gui.add_folder('④ 初始化与其他',expand_by_default=False):other_table=server.gui.add_markdown('')
 def update_timing_column():
  column=int(timing_column.value.removeprefix('第').removesuffix('列'));row=timing_rows[column];stages=row.get('stages',[])
  grouped={key:[stage for stage in stages if timing_group(stage['stage'])==key] for key in ('transition','upper','lower','other')}
  totals={key:sum(stage['ms'] for stage in values) for key,values in grouped.items()}
  column_summary.content=(f"### 第{column}列 · 规划合计 **{row['total_ms']/1000:.3f}秒**\n"
      f"| 层级 | 秒 | 占本列 |\n|---|---:|---:|\n"
      f"| 列间状态衔接 | {totals['transition']/1000:.3f} | {totals['transition']/row['total_ms']*100:.1f}% |\n"
      f"| 本列卸载动作 | {(row['total_ms']-totals['transition'])/1000:.3f} | {(row['total_ms']-totals['transition'])/row['total_ms']*100:.1f}% |")
  transition_table.content=stage_table(grouped['transition'],row['total_ms']);upper_table.content=stage_table(grouped['upper'],row['total_ms']);lower_table.content=stage_table(grouped['lower'],row['total_ms']);other_table.content=stage_table(grouped['other'],row['total_ms'])
 @timing_column.on_update
 def _(_event):update_timing_column()
 update_timing_column()
 def update():
  i=int(slider.value);values=dict(zip(result['joint_names'],frames[i]));cfg=[values.get(n,0.) for n in urdf.actuated_joint_names];urdf.update_cfg(cfg);robot.update_cfg(cfg);active=attachments(i);active_ids={x['object_id'] for x in active};removed={x['object_id'] for x in result['lifecycle'] if x['global_frame_index']<=i and ('release' in x['phase'] or 'disappear' in x['phase'])}
  for oid,node in world.items():node.visible=oid not in active_ids and oid not in removed
  for node in payload.values():node.visible=False
  for item in active:
   side=item['parent_link'].removesuffix('_tool0');T=urdf.get_transform(item['parent_link'],urdf.base_link)@pose_matrix(Pose(**item['tool_to_object']));node=payload[side];node.position=T[:3,3];node.wxyz=quat(T);node.visible=True
  phase=phases[i];status.content=f"**{LABELS.get(phase,phase)}**  \n帧：**{i+1}/{len(frames)}**  \n底盘Y：**{values['base_y']:.3f} m** · yaw：**{np.degrees(values['base_yaw']):.1f}°**  \n已移除箱体：**{len(removed)}/10** · 播放：**{speed.value:.1f}×**"
 @slider.on_update
 def _(_event):update()
 @jump.on_update
 def _(_event):
  phase=next(k for k,v in LABELS.items() if v==jump.value);play.value=False;slider.value=phases.index(phase)
 def loop():
  progress=0.0
  while True:
   if play.value:
    progress+=float(speed.value)
    step=int(progress)
    progress-=step
    if step:
     slider.value=(int(slider.value)+step)%len(frames)
   else:
    progress=0.0
   time.sleep(.02)
 update();threading.Thread(target=loop,daemon=True).start()
 while True:time.sleep(1)
if __name__=='__main__':main()
