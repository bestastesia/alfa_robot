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
 with server.gui.add_folder('各列规划耗时与求解方式',expand_by_default=True):
  server.gui.add_markdown(f"**累计规划计算：{planning.get('total_ms',0.)/1000:.3f} 秒**  \n全部为规划计算墙钟；不包含GPU锁等待。  \n**执行时间：未提供（轨迹未做动力学时间参数化）**")
  timing_column=server.gui.add_dropdown('选择耗时列',options=[f"第{column}列" for column in result['order']],initial_value=f"第{result['order'][0]}列")
  column_summary=server.gui.add_markdown('')
  stage_slots=[server.gui.add_markdown('') for _ in range(max((len(row.get('stages',[])) for row in timing_rows.values()),default=0))]
 def update_timing_column():
  column=int(timing_column.value.removeprefix('第').removesuffix('列'));row=timing_rows[column]
  column_summary.content=(f"### 第{column}列 · 合计 **{row['total_ms']/1000:.3f} 秒**  \n"
      f"列间状态衔接与底盘路径检查（规划计算）：**{row['transition_ms']/1000:.3f}秒** · 本列规划：**{row['component_ms']/1000:.3f}秒**  \n{row['method']}")
  stages=row.get('stages',[])
  for index,slot in enumerate(stage_slots):
   slot.content=(f"**{index+1}. {stages[index]['stage']}**  \n耗时：**{stages[index]['ms']/1000:.3f}秒**  \n求解：{stages[index]['method']}" if index<len(stages) else '')
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
