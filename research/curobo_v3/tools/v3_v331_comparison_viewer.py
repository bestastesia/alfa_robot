import json
from pathlib import Path
import time
import queue
import argparse
import yaml

import numpy as np
from scipy.spatial.transform import Rotation
import viser
from viser.extras import ViserUrdf
import yourdfpy


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--candidate',action='store_true')
    parser.add_argument('--smoke-test',action='store_true')
    parser.add_argument('--original-home',action='store_true')
    args=parser.parse_args()
    if args.original_home and args.candidate:
        parser.error('--original-home displays the unchanged original URDF; do not combine with --candidate')
    home=yaml.safe_load((Path(__file__).resolve().parents[1]/'models/robot_description_analytic_071cb95/config/named_poses_analytic_proxy.yaml').read_text())['named_poses']['home']
    home_label='原第一初始姿态Home（仅旧URDF，不换算）'
    geometry=json.loads(Path('/tmp/v331_model_compare_20261018/geometry.json').read_text())
    paths={'current':Path('/mnt/mydisk/ALFA/alfa_robot_v3/research/curobo_v3/generated/v3_analytic_071cb95/model/robot_meter_meshes.urdf'),
           'new':Path('/mnt/mydisk/ALFA/backpack/real2sim_v3.3.1/urdf/robot.urdf')}
    if args.candidate:
        paths['current']=Path(__file__).resolve().parents[1]/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf'
    server=viser.ViserServer(host='127.0.0.1',port=8152,label='当前cuRobo vs 3.3.1 · footprint与运动学对照')
    robots={};roots={};models={};markers=[]
    for label,path in paths.items():
        def resolve(fname,path=path):
            return str((path.parent/fname).resolve()) if not Path(fname).is_absolute() else fname
        model=yourdfpy.URDF.load(path,load_meshes=True,build_scene_graph=True,filename_handler=resolve)
        models[label]=model
        roots[label]=server.scene.add_frame('/'+label,show_axes=False)
        robots[label]=ViserUrdf(server,model,root_node_name='/'+label,mesh_color_override=(80,155,240) if label=='current' else (245,160,70))
    if args.candidate:
        from v3_rezero_validation import centers
        geometry['current']={side:dict(geometry['current'][side],shoulder_elbow_wrist_m=centers(models['current'],side).tolist()) for side in ('left','right')}
    mode=server.gui.add_dropdown('对照方式',options=['并排：各自footprint在地面','同一footprint重合：仅当前模型','同一footprint重合：仅3.3.1','同一footprint重合：两者'],initial_value='并排：各自footprint在地面')
    presets=['全部零位','同一组关节角（不代表同一姿态）']+([] if args.candidate else [home_label])
    pose=server.gui.add_dropdown('关节状态',options=presets,initial_value=home_label if args.original_home else '全部零位')
    axes=server.gui.add_checkbox('显示footprint/base_link及肩肘腕中心',initial_value=True)
    tools_visible=server.gui.add_checkbox('显示两版tool0原点和XYZ轴',initial_value=True)
    focus_tools=server.gui.add_button('近看左腕与两版tool0')
    side_view=server.gui.add_button('侧面看footprint与高度')
    front_view=server.gui.add_button('正面看肩宽')
    reset_current=server.gui.add_button('新旧模型双臂全部归零')
    edits=queue.Queue()
    controls={}
    for label in models:
        with server.gui.add_folder('旧current蓝色双臂' if label=='current' else '新3.3.1橙色双臂'):
            for name in [f'{side}_joint{index}' for side in ('left','right') for index in range(1,8)]:
                joint=models[label].joint_map[name]
                factor=180/np.pi
                key=(label,name)
                initial=float(np.rad2deg(home[name])) if args.original_home and label=='current' else 0.
                slider=server.gui.add_slider(name+' °',min=float(joint.limit.lower*factor),max=float(joint.limit.upper*factor),step=.1,initial_value=initial)
                number=server.gui.add_number(name+' 精确值 °',initial_value=initial,step=.1)
                controls[key]=(slider,number,factor)
                def slider_changed(_event,key=key):edits.put((key,controls[key][0].value))
                def number_changed(_event,key=key):edits.put((key,controls[key][1].value))
                slider.on_update(slider_changed);number.on_update(number_changed)
    server.gui.add_markdown('新旧模型左右臂共28轴独立修改；升降、头部和底盘关节保持零位。支持滑动或输入数值；不检查碰撞，不连接实机。')
    server.gui.add_markdown('蓝色＝当前cuRobo模型；橙色＝3.3.1。并排仅为显示加Y偏移，重合视图严格对齐footprint，不做肩部平移对齐。红点base_link、绿色footprint；肩/肘/腕为关节轴交点估计，不是CAD link原点。只读离线对比，无SSH/ROS或实机控制。')
    if args.candidate:
        server.gui.add_markdown('本页蓝色current已切为重零候选URDF：左旧角[-90,15,0,0,90,0,90]及右[90,-15,0,0,-90,0,-90]现在是真正关节0；J2正方向反转，左限位[-90,120]°、右[-120,90]°，其余数值限位不变。不是改Home。橙色仍是同事原始3.3.1，未改tool0或J3。')
    if args.original_home:
        server.gui.add_markdown('现在蓝色使用未重零的原URDF和文件内第一Home：Updown=-0.3m；左[155,-90,20,90,-90,-40,0]°，右[-155,90,-20,-90,90,40,0]°。没有将J2改成±105或截到±90。橙色保持原3.3.1，可独立拖动比较。')
    status=server.gui.add_markdown('')

    def render():
        for node in markers:node.remove()
        markers.clear()
        summary=[]
        for label,model in models.items():
            offset=np.array([0.,1.4 if label=='current' else -1.4,0.]) if mode.value.startswith('并排') else np.zeros(3)
            roots[label].position=offset
            roots[label].visible=mode.value.startswith('并排') or mode.value.endswith('两者') or (label=='current' and mode.value.endswith('当前模型')) or (label=='new' and mode.value.endswith('3.3.1'))
            values={name:0. for name in model.actuated_joint_names}
            if label=='current' and pose.value==home_label:
                values.update({name:float(value) for name,value in home.items() if name in values})
            values.update({name:slider.value/factor for (owner,name),(slider,number,factor) in controls.items() if owner==label})
            robots[label].update_cfg(np.array([values[name] for name in model.actuated_joint_names]))
            if roots[label].visible and tools_visible.value:
                for side in ('left','right'):
                    tool=model.get_transform(side+'_tool0',model.base_link)
                    markers.append(server.scene.add_frame(f'/markers/{label}_{side}_tool',position=tool[:3,3]+offset,
                        wxyz=np.roll(Rotation.from_matrix(tool[:3,:3]).as_quat(),1),axes_length=.18,axes_radius=.0035))
                    markers.append(server.scene.add_icosphere(f'/markers/{label}_{side}_tool_origin',position=tool[:3,3]+offset,
                        radius=.009,color=(40,130,255) if label=='current' else (255,120,15)))
                    markers.append(server.scene.add_label(f'/markers/{label}_{side}_tool_label',text=label+' '+side+' tool0',
                        position=tool[:3,3]+offset+[0.,0.,.06],font_screen_scale=.8))
                    summary.append(f"{label} {side} tool0 footprint位置(mm) `{np.round(tool[:3,3]*1000,2).tolist()}`；XYZ轴颜色为红绿蓝。")
            base=model.get_transform('base_link',model.base_link)
            positions=[('footprint',np.zeros(3),(20,220,90)),('base_link',base[:3,3],(240,50,40))]
            if pose.value=='全部零位' and not any(abs(value)>1e-10 for value in values.values()):
                for side in ('left','right'):
                    for name,point in zip(('肩轴心','肘轴心','腕轴心'),geometry[label][side]['shoulder_elbow_wrist_m']):
                        positions.append((side+' '+name,np.asarray(point),(230,200,40)))
            if axes.value and roots[label].visible:
                markers.append(server.scene.add_frame('/markers/'+label+'_footprint_axes',position=offset,axes_length=.3,axes_radius=.005))
                for index,(name,point,color) in enumerate(positions):
                    markers.append(server.scene.add_icosphere(f'/markers/{label}_{index}',radius=.025,position=point+offset,color=color))
                    markers.append(server.scene.add_label(f'/markers/{label}_{index}_label',text=label+' '+name,position=point+offset+[0.,0.,.05],font_screen_scale=.7))
            summary.append(f"**{label}** base_link在footprint下 `{np.round(base[:3,3],6).tolist()}` m；上臂/前臂 `{np.round(geometry[label]['left']['upper_forearm_mm'],3).tolist()}` mm；腕心→tool0 `{geometry[label]['left']['wrist_to_tool_mm']:.3f}` mm。")
        status.content='\n\n'.join(summary)+'\n\n标记大小仅方便看点，不是几何尺寸或碰撞球。'

    @mode.on_update
    @axes.on_update
    @tools_visible.on_update
    def changed(_event):edits.put((None,None))

    @focus_tools.on_click
    def focus_clicked(event):
        if event.client:
            mode.value='同一footprint重合：两者'
            center=np.mean([models[label].get_transform('left_tool0',models[label].base_link)[:3,3] for label in models],axis=0)
            event.client.camera.look_at=center
            event.client.camera.position=center+[.65,-.35,.35]
            event.client.camera.up_direction=(0.,0.,1.)

    @pose.on_update
    def preset_changed(_event):
        values={key:0. for key in controls}
        if pose.value.startswith('同一'):
            for label in models:
                for index,value in enumerate([65,-80,10,80,0,-30,0],1):
                    values[(label,f'left_joint{index}')]=value
                    values[(label,f'right_joint{index}')]=-value
        if pose.value==home_label:
            for key in controls:
                if key[0]=='current':values[key]=float(np.rad2deg(home[key[1]]))
                else:values[key]=controls[key][0].value
        for name,value in values.items():edits.put((name,value))

    @reset_current.on_click
    def reset_clicked(_event):
        for name in controls:edits.put((name,0.))

    @side_view.on_click
    def side_clicked(event):
        if event.client:
            event.client.camera.position=(4.,-4.,1.8);event.client.camera.look_at=(0.,0.,1.1);event.client.camera.up_direction=(0.,0.,1.)

    @front_view.on_click
    def front_clicked(event):
        if event.client:
            event.client.camera.position=(5.,0.,1.7);event.client.camera.look_at=(0.,0.,1.1);event.client.camera.up_direction=(0.,0.,1.)

    @server.on_client_connect
    def connected(client):
        client.camera.position=(4.5,-4.5,3.0);client.camera.look_at=(0.,0.,1.1);client.camera.up_direction=(0.,0.,1.)
    render()
    if args.smoke_test:
        for view in mode.options:
            mode.value=view
            for angle in (0.,15.,0.):
                for (owner,name),(slider,number,factor) in controls.items():
                    slider.value=angle if name=='left_joint2' else 0.
                    number.value=slider.value
                render()
        print('SMOKE PASS: all four views at zero and edited joint2; tool markers regenerated',flush=True)
        server.stop()
        return
    print('Ready http://127.0.0.1:8152 · loaded geometry counts '+str({key:len(model.scene.geometry) for key,model in models.items()}),flush=True)
    while True:
        changed=False
        while not edits.empty():
            name,value=edits.get_nowait()
            if name is not None:
                slider,number,factor=controls[name]
                value=float(np.clip(value,slider.min,slider.max))
                if slider.value!=value:slider.value=value
                if number.value!=value:number.value=value
            changed=True
        if changed:render()
        time.sleep(.03)


if __name__=='__main__':main()
