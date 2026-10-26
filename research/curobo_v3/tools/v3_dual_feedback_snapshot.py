import io
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import viser
from viser.extras import ViserUrdf
import yourdfpy
from v3_dual_zero_review import load_review,old_to_review,review_definition


NAMES=['left_joint1','right_joint6','right_joint5','left_joint3','right_joint3','right_joint2',
    'right_joint1','left_joint6','left_joint4','left_joint2','right_joint4','left_joint7','right_joint7','left_joint5']
POSITIONS=[-.0002636529479178584,1.5708922005941393,-.00011984224905337015,-1.5708562479194232,
    -1.5708682321443288,-.0005872270203624019,-.000623179695078413,1.5708562479194237,
    -.00027563717282319544,-.00017976337358049932,-.0002516687230125214,-.0002396844981071844,
    .00005992112452668508,.00008388957433735911]


def main():
    root=Path(__file__).resolve().parents[1]
    source=root/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf'
    model=load_review(source)
    output=root/'generated/dual_zero_review_20261024'
    output.mkdir(parents=True,exist_ok=True)
    review_definition(source).write(output/'robot_dual_zero_review.urdf',encoding='utf-8',xml_declaration=True)
    raw=old_to_review(dict(zip(NAMES,POSITIONS)))
    server=viser.ViserServer(host='127.0.0.1',port=8176,label='双臂反馈角度对照 · tool0 191mm')
    server.gui.configure_theme(control_width='large',show_share_button=False)
    robot=ViserUrdf(server,model,root_node_name='/robot')
    server.scene.add_grid('/floor',width=4,height=4)
    server.gui.add_markdown('**已确认零位 · 仅模型，不控制实机**\n\n已删除误加的左J4 −1.3°零位偏置。相对原模型只保留90°/180°整数变换、J2 ±15°和已确认轴方向，不使用拟合小角度补偿。\n\n双臂J6正轴朝上，两侧tool0为Joint7局部+Z 191mm。Updown=-0.3m仅参考，桌面真实安装高度未知。模型已同步当前cuRobo分支，旧规划缓存/实机映射未重新验收，不得直接使用。\n\n“恢复历史快照”按历史坐标换算，非实时；不叠加contact-fit补偿。')
    server.gui.add_markdown('**离线关节调姿**：左右14轴均可拖动或在精确值中输入角度；使用±180°诊断范围，可能超出规划限位，不代表允许实机执行。页面无发送接口。')
    controls={}
    for side,title in [('left','左臂'),('right','右臂')]:
        with server.gui.add_folder(title+' J1～J7（仅模型）'):
            for index in range(1,8):
                name=f'{side}_joint{index}'
                initial=-90. if index==3 else 90. if index==6 else 0.
                slider=server.gui.add_slider(f'J{index} / 度',min=-180.,max=180.,step=.1,initial_value=initial)
                number=server.gui.add_number(f'J{index}精确值 / 度',initial_value=initial,step=.1,min=-180.,max=180.)
                def bind(slider,number):
                    @slider.on_update
                    def dragged(_event):
                        if number.value!=slider.value:number.value=slider.value
                    @number.on_update
                    def typed(_event):
                        if np.isfinite(number.value) and -180<=number.value<=180 and slider.value!=number.value:slider.value=number.value
                bind(slider,number)
                controls[name]=(slider,number)
    restore=server.gui.add_button('恢复历史快照（换算到新零位）')
    zero=server.gui.add_button('恢复你确认的新零位（双臂全零）')
    pose_text=server.gui.add_markdown('')
    @restore.on_click
    def restore_snapshot(_event):
        for name,(slider,number) in controls.items():slider.value=number.value=float(np.rad2deg(raw[name]))
    @zero.on_click
    def zero_preview(_event):
        for slider,number in controls.values():slider.value=number.value=0.
    alpha=server.gui.add_checkbox('半透明查看关节',initial_value=False)
    lift=server.gui.add_slider('参考Updown / m',min=-.6,max=0.,step=.01,initial_value=-.3)
    info=server.gui.add_markdown('')
    nodes={}
    j6_nodes={side:dict(line=server.scene.add_line_segments('/j6_axis/'+side,points=np.zeros((1,2,3)),colors=(20,180,80),line_width=6),
        label=server.scene.add_label('/j6_axis_label/'+side,text=side+' J6正轴↑')) for side in ('left','right')}
    for side in ('left','right'):
        nodes[side]=dict(tcp=server.scene.add_frame('/tcp/'+side,axes_length=.12,axes_radius=.003),
            center=server.scene.add_icosphere('/j7/'+side,radius=.012,color=(255,140,20)),
            tip=server.scene.add_icosphere('/tip/'+side,radius=.012,color=(220,40,220)),
            line=server.scene.add_line_segments('/length/'+side,points=np.zeros((1,2,3)),colors=(220,40,220),line_width=5),
            label=server.scene.add_label('/label/'+side,text=side+' tool0 · J7+Z 191mm'))
    @alpha.on_update
    def opacity(_event):
        for handle in robot._meshes.values() if isinstance(robot._meshes,dict) else robot._meshes:
            handle.opacity=.3 if alpha.value else 1.
    @server.on_client_connect
    def connect(client):
        client.camera.position=(3.,3.,2.5);client.camera.look_at=(.3,0.,1.25);client.camera.up_direction=(0.,0.,1.)
    previous=None
    print('READY http://127.0.0.1:8176/',flush=True)
    while True:
        signature=(lift.value,*(control[0].value for control in controls.values()))
        if previous!=signature:
            values={name:np.deg2rad(controls[name][0].value) if name in controls else raw.get(name,0.) for name in model.actuated_joint_names};values['updown']=lift.value
            robot.update_cfg(np.asarray([values[name] for name in model.actuated_joint_names]))
            carriage=model.get_transform('arm_carriage',model.base_link)
            lines=[]
            for side,group in nodes.items():
                j6=model.joint_map[side+'_joint6']
                j6_pose=model.get_transform(j6.child,model.base_link)
                direction=j6_pose[:3,:3]@j6.axis
                endpoint=j6_pose[:3,3]+direction*.16
                j6_nodes[side]['line'].points=np.array([[j6_pose[:3,3],endpoint]])
                j6_nodes[side]['label'].position=endpoint
                joint=model.get_transform(side+'_link7',model.base_link);tcp=model.get_transform(side+'_tool0',model.base_link)
                group['center'].position=joint[:3,3];group['tip'].position=tcp[:3,3]
                group['tcp'].position=tcp[:3,3];group['tcp'].wxyz=np.roll(Rotation.from_matrix(tcp[:3,:3]).as_quat(),1)
                group['line'].points=np.array([[joint[:3,3],tcp[:3,3]]]);group['label'].position=tcp[:3,3]+[0,0,.06]
                assert abs(np.linalg.norm(tcp[:3,3]-joint[:3,3])-.191)<1e-8
                lines.append(f"**{side} tool0**：模型地面XYZ(m) `{np.round(tcp[:3,3],6).tolist()}`；相对安装参考点的世界竖直高度差 `{tcp[2,3]-carriage[2,3]:+.6f}m`。")
            info.content='\n\n'.join(lines)
            pose_text.content='**当前模型角度，可选中文本复制给我**\n\n'+'\n\n'.join(side+'=`'+str([round(controls[f'{side}_joint{i}'][0].value,4) for i in range(1,8)])+'` deg' for side in ('left','right'))
            previous=signature
        time.sleep(.03)


if __name__=='__main__':main()
