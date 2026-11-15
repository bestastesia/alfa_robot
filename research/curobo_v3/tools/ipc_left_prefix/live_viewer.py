import argparse
import json
from pathlib import Path
import time
from urllib.request import urlopen

import numpy as np
import viser
from viser.extras import ViserUrdf
import yourdfpy
from coordinate_mapping import motor_to_model, calibration_summary, feedback_summary


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--status-url',default='http://127.0.0.1:8155/api/status')
    parser.add_argument('--library',type=Path,required=True)
    parser.add_argument('--urdf',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8156)
    args=parser.parse_args()
    library=json.loads(args.library.read_text())
    policy=Path(__file__).parent/'coordinate_policy.json'
    if policy.is_file():
        library.update(json.loads(policy.read_text()))
    model=yourdfpy.URDF.load(args.urdf,load_meshes=True)
    for name,link in model.link_map.items():
        if name.startswith('right_'):
            link.visuals=[]
    model._scene=model._create_scene()
    server=viser.ViserServer(host='127.0.0.1',port=args.port,label='左臂实测运动 · 只读显示')
    server.gui.configure_theme(control_width='small',show_share_button=False)
    robot=ViserUrdf(server,model,root_node_name='/feedback',mesh_color_override=(140,185,220,.35))
    server.scene.add_grid('/ground',width=3,height=3)
    marker=server.scene.add_icosphere('/target_tcp',radius=.025,color=(70,230,140),visible=False)
    joint7_marker=server.scene.add_icosphere('/joint7_center',radius=.014,color=(255,90,30))
    joint7_axes=server.scene.add_frame('/joint7_axes',axes_length=.10,axes_radius=.003)
    joint7_label=server.scene.add_label('/joint7_label',text='左Joint7关节中心（非已标定法兰面）')
    test_tool_marker=server.scene.add_icosphere('/test_tool_tip',radius=.009,color=(220,40,230))
    test_tool_axes=server.scene.add_frame('/test_tool_axes',axes_length=.055,axes_radius=.002)
    test_tool_label=server.scene.add_label('/test_tool_label',text='临时TCP：Joint7局部+Z 70mm')
    test_tool_line=server.scene.add_line_segments('/test_tool_length',points=np.zeros((1,2,3)),colors=(220,40,230),line_width=5)
    joint7_text=server.gui.add_markdown('')
    closeup=server.gui.add_button('近看左Joint7')

    @closeup.on_click
    def focus_joint7(event):
        center=model.get_transform('left_link7',model.base_link)[:3,3]
        event.client.camera.look_at=center
        event.client.camera.position=center+np.array([.55,.55,.35])

    def update_joint7():
        from scipy.spatial.transform import Rotation
        transform=model.get_transform('left_link7',model.base_link)
        joint7_marker.position=transform[:3,3]
        joint7_axes.position=transform[:3,3]
        joint7_axes.wxyz=np.roll(Rotation.from_matrix(transform[:3,:3]).as_quat(),1)
        joint7_label.position=transform[:3,3]+np.array([0.,0.,.08])
        tip=transform[:3,3]+transform[:3,:3]@np.array([0.,0.,.070])
        test_tool_marker.position=tip
        test_tool_axes.position=tip
        test_tool_axes.wxyz=joint7_axes.wxyz
        test_tool_label.position=tip+np.array([0.,0.,-.055])
        test_tool_line.points=np.array([[transform[:3,3],tip]])
        joint7_text.content='**橙球：左Joint7中心；紫球：临时TCP**\n\n临时TCP相对left_link7：XYZ=[0,0,0.070]m，RPY=[0,0,0]。按用户给定尺寸标记，非实测标定。\n\n'+model.base_link+'下Joint7 XYZ（米）：`'+str(np.round(transform[:3,3],6).tolist())+'`\n\n临时TCP XYZ（米）：`'+str(np.round(tip,6).tolist())+'`\n\n仅可视化；原吸盘tool0及规划/控制配置未改。'

    update_joint7()
    state_text=server.gui.add_markdown('等待左臂实测反馈…')
    server.gui.add_markdown('机器人运动来自实测 /joint_states，不是预设播放。绿球表示所选轨迹的接触目标TCP；仅左7轴。Updown固定-0.3m只用于显示参考，未反馈的轴不代表实机状态。')
    server.gui.add_markdown('**左J5新零位已同步电控，不重复加减180°。** 当前另应用下方小角度拟合补偿。旧吸附轨迹目标因零位变更暂不显示。')
    server.gui.add_markdown(calibration_summary())

    @server.on_client_connect
    def connected(client):
        client.camera.position=(2.3,2.7,2.1)
        client.camera.look_at=(.25,.25,1.25)
        client.camera.up_direction=(0.,0.,1.)

    last=None
    while True:
        try:
            with urlopen(args.status_url,timeout=1) as response:
                status=json.load(response)
            task=status.get('last_task') if status.get('busy') else status.get('view_task')
            task=task if task in library['tasks'] else next(iter(library['tasks']))
            degrees=status.get('feedback_deg')
            if degrees is None or status.get('feedback_age_s',999)>.5:
                state_text.content='**反馈过期：保持最后画面，不当作实时状态**'
            else:
                if task!=last and not library.get('legacy_replay_blocked'):
                    row=library['tasks'][task]['model_positions'][-1]
                    mapping=dict(zip(library['joint_names'],row));mapping['updown']=-.3
                    model.update_cfg({name:mapping.get(name,0.) for name in model.actuated_joint_names})
                    marker.position=model.get_transform('left_tool0',model.base_link)[:3,3]
                    marker.visible=True;last=task
                values=dict(zip(library['joint_names'],motor_to_model(np.deg2rad(degrees))));values['updown']=-.3
                robot.update_cfg(np.array([values.get(name,0.) for name in model.actuated_joint_names]))
                update_joint7()
                state_text.content=f"**左臂实测 · {task} · {status.get('action_phase','idle')}**\n\n反馈年龄 {status['feedback_age_s']:.3f}s\n\n{status.get('message','')}"
                state_text.content += feedback_summary(degrees)
        except Exception as error:
            state_text.content='**后端连接中断，画面不再更新**\n\n'+str(error)
        time.sleep(.05)


if __name__=='__main__':
    main()
