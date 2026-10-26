import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys
import time
from urllib.error import HTTPError
from urllib.request import Request,urlopen

import numpy as np
import viser
from viser.extras import ViserUrdf
import yourdfpy

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from plan_preview import PrefixPreviewPlanner
from smooth_timing import rt_curves,validate_preview


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--backend',default='http://127.0.0.1:8155')
    parser.add_argument('--port',type=int,default=8160)
    parser.add_argument('--preview-only',action='store_true')
    args=parser.parse_args()
    engine=PrefixPreviewPlanner(args.root)
    options=dict(engine.catalog())
    server=viser.ViserServer(host='127.0.0.1',port=args.port,label='全部任务 · 平滑前缀预览后执行左臂')
    server.gui.configure_theme(control_width='large',show_share_button=False)
    model=yourdfpy.URDF.load(args.root/'generated/zero_model_candidate_20261018/robot_rezero_candidate.urdf',load_meshes=True)
    home=json.loads((args.root/'config/rezero_home.json').read_text())
    robot=ViserUrdf(server,model,root_node_name='/robot',mesh_color_override=(150,190,220))
    server.scene.add_grid('/floor',width=4,height=4)
    server.gui.add_markdown(f'**{len(options)}组任务 · 计算→预览→确认→执行**\n\n只发送左7轴Home→接触及逆序返回；右臂/升降/真空/抽离/运输均不发送。右单臂任务可预览但不能发送左臂。\n\n10Hz、≤1000点、≤15°/s、≤60秒；当前仍按60秒，超时仅预览。参考碰撞通过不等于未反馈右臂/升降的实机安全保证。')
    first=next((label for label,boxes in options.items()if boxes==dict(left=18,right=22)),next(iter(options)))
    task=server.gui.add_dropdown('组合任务 / 单臂任务',options=tuple(options),initial_value=first)
    speed=server.gui.add_dropdown('最高速度',options=('15°/s','10°/s','5°/s'),initial_value='15°/s')
    calculate=server.gui.add_button('计算平滑接触前缀并验收')
    progress=server.gui.add_markdown('选择任务后点击计算；失败不会提供执行入口。')
    play=server.gui.add_checkbox('播放离线预览',initial_value=False)
    timeline=server.gui.add_slider('预览时间 / s',min=0.,max=1.,step=.01,initial_value=0.)
    reset=server.gui.add_button('预览回到初始')
    confirm=server.gui.add_checkbox('已检查本次预览，仅执行左7轴',initial_value=False)
    lease=server.gui.add_button('获取18121控制权')
    forward=server.gui.add_button('发布已预览轨迹：左臂到接触',disabled=True)
    backward=server.gui.add_button('一键沿本次预览回Home',disabled=True)
    cancel=server.gui.add_button('取消实机Action',disabled=True)
    result=server.gui.add_markdown('未发送实机动作。')
    actual=server.gui.add_markdown('正在读取RT状态…')
    if args.preview_only:
        server.gui.add_markdown('**当前只读预览版：GPU验碰接入遇到显存不足，暂停新规划与发布。**\n119组可选，六组已保存前缀可做平滑预览；其余显示待规划。旧8155执行版本未替换。')
    state=dict(payload=None,curves=None,future=None,status={},seen=False,last_render=None)
    executor=ThreadPoolExecutor(max_workers=1)
    status_executor=ThreadPoolExecutor(max_workers=1)
    status_future=None

    def request(path,data=None):
        body=None if data is None else json.dumps(data).encode()
        query=Request(args.backend+path,data=body,headers={'Content-Type':'application/json','X-Alfa-Request':'left-prefix'})
        try:
            with urlopen(query,timeout=8)as response:return json.load(response)
        except HTTPError as error:
            try:message=json.load(error).get('error','请求失败')
            except ValueError:message='后端请求失败'
            raise ValueError(message)from None

    def change(_event):
        state['payload']=None;state['curves']=None;state['seen']=False;confirm.value=False;play.value=False
        progress.content='选择已变更，请重新计算预览。'
    task.on_update(change);speed.on_update(change)

    @calculate.on_click
    def compute(_event):
        if state['future']is not None or state['status'].get('busy'):return
        confirm.value=False;play.value=False;state['payload']=None;state['seen']=False
        boxes=dict(options[task.value]);velocity=float(speed.value.split('°')[0])
        task.disabled=speed.disabled=calculate.disabled=True
        compute_method=engine.compute_offline if args.preview_only else engine.compute
        state['future']=executor.submit(compute_method,boxes,velocity,lambda text:setattr(progress,'content',text))

    @reset.on_click
    def rewind(_event):play.value=False;timeline.value=0.

    def control(path,data):
        try:response=request(path,data);result.content=json.dumps(response,ensure_ascii=False)
        except Exception as error:result.content='**未发送/操作失败：** '+str(error)

    @lease.on_click
    def acquire(_event):control('/api/release'if state['status'].get('lease')=='you'else'/api/acquire',{})

    @forward.on_click
    def send(_event):
        payload=state['payload']
        if not payload or not confirm.value or not state['seen']:return
        try:
            validate_preview(payload)
            request('/api/prepare',payload)
            control('/api/move',dict(task=payload['task'],direction='forward',preview_id=payload['preview_id']))
        except Exception as error:result.content='**禁止发送：** '+str(error)

    @backward.on_click
    def return_home(_event):
        if state['payload']:control('/api/move',dict(direction='home',preview_id=state['payload']['preview_id']))

    @cancel.on_click
    def stop(_event):control('/api/stop',{})

    @server.on_client_connect
    def connected(client):
        client.camera.position=(2.4,2.7,2.1);client.camera.look_at=(.25,.2,1.25);client.camera.up_direction=(0.,0.,1.)

    clock=time.monotonic();last_status=0.
    print(f'READY http://127.0.0.1:{args.port}/ {len(options)} tasks',flush=True)
    while True:
        now=time.monotonic();elapsed=now-clock;clock=now
        future=state['future']
        if future is not None and future.done():
            try:
                payload=future.result();state['payload']=payload;state['curves']=rt_curves(payload['points']);state['last_render']=None
                timeline.max=payload['report']['duration_s'];timeline.value=0.
                report=payload['report']
                progress.content=(f"**预览计算完成 · {payload['task']}**\n\n{report['duration_s']:.1f}s / {report['points']}点 / 10Hz\n\n"
                    f"峰值速度{report['peak_velocity_deg_s']:.2f}°/s、加速度{report['peak_acceleration_deg_s2']:.2f}°/s²、jerk{report['peak_jerk_deg_s3']:.2f}°/s³\n\n"
                    f"最大局部偏离{report['maximum_joint_deviation_deg']:.3f}°；"
                    +(f"参考碰撞检查{report['collision_samples']}状态通过。"if report['collision_checked']else'尚未完成本入口碰撞复核，只能预览。')
                    +(' **超过60秒，只能预览。**'if not report['within_duration']else''))
            except Exception as error:
                progress.content='**计算失败，禁止执行：** '+type(error).__name__+': '+str(error)
                state['payload']=None;state['curves']=None
            state['future']=None;task.disabled=speed.disabled=calculate.disabled=False
        if play.value and state['curves']:
            timeline.value=min(timeline.max,timeline.value+elapsed)
            state['seen']=True
            if timeline.value>=timeline.max:play.value=False
        values={name:0. for name in model.actuated_joint_names};values['updown']=home['updown_m']
        for side in ('left','right'):values.update({f'{side}_joint{index}':np.deg2rad(angle)for index,angle in enumerate(home[side+'_deg'],1)})
        if state['curves']:
            values.update({f'left_joint{index}':float(curve(timeline.value))for index,curve in enumerate(state['curves'],1)})
            if timeline.value>0:state['seen']=True
        signature=(task.value,timeline.value,bool(state['curves']))
        if signature!=state['last_render']:
            robot.update_cfg(np.array([values[name]for name in model.actuated_joint_names]));state['last_render']=signature
        if status_future is not None and status_future.done():
            try:
                state['status']=status_future.result()
                actual.content='**RT状态：** '+(state['status'].get('reason')or'就绪')+'\n\n历史执行消息：'+state['status'].get('message','无')
            except Exception as error:
                state['status']={};actual.content='**执行后端未就绪：** '+str(error)
            status_future=None
        if now-last_status>.5 and status_future is None:
            last_status=now
            status_future=status_executor.submit(request,'/api/status')
        payload=state['payload'];ready=bool(not args.preview_only and payload and confirm.value and state['seen'] and payload['report']['collision_checked'] and payload['report']['within_duration'] and payload['report']['left_task'] and state['status'].get('ready'))
        forward.disabled=not ready
        backward.disabled=not bool(ready and state['status'].get('last_task'))
        cancel.disabled=args.preview_only or not bool(state['status'].get('busy'))
        lease.disabled=args.preview_only or bool(state['status'].get('busy'))
        lease.label='释放18121控制权'if state['status'].get('lease')=='you'else'获取18121控制权'
        time.sleep(.02)


if __name__=='__main__':main()
