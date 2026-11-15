import argparse
import json
from pathlib import Path
import queue
import time

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from scipy.interpolate import BPoly,PPoly
import viser
from viser.extras import ViserUrdf
import yourdfpy


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--audit-dir',type=Path,required=True)
    parser.add_argument('--urdf',type=Path,required=True)
    parser.add_argument('--home',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8159)
    args=parser.parse_args()
    records={path.stem:json.loads(path.read_text()) for path in args.audit_dir.glob('L*.json')}
    home=json.loads(args.home.read_text())
    server=viser.ViserServer(host='127.0.0.1',port=args.port,label='原规划帧插值诊断 · 七轴曲线与末段减速')
    server.gui.configure_theme(control_width='large',show_share_button=False)
    model=yourdfpy.URDF.load(args.urdf,load_meshes=True)
    robot=ViserUrdf(server,model,root_node_name='/robot',mesh_color_override=(150,190,220))
    server.scene.add_grid('/floor',width=4,height=4)
    server.gui.add_markdown('**离线诊断：未发送、未验收的候选轨迹**\n保留原规划帧位置，每帧0.1s，差分估计中间速度/加速度，末帧强制v=a=0；曲线为RT五次Hermite插值结果，不是实机加速度测量。\n左臂运动，右臂Home与Updown=-0.3m仅作参考。')
    task=server.gui.add_dropdown('任务',options=tuple(sorted(records)),initial_value='L18_R22')
    play=server.gui.add_checkbox('播放',initial_value=False)
    seconds=server.gui.add_slider('时间 / s',min=0.,max=29.9,step=.001,initial_value=0.)
    jump=server.gui.add_button('定位加速度峰值')
    restart=server.gui.add_button('回初始姿态')
    window=server.gui.add_dropdown('曲线范围',options=('全程','最后1秒','峰值附近0.5秒'),initial_value='最后1秒')
    status=server.gui.add_markdown('')
    explanation=server.gui.add_markdown('')
    with server.gui.add_folder('七个关节的角度曲线'):
        angles_plot=server.gui.add_plotly(go.Figure(),aspect=3.0)
    with server.gui.add_folder('最坏关节：速度与加速度'):
        dynamics_plot=server.gui.add_plotly(go.Figure(),aspect=1.7)
    pending=queue.Queue()
    cache={}

    def prepare(name):
        record=records[name]
        q=np.asarray(record['positions']);v=np.asarray(record['velocities']);a=np.asarray(record['accelerations'])
        ticks=np.arange(len(q))*.1
        curves=[PPoly.from_bernstein_basis(BPoly.from_derivatives(ticks,np.column_stack([q[:,joint],v[:,joint],a[:,joint]]))) for joint in range(7)]
        cache.update(name=name,record=record,q=q,v=v,a=a,ticks=ticks,curves=curves)
        seconds.max=float(ticks[-1]);seconds.value=0.;play.value=False
        redraw()

    def redraw():
        record=cache['record'];ticks=cache['ticks'];peak=record['report']['worst']['acceleration'];peak_time=peak['time_s']
        start,end=(0.,float(ticks[-1])) if window.value=='全程' else (max(0.,float(ticks[-1])-1),float(ticks[-1])) if window.value=='最后1秒' else (max(0.,peak_time-.4),min(float(ticks[-1]),peak_time+.1))
        stamps=np.linspace(start,end,1000 if window.value=='全程' else 400)
        originals=(ticks>=start)&(ticks<=end)
        figure=make_subplots(rows=7,cols=1,shared_xaxes=True,vertical_spacing=.035,subplot_titles=[f'左J{index+1}角度（°）' for index in range(7)])
        for joint,curve in enumerate(cache['curves']):
            figure.add_trace(go.Scatter(x=stamps,y=np.rad2deg(curve(stamps)),mode='lines',name='RT插值',line=dict(color='#37a5db',width=2),showlegend=joint==0),row=joint+1,col=1)
            figure.add_trace(go.Scatter(x=ticks[originals],y=np.rad2deg(cache['q'][originals,joint]),mode='markers',name='原规划帧',marker=dict(size=5,color='#e19b40'),showlegend=joint==0),row=joint+1,col=1)
            figure.add_vline(x=peak_time,line_dash='dot',line_color='#d64545',row=joint+1,col=1)
        figure.update_xaxes(title_text='时间（s）',row=7,col=1)
        figure.update_layout(height=1050,margin=dict(l=55,r=15,t=45,b=45),legend=dict(orientation='h',y=1.055),template='plotly_white')
        angles_plot.figure=figure
        joint=int(peak['joint'].removeprefix('left_joint'))-1;curve=cache['curves'][joint]
        dynamic=make_subplots(rows=2,cols=1,shared_xaxes=True,vertical_spacing=.13,subplot_titles=[f'左J{joint+1}速度（°/s）',f'左J{joint+1}加速度（°/s²）'])
        for row,order,bound in [(1,1,15.),(2,2,30.)]:
            dynamic.add_trace(go.Scatter(x=stamps,y=np.rad2deg(curve.derivative(order)(stamps)),mode='lines',name='RT插值',line=dict(color='#37a5db')),row=row,col=1)
            values=cache['v'] if order==1 else cache['a']
            dynamic.add_trace(go.Scatter(x=ticks[originals],y=np.rad2deg(values[originals,joint]),mode='markers',name='发送点给定值',marker=dict(size=6,color='#e19b40')),row=row,col=1)
            for limit in (-bound,bound):
                dynamic.add_hline(y=limit,line_dash='dash',line_color='#d64545',row=row,col=1)
            dynamic.add_vline(x=peak_time,line_dash='dot',line_color='#d64545',row=row,col=1)
        dynamic.update_layout(height=600,showlegend=False,margin=dict(l=60,r=15,t=45,b=40),template='plotly_white')
        dynamic.update_xaxes(title_text='时间（s）',row=2,col=1)
        dynamics_plot.figure=dynamic
        segment=peak['from_frame']-1
        displacement=np.rad2deg(cache['q'][segment+1,joint]-cache['q'][segment,joint])
        explanation.content=(f"**峰值：{peak['joint']}，{peak_time:.6f}s，第{peak['from_frame']}→{peak['to_frame']}帧**\n\n"
            f"该0.1s内仍要求走{displacement:.4f}°；起点速度{np.rad2deg(cache['v'][segment,joint]):.3f}°/s，终点速度{np.rad2deg(cache['v'][segment+1,joint]):.3f}°/s。\n\n"
            f"整段插值加速度绝对峰值 **{record['report']['peak_deg_units']['acceleration']:.2f}°/s²**。蓝线是帧间真实插值，橙点只是发送点给定值，不能只检查橙点。")

    def render(stamp):
        values={name:0. for name in model.actuated_joint_names};values['updown']=home['updown_m']
        for side in ('left','right'):
            values.update({f'{side}_joint{index}':np.deg2rad(angle) for index,angle in enumerate(home[side+'_deg'],1)})
        q=[curve(stamp) for curve in cache['curves']]
        values.update({f'left_joint{index}':float(value) for index,value in enumerate(q,1)})
        robot.update_cfg(np.array([values[name] for name in model.actuated_joint_names]))
        worst=int(cache['record']['report']['worst']['acceleration']['joint'].removeprefix('left_joint'))-1
        velocity=np.rad2deg(cache['curves'][worst].derivative(1)(stamp));acceleration=np.rad2deg(cache['curves'][worst].derivative(2)(stamp))
        status.content=f"**{task.value} · {stamp:.3f}s · 左J{worst+1} v={velocity:.2f}°/s，a={acceleration:.2f}°/s²**\n\n当前左七轴角度：`{np.round(np.rad2deg(q),2).tolist()}`"

    @task.on_update
    def task_changed(_event):pending.put('task')
    @window.on_update
    def window_changed(_event):pending.put('plot')
    @jump.on_click
    def peak_clicked(_event):pending.put('peak')
    @restart.on_click
    def restart_clicked(_event):pending.put('start')
    @server.on_client_connect
    def connected(client):
        client.camera.position=(2.3,2.7,2.1);client.camera.look_at=(.25,.2,1.25);client.camera.up_direction=(0.,0.,1.)

    prepare(task.value);seconds.value=float(cache['record']['report']['worst']['acceleration']['time_s']);render(seconds.value)
    last=time.monotonic();previous=None
    print(f'READY http://127.0.0.1:{args.port}/ · OFFLINE ONLY',flush=True)
    while True:
        now=time.monotonic();elapsed=now-last;last=now
        while not pending.empty():
            operation=pending.get()
            if operation=='task':prepare(task.value)
            elif operation=='plot':redraw()
            elif operation=='peak':
                play.value=False;seconds.value=float(cache['record']['report']['worst']['acceleration']['time_s']);window.value='峰值附近0.5秒';redraw()
            else:play.value=False;seconds.value=0.
        if play.value:
            seconds.value=min(seconds.max,seconds.value+elapsed)
            if seconds.value>=seconds.max:play.value=False
        if previous!=(task.value,seconds.value):render(seconds.value);previous=(task.value,seconds.value)
        time.sleep(.02)


if __name__=='__main__':
    main()
