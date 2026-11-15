# 临时左臂平滑预览（未部署执行）

分支：`feature/smooth-prefix-preview`。入口：http://127.0.0.1:8160/ 。

本轮用户要求：临时平滑、扩大任务选择、预览后再明确发布。当前仅交付离线预览，发布未上线。

## 已完成

- `tools/ipc_left_prefix/smooth_timing.py`：局部时间滤波、端点零速度/零加速度、时间拉伸，10Hz输出位置/速度/加速度；按RT实际五次Hermite插值求内部速度、加速度和jerk峰值。限值15°/s、30°/s²、150°/s³，≤1000点。
- 滤波会局部圆滑原折线，并非严格只改变时间；逐轴偏离上限0.5°，首组实测0.213°。需要重新验碰。
- `preview_app.py --preview-only`：119项任务目录（94双臂、25单臂），六组已有前缀能离线播放；其余明确提示待规划。预览版所有实机按钮禁用。
- 六组平滑时长依次：L18/R22 31.9秒、L4/R2 48.4秒、L4/R5 51.8秒、L7/R5 62.3秒、L8/R7 61.9秒、L9/R1 42.6秒。未得到延长至90秒的确认，仍以60秒作为执行门槛，超时只能预览。
- `tests/test_smooth_prefix_preview.py`四项测试通过，包括实际插值内部导数峰值、端点、时间网格、超时与未验碰数据拒绝。

## 验证范围与阻塞

首组单独进程在记录场景及记录的右臂/升降参考状态下，检查6381个200Hz插值状态，碰撞0；证据`/tmp/smooth_prefix_reference_collision.json`。这不等于冻结右臂/升降的实机验收，也不代表新入口已完成复核。

随后集成`PrefixPreviewPlanner.compute`时，碰撞检查器在GPU初始化阶段CUDA显存不足；证据`/tmp/prefix_preview_compute.log`。按用户遇异常先停要求，停止新任务规划、GPU重试及新执行服务部署，没有停止其他工程师进程。8160使用`compute_offline`且明确标记未验碰。

`plan_preview.py`未缓存任务的规划分支及`server.py`预览哈希绑定、准备/发送接口尚未完成集成验证；不能当作已可上线。8155原执行服务未替换，未使能或发送实机运动。待确认GPU资源安排后，先完成新任务规划与逐轨迹复核，再验证预览绑定/返回轨迹及模拟Action，最后部署。

启动当前只读界面：

```bash
cd /mnt/mydisk/ALFA/alfa_robot_v3 && /mnt/mydisk/ALFA/.venvs/curobo-v2-py310/bin/python research/curobo_v3/tools/ipc_left_prefix/preview_app.py --port 8160 --preview-only
```

当前systemd用户服务：`alfa-smooth-prefix-preview-8160.service`，已在运行，不要重复占用同端口。
