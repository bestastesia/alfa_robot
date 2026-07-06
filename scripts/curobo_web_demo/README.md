# ALFA Rerun Web Demo

这是一个最小网页交互回放 demo，用当前 ALFA URDF 生成双臂合成运动轨迹。

## 生成 RRD

```bash
cd /mnt/mydisk/ALFA/alfa_robot
source /mnt/mydisk/anaconda3/etc/profile.d/conda.sh
conda activate curobo_py310
python scripts/curobo_web_demo/generate_alfa_dual_arm_web_rrd.py \
  --save data/curobo_web_demo/alfa_dual_arm_motion_web.rrd
```

## 网页查看

```bash
cd /mnt/mydisk/ALFA/alfa_robot
scripts/curobo_web_demo/serve_alfa_dual_arm_web_demo.sh
```

然后浏览器打开：

```text
http://127.0.0.1:19090
```

如果 19090 被占用：

```bash
PORT=19091 scripts/curobo_web_demo/serve_alfa_dual_arm_web_demo.sh
```

## 注意

- 这是网页可视化链路 demo，不是 cuRobo 真实规划结果。
- 当前 `/mnt/mydisk/ALFA/curobo` 目录为空，之前 cuRobo 生成的 `alfa_dual_tool_motion_plan.rrd` 不在当前文件系统里。
- 后续 cuRobo demo 恢复后，只需要把生成的 `.rrd` 换成 cuRobo 规划输出，同样可以用 `rerun xxx.rrd --web-viewer` 网页查看。
