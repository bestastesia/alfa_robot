# MOTION-292 当前验收状态

当前实现基于 MOTION-288 / PR #71 `73f40c5`，运行合同见 `docs/motion292_pr71_rnea_integration.md`。

## 已完成

- 滚动接续只使用上游 RNEA 认证采样点。
- 安全旧后缀只做最新场景碰撞复核并复用上游动力学证书。
- 新避障候选支持非零边界 RNEA audit；超限后走 braking + rest-to-rest 固定 q 重定时。
- RNEA 前后 q hash 和逐点最大位置差进入强制门禁。
- 10kg、0.3×0.4×0.4m、90% 持续力矩合同进入 worker、阶段输入和 verifier。
- 完整 torque/gravity 进入异步 sidecar；Viser 已接入三层路径、双影子和动力学曲线。
- CPU 合同测试当前为 141 项。
- 2026-10-29 GPU smoke：原生场景更新1/1、round-0六阶段clear CPU/GPU verifier全部通过；transport静态障碍成功换轨1次，RNEA前后q delta为0，持续力矩利用率0.632。
- 空闲主机15/15轮、25/25箱通过：端到端median 2.306s、p95 3.431s、max 4.533s；所有轮`controller_ready`，最大q delta为0，最大持续力矩利用率0.671。
- 障碍矩阵114/114均可独立audit；将14个deadline样本在空闲环境重跑后，9个转为成功、5个仍按预算安全拒绝，最终50项完成、64项安全拒绝、45次成功避障换轨，`NO_VALID_REPLACEMENT` 1项，安全旧路在线TrajOpt为0，最大q delta为0、最大持续力矩利用率0.676，达到当前门槛。
- clear/CPU压力360/360通过，覆盖3轮hot和1轮4×CPU burner soak；正式样本无deadline miss。另保留6个首次deadline失败尝试，重启worker后同配置通过，不从证据中删除。
- 故障注入8/8按预期fail-closed并通过audit。
- Viser由Edge真实点击并完成整轮六阶段回放；截图和页面文本确认灰/橙/红蓝三层合同、双影子开关、动力学曲线入口、q差0及力矩利用率摘要。
- 冷worker实测旧`t2+1.5s`启动门禁会偶发拒绝；空闲approach最高约3.13s，12线程CPU压力下约3.77s。仅一次性启动余量改为`t2+3.5s`；执行期`t2_rolling`不变。

## 仿真验收结论

当前门槛全部满足：clear/CPU 360/360、fault 8/8、障碍114/114可审计、完成50项、成功避障换轨45次、`NO_VALID_REPLACEMENT` 1项、所有安装轨迹q delta为0、最大持续力矩利用率0.676、安全旧路在线TrajOpt次数为0。

15轮回归曾在外部Blender约1000% CPU负载下触发上游5s `controller_ready`门禁；该污染样本已保留。空闲主机重跑15/15轮通过，没有放宽PR #71的5s门禁。

仍未完成的是实机执行层、真实感知/时钟、保护停机和硬件安全评审。本交付只证明规划与仿真合同，不表示已具备实机安全认证。
