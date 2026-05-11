# AI 协作日志

这里记录每个 AI 最近做了什么，方便其他独立上下文的 AI 快速接上。

## 2026-05-12 项目经理 / Codex / T-0001
- 做了什么：创建 AI 协作入口，并根据用户反馈从“重流程管理”改成“轻量高效协作”。
- 改了哪里：`AGENTS.md`、`CLAUDE.md`、`.ai_teamwork/START.md`、`.ai_teamwork/NOW.md`、`.ai_teamwork/TASKS.md`、`.ai_teamwork/LOG.md`、`.ai_teamwork/ROLES.md`。
- 验证结果：入口文件能把 Codex/ClaudeCode 引导到 `.ai_teamwork/START.md`。
- 留给下个 AI：先读 `START.md`、`NOW.md`、`TASKS.md`、`LOG.md`；具体任务等待用户/PM 分配。

## 2026-05-12 项目经理 / Codex / 安排 T-0002
- 做了什么：把“提交 AI 轻量协作机制更改”安排给 Git 操作工程师。
- 改了哪里：更新 `.ai_teamwork/TASKS.md`，追加本日志。
- 验证结果：任务范围已限定为 `AGENTS.md`、`CLAUDE.md`、`.ai_teamwork/`。
- 留给下个 AI：提交时不要带入 `.windsurfrules`、`lidar_ws/log/*` 等未归属改动。

## 2026-05-12 项目经理 / Codex / 补充工程师长期注意事项
- 做了什么：按用户反馈恢复每个工程师自己的长期注意事项 md，但保持轻量。
- 改了哪里：新增 `.ai_teamwork/engineers/`，更新 `START.md`、`ROLES.md`、`README.md`、`TASKS.md`。
- 验证结果：启动仍然只要求先读 `START.md`，具体岗位文件按任务需要再读。
- 留给下个 AI：承担某岗位任务时，先读 `.ai_teamwork/engineers/` 下对应文件。

## 2026-05-12 Git 操作工程师 / Codex / T-0002
- 做了什么：提交 AI 轻量协作机制相关文件，并按任务要求限定提交范围。
- 改了哪里：`AGENTS.md`、`CLAUDE.md`、`.ai_teamwork/`（含 `engineers/`）。
- 验证结果：提交前核对 `git status --short --untracked-files=all`，未纳入 `lidar_ws/log/*` 未归属文件。
- 留给下个 AI：后续开工继续先读 `AGENTS.md` 和 `.ai_teamwork/START.md`。
