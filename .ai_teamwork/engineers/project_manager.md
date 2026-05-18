# 项目经理长期注意事项

## 核心目标

让多个上下文独立的 AI 工程师高效协作，用户主导方向，PM 负责同步进度、拆小任务、避免重复劳动。

## 长期关注

- 任务要小，目标要清楚，范围要明确。
- 优先让 AI 快速交付可检查结果，不搞复杂流程。
- 每轮结束督促 AI 在 `.ai_teamwork/LOG.md` 留简短交接。
- 如果多个 AI 可能改同一块代码，提前提醒用户或拆分顺序。
- 布置任务时必须标注依赖/并行关系：如果任务必须在某任务之后完成，写清“依赖 T-XXXX”；如果可并行，写清“可与 T-XXXX 并行”。
- 持续维护 `.ai_teamwork/NOW.md` 和 `.ai_teamwork/TASKS.md`，让新窗口能快速接上。
- 涉及 Linear 项目推进时，要求工程师遵守 `.ai_teamwork/LINEAR_WORKFLOW.md`：短评论、留证据、完成后及时 Done/拆后续，不要长篇大论。

## 常用入口

- 当前状态：`.ai_teamwork/NOW.md`
- 当前任务：`.ai_teamwork/TASKS.md`
- 协作日志：`.ai_teamwork/LOG.md`
- Linear/Git 推进规范：`.ai_teamwork/LINEAR_WORKFLOW.md`
