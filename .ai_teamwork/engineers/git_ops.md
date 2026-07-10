# Git 操作工程师长期注意事项

## 核心目标

保证多人/多 AI 修改可以安全提交、回退、追踪，不把未归属改动混入提交。

## 长期关注

- 提交前必须看 `git status --short --untracked-files=all`。
- 只 stage 当前任务范围内文件。
- 仓库里经常会有生成目录、日志、未归属文件，不要顺手提交。
- 未经用户明确要求，不主动 `git commit` 以外的危险操作，如 reset、clean、rebase、force push。
- 提交信息尽量简短清晰，说明任务目的。
- AI 负责或协助产生的提交，必须在提交信息中标明对应 AI 协作方；例如 Codex 协作使用 `Co-Authored-By: Codex <codex@openai.com>`，其他 AI 按实际身份填写，不能冒用。
- 接手历史提交管理时，先查看近期提交署名风格，保持 AI 协作标记一致。
- 后续提交信息统一使用中文书写；允许 `feat:`、`fix:`、`chore:`、`docs:` 等 conventional commit 类型前缀使用英文，但冒号后的标题和正文应使用中文；历史英文提交暂不处理，除非用户明确要求改写。

## Linear issue 与 Git 分支对应规范

详细规范见 `.ai_teamwork/LINEAR_WORKFLOW.md`；本文件只保留 Git 操作工程师必须记住的要点。

这是重要流程：只要任务来自 Linear 且本次提交直接对应该 issue 的验收目标、修复项或推进内容，Git 分支、commit、PR 都必须能反查到对应 issue。

如果提交只是分支内常规维护、协作区整理、架构清理、通用重构，且不直接对应任何 Linear issue，不要强行添加 Linear 魔法词。

### 基本规则

- 开发分支名必须包含 Linear issue ID，例如 `TIM-25`。
- commit message 必须包含同一个 Linear issue ID。
- 如果后续有 PR，PR 标题也必须包含 Linear issue ID。
- 不建议直接使用 Linear 自动生成的中文长分支名；优先使用短英文描述，避免脚本、CI、终端兼容问题。

### 推荐格式

```bash
git checkout -b tim-25-baseline-flow
```

commit 示例：

```text
Refs TIM-25: 记录 place_safe 失败候选并补充诊断
```

不要只写 `TIM-25: ...`，Linear 可能不会稳定识别；普通提交默认用 `Refs`，只有确认合并后应关闭 issue 才用 `Fixes/Closes/Resolves`。

PR 标题示例：

```text
TIM-25 定义并跑通水管版 baseline 标准流程
```

### 分支命名建议

```text
<linear-id>-<english-short-description>
```

示例：

```text
tim-25-baseline-flow
tim-26-v5-joint-limits
tim-27-collision-jsonl-export
tim-28-bioik-tip-binding
```

### 责任边界

- 如果用户或 PM 已指定 Linear issue，且提交内容直接服务该 issue，必须使用该 issue ID。
- 如果当前任务还没有 Linear issue，先提醒 PM/用户创建或确认，不要自己随便编 ID。
- 如果一个提交同时涉及多个 issue，commit message 中列出主 issue；正文或 PR 描述里说明其他关联 issue。
- 如果只是本地协作文档整理、分支常规维护、架构清理或通用重构，没有直接对应 Linear issue，可继续按原有中文提交规范，不要强行挂 Linear 魔法词。

### 当前 ALFA 示例

当前 `v5机械臂水管版运控全流程项目推进` 的核心进行中 issue：

```text
TIM-25 T-0001 定义并跑通水管版 baseline 标准流程
```

对应建议分支：

```text
tim-25-baseline-flow
```
