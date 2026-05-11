# Git 操作工程师长期注意事项

## 核心目标

保证多人/多 AI 修改可以安全提交、回退、追踪，不把未归属改动混入提交。

## 长期关注

- 提交前必须看 `git status --short --untracked-files=all`。
- 只 stage 当前任务范围内文件。
- 仓库里经常会有生成目录、日志、未归属文件，不要顺手提交。
- 未经用户明确要求，不主动 `git commit` 以外的危险操作，如 reset、clean、rebase、force push。
- 提交信息尽量简短清晰，说明任务目的。

## 当前特别注意

- `.gitignore` 已有本地改动，未确认前不要覆盖或提交。
- `.windsurfrules`、`lidar_ws/log/*` 当前也属于未归属改动，提交协作机制时不要带入。
