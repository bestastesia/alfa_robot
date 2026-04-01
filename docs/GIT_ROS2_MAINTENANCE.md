# alfa_robot_ws Git / ROS2 日常维护手册

本文档用于记录 `~/alfa_robot_ws` 后续维护和更新时常用的命令。

当前仓库信息：

- 本地目录：`/home/kzoia/alfa_robot_ws`
- 远程仓库：`https://github.com/kkozia188/alfa_robot.git`
- 当前主分支：`master`

## 1. 进入工作区

先进入工作区根目录，后面的 Git 和 ROS2 命令都默认在这里执行。

```bash
cd ~/alfa_robot_ws
```

## 2. 查看当前状态

先检查当前分支、是否有未提交修改，以及哪些文件发生了变化。

```bash
git status
```

查看最近的提交历史，方便确认当前代码版本。

```bash
git log --oneline --graph --decorate -10
```

查看还没有暂存的代码差异。

```bash
git diff
```

查看已经暂存、准备提交的代码差异。

```bash
git diff --staged
```

## 3. 提交本地修改

把当前所有修改加入暂存区，适合明确知道本次变更都要提交时使用。

```bash
git add .
```

只暂存指定文件，适合只提交部分修改时使用。

```bash
git add src/xxx.cpp src/yyy.launch.py
```

提交本次修改，提交信息尽量简短明确。

```bash
git commit -m "修复底盘控制参数"
```

把本地提交推送到 GitHub 远程仓库。

```bash
git push origin master
```

## 4. 从远程同步更新

同步远程 `master` 的最新提交，并尽量保持提交历史整洁。

```bash
git pull --rebase origin master
```

如果拉取前本地还有修改，可以先临时保存，拉取后再恢复。

```bash
git stash
git pull --rebase origin master
git stash pop
```

如果本地修改已经整理好，也可以先提交再拉取。

```bash
git add .
git commit -m "WIP: 本地修改"
git pull --rebase origin master
```

## 5. 新建功能分支开发

新建一个功能分支，适合开发新功能或做较大改动时使用。

```bash
git checkout -b feature/xxx
```

把新分支推送到远程，并建立本地与远程分支的跟踪关系。

```bash
git push -u origin feature/xxx
```

切回主分支继续日常维护。

```bash
git checkout master
```

回到主分支后，先同步远程最新代码。

```bash
git pull --rebase origin master
```

## 6. 查看远程和分支信息

查看当前远程仓库地址，确认 push 和 pull 的目标是否正确。

```bash
git remote -v
```

查看本地分支列表。

```bash
git branch
```

查看本地分支和远程跟踪关系。

```bash
git branch -vv
```

## 7. ROS2 工作区更新依赖

先加载 ROS2 Humble 环境。

```bash
source /opt/ros/humble/setup.bash
```

更新 `rosdep` 依赖数据库。

```bash
rosdep update
```

根据 `src/` 中的包自动安装缺失依赖。

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 8. 编译工作区

在工作区根目录进行编译，并使用软链接安装方式方便开发调试。

```bash
colcon build --symlink-install
```

## 9. 清理旧编译缓存后重新编译

如果工作区来自别的机器，或者出现 `CMakeCache.txt` 路径冲突，先删除旧缓存。

```bash
rm -rf build install log
```

重新加载 ROS2 环境后再编译。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 10. 首次使用 rosdep 时的初始化

如果系统提示 `rosdep installation has not been initialized yet`，先执行初始化。

```bash
sudo rosdep init
rosdep update
```

如果提示已经初始化过了，就直接执行下面这一条即可。

```bash
rosdep update
```

## 11. 推荐日常流程

每天开始工作前，先同步远程并检查状态。

```bash
cd ~/alfa_robot_ws
git status
git pull --rebase origin master
```

修改代码后，更新依赖并重新编译验证。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

确认无误后，再提交并推送到 GitHub。

```bash
git add .
git commit -m "更新说明"
git push origin master
```

## 12. 额外说明

- `build/`、`install/`、`log/` 已在 `.gitignore` 中，通常不需要提交到 GitHub。
- 执行 `git pull` 前，建议先运行 `git status`，避免本地未提交修改与远程更新冲突。
- 如果是大改动，建议先建分支，不要直接在 `master` 上连续做高风险修改。
