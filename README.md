# WH00003 OCS2 改动补丁（patch repo）

这个仓库只包含对上游 OCS2 的改动补丁（`git format-patch`），用于在不 fork 大仓库的情况下分发/复现你的开发修改。

## 基线（必须对齐）

这些补丁基于上游 `leggedrobotics/ocs2` 的以下 commit 生成：

- base commit: `26386754b8bf31ab78b503971d0cbf4fdbcd7cb4`

如果你的本地 OCS2 不是这个版本，补丁可能出现冲突（可手动解决，但不保证一键成功）。

## 内容概览

- WH00003 / 双臂 target / 腰部约束 / 可视化与 DummyMRT 改造
- Dummy loop 增加 external observation 注入（订阅 `ocs2_msgs/mpc_observation`）
- WH00003 相关 launch/task 配置、文档、topic 检查脚本等

补丁文件位于：`patches/`

## 应用方式（推荐：git am）

1) 获取上游 OCS2 并切到基线 commit

```bash
git clone https://github.com/leggedrobotics/ocs2.git
cd ocs2
git checkout 26386754b8bf31ab78b503971d0cbf4fdbcd7cb4

git clone https://github.com/leggedrobotics/pinocchio.git
git clone https://github.com/leggedrobotics/hpp-fcl.git
```

2) 在该仓库内应用补丁

```bash
# 假设你把本 patch repo 放在 ../wh00003_patch_repo
bash ../wh00003_patch_repo/apply_patches.sh
```

3) 按你的工作区方式编译（例如 ROS1/catkin）

> 说明：编译命令与工作区结构有关；如果你在 catkin 工作区中使用 OCS2，请在工作区里执行 `catkin build`。

## 更新补丁（当你做了新的更改）

你后续对 `ocs2` 做了新的更改后，不需要手工改 `.patch` 文件，推荐按下面流程自动更新：

### 前提

- 你的新改动已经在本地 `ocs2` 仓库里 `git commit`（脚本会拒绝未提交改动的工作区）
- 你的分支是基于本 README 指定的 base commit 开发的

base commit 同时写在本仓库的 `.base_commit` 文件里，更新脚本默认读取它。

### 使用方法

在本 patch repo 根目录执行（把 `--ocs2` 改成你本地 ocs2 仓库路径）：

```bash
./update_patches.sh --ocs2 /root/ocs2_ws/src/ocs2
```

如果确实需要换 base（一般不建议），可以显式指定：

```bash
./update_patches.sh --ocs2 /path/to/ocs2 --base <new_base_commit>
```

脚本运行成功后，提交并推送 patch repo：

```bash
git add patches .base_commit update_patches.sh README.md
git commit -m "Update patches"
git push
```

### 脚本逻辑（`update_patches.sh`）

脚本做的事情（保证可复现、可重放）：

1) 解析参数：`--ocs2` 指向本地 ocs2 仓库；`--base` 指向基线 commit（默认读 `.base_commit`）
2) 校验 ocs2 仓库：
	- 必须是 git 仓库
	- base 必须存在且是 commit
	- ocs2 工作区必须干净（`git status --porcelain` 为空）
	- base 必须是 HEAD 的祖先（`git merge-base --is-ancestor base HEAD`）
3) 覆盖更新补丁：
	- 删除旧的 `patches/*.patch`
	- 执行 `git format-patch --binary --full-index --no-stat base..HEAD -o patches/`
4) 输出生成的补丁列表，并提示你在 patch repo 里提交/推送

## 回退

如果用 `git am` 应用失败：

- 放弃当前 am：`git am --abort`

如果已成功应用但想回退整个补丁：

- `git reset --hard 26386754b8bf31ab78b503971d0cbf4fdbcd7cb4`

