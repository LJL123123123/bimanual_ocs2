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
```

2) 在该仓库内应用补丁

```bash
# 假设你把本 patch repo 放在 ../wh00003_patch_repo
bash ../wh00003_patch_repo/apply_patches.sh
```

3) 按你的工作区方式编译（例如 ROS1/catkin）

> 说明：编译命令与工作区结构有关；如果你在 catkin 工作区中使用 OCS2，请在工作区里执行 `catkin build`。

## 回退

如果用 `git am` 应用失败：

- 放弃当前 am：`git am --abort`

如果已成功应用但想回退整个补丁：

- `git reset --hard 26386754b8bf31ab78b503971d0cbf4fdbcd7cb4`

