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

## Docker 环境（可复现编译/运行）

仓库内提供了一个 Dockerfile，用于一键搭建 Noetic + OCS2 依赖并完成基础编译：

- Dockerfile： [src/bimanual_ocs2/docker/dockerfile](src/bimanual_ocs2/docker/dockerfile)

它做的事情（简述）：

- 基于 `osrf/ros:noetic-desktop-full`
- 安装 catkin_tools、Pinocchio、hpp-fcl、OCS2 相关依赖
- 拉取上游仓库并在容器内初始化 `/root/ocs2_ws`
- 在容器内用 `RelWithDebInfo` 构建（与本文“RelWithDebInfo 差异”章节一致）

### 1) 构建镜像

在工作区根目录执行：

```bash
docker build \
	-t wh00003-ocs2:noetic \
	-f src/bimanual_ocs2/docker/dockerfile \
	src/bimanual_ocs2/docker
```

> 说明：该 Dockerfile 会在 build 阶段 `git clone` 多个仓库并编译，首次构建耗时较长。

### 2) 启动容器

如果你只需要编译/跑 ROS（不跑 RViz GUI），最简单：

```bash
docker run --rm -it \
	--net=host \
	--name wh00003_ocs2 \
	wh00003-ocs2:noetic
```

如果你需要在容器里跑 RViz（Linux 桌面常见做法，X11）：

```bash
# 在宿主机允许本地 root 访问 X server
xhost +local:root

docker run --rm -it \
	--net=host \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	--name wh00003_ocs2 \
	wh00003-ocs2:noetic
```

### 3) 容器内如何使用

Dockerfile 已在容器内创建并编译了 `/root/ocs2_ws`，并把 `source /root/ocs2_ws/devel/setup.bash` 写入了 `.bashrc`。

一般进入容器后即可直接运行（必要时手动 source 一次）：

```bash
source /root/ocs2_ws/devel/setup.bash
roslaunch ocs2_mobile_manipulator_ros manipulator_mabi_mobile_WH00003.launch
```

### 4)（可选）在容器里应用本 patch repo

该镜像默认拉的是“上游最新”代码；而本仓库补丁要求基线 commit 对齐（见本文“基线（必须对齐）”）。

推荐做法：把本 patch repo 挂载进容器，然后在容器内把 `/root/ocs2_ws/src/ocs2` 切到基线 commit，再应用补丁并重编译。

启动时挂载（把 `/path/to/bimanual_ocs2` 换成你宿主机上的真实路径）：

```bash
docker run -it --gpus all -P \
	--env="NVIDIA_DRIVER_CAPABILITIES=all"\
	--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw\
	--volume=/dev/dri:/dev/dri:rw --device=/dev/snd\
	--device=/dev/dri --env="DISPLAY=$DISPLAY"\
	--net=host\
	-v /path/to/bimanual_ocs2:/root/wh00003_patch_repo:ro\
	--name=wh00003_ocs2 wh00003-ocs2:noetic
```

容器内执行：

```bash
cd /root/ocs2_ws/src/ocs2
git checkout 26386754b8bf31ab78b503971d0cbf4fdbcd7cb4

bash /root/wh00003_patch_repo/apply_patches.sh

cd /root/ocs2_ws
catkin build ocs2_mobile_manipulator_ros
```

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

## 常见问题：不同工作区行为不一致（RelWithDebInfo 差异）

如果你遇到“同一条 `roslaunch` 在 `ocs2_ws` 正常、在另一个 overlay 工作区（例如 `WH000003_ws`）出现 warning/性能问题”，优先检查 **构建类型是否一致**。

### 现象示例

OCS2 里有一个告警：

> `WARNING: The solution time window might be shorter than the MPC delay!`

它本质是在检查：

$$\text{solutionTimeWindow} < 2 \times \text{avg(MPC\_runtime)}$$

因此出现该 warning 往往不是“代码不一样”，而是：

- `solutionTimeWindow` 配得比较短（例如 0.2s）
- 或者该工作区编译没有优化（例如 `CMAKE_BUILD_TYPE` 为空），导致 MPC 每次迭代更慢

在我们的排查中，两边 `task.info` 相同（`solutionTimeWindow = 0.2`），但构建类型不同：

- `ocs2_ws`: `RelWithDebInfo`
- `WH000003_ws`: `CMAKE_BUILD_TYPE` 为空（默认/无优化），更容易触发上述告警

### 用 catkin 命令查看（推荐）

查看工作区的全局 CMake 配置（最直观）：

```bash
catkin config --workspace /root/ocs2_ws | sed -n '1,80p'
catkin config --workspace /root/WH000003_ws | sed -n '1,80p'

# 只看关键行
catkin config --workspace /root/ocs2_ws | grep -n "Additional CMake Args" -n
catkin config --workspace /root/WH000003_ws | grep -n "Additional CMake Args" -n
```

查看某个包最终生效的 `CMAKE_BUILD_TYPE`（以 `ocs2_mpc` 为例，读 CMakeCache 最准确）：

```bash
grep -n '^CMAKE_BUILD_TYPE' /root/ocs2_ws/build/ocs2_mpc/CMakeCache.txt
grep -n '^CMAKE_BUILD_TYPE' /root/WH000003_ws/build/ocs2_mpc/CMakeCache.txt
```

### 按 “catkin build <pkg> --cmake-args … --” 的形式临时覆盖（不改持久配置）

当你只想快速验证（一次性）某个包用 `RelWithDebInfo` 是否能消除告警，可以用 `catkin build` 临时传参：

```bash
# 注意：--cmake-args 会一直收集参数直到遇到 "--"
catkin build --workspace /root/WH000003_ws ocs2_mpc --no-deps \
	--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --

# 或者直接对你要跑的节点包重编译（会带上依赖，时间更久但更接近真实情况）
catkin build --workspace /root/WH000003_ws ocs2_mobile_manipulator_ros \
	--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --
```

### 永久对齐（建议做法）

让 overlay 工作区和 `ocs2_ws` 对齐为 `RelWithDebInfo`：

```bash
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInf

# 清掉受影响包，强制用新 build type 重编译
catkin clean --workspace /root/WH000003_ws -y \
	ocs2_core ocs2_oc ocs2_ddp ocs2_mpc ocs2_ros_interfaces \
	ocs2_mobile_manipulator ocs2_mobile_manipulator_ros

catkin build --workspace /root/WH000003_ws ocs2_mobile_manipulator_ros
```

完成后再验证：

```bash
source /root/WH000003_ws/devel/setup.bash
roslaunch ocs2_mobile_manipulator_ros manipulator_mabi_mobile_WH00003.launch
```

