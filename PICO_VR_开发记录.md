# PICO VR 双臂控制 + ros_env 库兼容性修复 开发记录

> 日期：2026-05-07 ~ 2026-05-11
> 关联工程：`ocs2ros2_ws`
> 参考项目：`vr_bimanual_teleop`

---

## 目录

- [1. 概述](#1-概述)
- [2. 系统架构](#2-系统架构)
- [3. PICO SDK 双臂控制实现](#3-pico-sdk-双臂控制实现)
- [4. 坐标系变换](#4-坐标系变换)
- [5. 参考帧校准](#5-参考帧校准)
- [6. RViz 可视化 (PICO TF 帧)](#6-rviz-可视化-pico-tf-帧)
- [7. ros_env 库兼容性修复](#7-ros_env-库兼容性修复)
- [8. 问题排查与修复](#8-问题排查与修复)
- [9. 完整参数说明](#9-完整参数说明)
- [10. 创建/修改的文件清单](#10-创建修改的文件清单)
- [11. 运行方式](#11-运行方式)
- [12. 恢复 mamba tinyxml2](#12-恢复-mamba-tinyxml2)

---

## 1. 概述

本次开发实现了完整的 **PICO VR 手柄 → OCS2 MPC/MRT 双臂末端位姿控制** 链路：

1. PICO SDK 数据读取与坐标系变换
2. 参考帧自动校准（读取机器人 link TF 作为偏移基准）
3. RViz 中 PICO 设备与世界坐标系可视化
4. 修复 `ros_env` mamba 环境中 `tinyxml2` 库版本冲突
5. 修复 `command_right` 跳变（多节点冲突、僵尸进程）
6. 修复 `command_left` TF 不显示

---

## 2. 系统架构

### 2.1 完整数据流

```
PICO 手柄 (xrobotoolkit_sdk)
    → pico_dual_arm_target.py (ROS2 节点, 50Hz)
        ├─ Step 1: R_headset_world (PICO 坐标系 → 世界坐标系)
        ├─ Step 2: T_ref 偏移 (参考帧世界位姿 @ PICO 位姿)
        └─ 发布到 /mobile_manipulator_mpc_target (MpcTargetTrajectories, 14D)
            → RosReferenceManager → MPC 求解器 (SLQ, 100Hz)
                → /mobile_manipulator_mpc_policy → MRT 跟踪 (400Hz)
                    → MobileManipulatorDummyVisualization
                        ├─ command_right TF (右手末端命令位姿)
                        ├─ command_left  TF (左手末端命令位姿)
                        └─ 优化轨迹可视化
```

### 2.2 VR Launch 文件结构

```
manipulator_mabi_mobile_*_vr_ar.launch.py
├─ visualize.launch.py      (robot_state_publisher + rviz2)
├─ mobile_manipulator_mpc_node         (MPC 求解器)
├─ mobile_manipulator_dummy_mrt_node   (MRT 跟踪 + Visualization)
└─ pico_dual_arm_target               (PICO → MPC target 桥接)
```

**注意**：VR launch 不再 include `mobile_manipulator.launch.py`（不再启动 InteractiveMarker 目标节点），避免双节点冲突。

---

## 3. PICO SDK 双臂控制实现

### 3.1 PICO SDK API

```python
import xrobotoolkit_sdk as xrt

xrt.init()
left_pose  = xrt.get_left_controller_pose()   # [x,y,z,qx,qy,qz,qw]
right_pose = xrt.get_right_controller_pose()
head_pose  = xrt.get_headset_pose()
timestamp  = xrt.get_time_stamp_ns()
xrt.close()
```

- SDK 通过 `127.0.0.1:60061` 连接 PICO PC Service (`RoboticsServiceProcess`)
- 初始化后约 0.5~2 秒数据流到达
- `timestamp > 0` 表示真实追踪数据在流动（`_pico_data_active()` 检查）

### 3.2 14D Target Vector 定义

```
[右手 x, y, z, qx, qy, qz, qw, 左手 x, y, z, qx, qy, qz, qw]
```

四元数顺序为 Eigen `coeffs()` 顺序 `[q_x, q_y, q_z, q_w]`。

### 3.3 `pico_dual_arm_target.py` 节点

- **位置**：`src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/scripts/pico_dual_arm_target.py`
- **订阅**：`/{robot_name}_mpc_observation` (MpcObservation)
- **发布**：`/{robot_name}_mpc_target` (MpcTargetTrajectories, 14D)
- **发布 (TF)**：`pico_headset`, `pico_right`, `pico_left` (50Hz)
- **行为**：
  - 连接 PICO SDK 并检测数据活跃度（`timestamp > 0`）
  - 通过 TF2 捕获参考帧初始世界位姿
  - 定时器回调中读取左右手柄位姿，经坐标系变换 + 参考帧偏移后发布
  - PICO 未连接时 (`_pico_data_active() == False`) 不发布任何数据

---

## 4. 坐标系变换

### 4.1 变换管道

```
PICO 原始位姿 (headset tracking frame: X=右, Y=上, Z=前)
    ↓
Step 1: R_headset_world  固定矩阵
    PICO X → World -Y
    PICO Y → World +Z
    PICO Z → World -X
    ↓
Step 2: T_ref 偏移 (参考帧世界位姿)
    T_result = T_ref @ T_world
    即：位置 = R_ref * 位置 + ref_pos
       姿态 = ref_quat * pico_quat
    ↓
发布到 MPC target
```

### 4.2 R_headset_world 矩阵

对应 `vr_bimanual_teleop/dual_arm_teleop.cpp` 中的 `R_headset_world_`：

```python
_R_headset_world = np.array([
    [ 0,  0, -1],   # PICO X → World -Z (实际：World -Y)
    [-1,  0,  0],   # PICO Y → World -X (实际：World +Z)
    [ 0,  1,  0],   # PICO Z → World +Y (实际：World -X)
], dtype=float)
```

注：注释来自 C++ 原始代码。实际映射为 PICO X → World -Y, PICO Y → World +Z, PICO Z → World -X，由测试验证。

### 4.3 参考帧偏移 (T_ref)

从 task.info 的 `picoReferenceFrame` 读取参考 link 名，通过 TF2 在启动时捕获其在 world 坐标系中的初始 4x4 变换矩阵 `T_ref`。

应用于所有 PICO 位姿：`target = T_ref @ pico_world_pose`

这使得 PICO 坐标自动与机器人的实际空间位置对齐。

---

## 5. 参考帧校准

### 5.1 task.info 配置

在所有 task.info 的 `model_information` 段中添加：

```ini
picoReferenceFrame  "waist_yaw_link"
```

设置为 `""` 可禁用校准（回退到仅固定 R_headset_world 变换）。

### 5.2 校准逻辑

1. 节点启动后，通过 TF2 监听参考帧 (`waist_yaw_link`) 的 world 坐标变换
2. 捕获成功后记录为 `T_ref`（4x4 矩阵）
3. 所有 PICO 位姿在 Step 1 后乘以 `T_ref`：`T_result = T_ref @ T_pico_world`
4. 若 5 秒内未捕获到 TF，校准自动禁用

### 5.3 节点参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `robot_name` | `"mobile_manipulator"` | ROS topic 前缀 |
| `publish_rate` | `50.0` | 发布频率 Hz |
| `pico_reference_frame` | `""` | 参考校准 link 名 |
| `z_adjust_count` | `0` | 遗留：Z 轴 90° 旋转次数 (0-3) |

---

## 6. RViz 可视化 (PICO TF 帧)

PICO 节点以 50Hz 广播 3 个 TF 帧（raw，不加 bias）：

| TF 帧 | 父帧 | 说明 |
|---|---|---|
| `pico_headset` | `world` | PICO 头显位置（展示 VR 空间原点） |
| `pico_right` | `world` | 右手柄原始位姿 |
| `pico_left` | `world` | 左手柄原始位姿 |

在 RViz 中添加 TF 显示并勾选对应帧即可观察 VR 空间与机器人的相对位置。

MPC 命令 TF 帧（由 `MobileManipulatorDummyVisualization` 发布）：

| TF 帧 | 说明 |
|---|---|
| `command_right` | 右手末端 MPC 目标 |
| `command_left` | 左手末端 MPC 目标 |

---

## 7. ros_env 库兼容性修复

### 7.1 问题根因

MPC 节点启动即 SIGSEGV。GDB backtrace 显示崩溃在 `PinocchioGeometryInterface::buildGeomFromPinocchioInterface` 的 `strlen` 调用。

**根因**：mamba `ros_env` 中的 `tinyxml2` 11.0.0 (conda-forge) 与系统 `libtinyxml2-10` 10.0.0 同时加载到同一进程：

- OCS2 C++ 代码 `#include <tinyxml2.h>`，编译时使用 mamba v11 头文件
- `liburdfdom_model.so` (系统 ROS2) 编译时使用系统 v10 头文件
- `urdf::exportURDF()` 返回 v10 布局的 `XMLDocument`，但代码按 v11 布局访问 → 内存损坏 → SIGSEGV

`ldd` 确认两套 `.so` 同时加载：
```
libtinyxml2.so.11 => .../ros_env/lib/libtinyxml2.so.11
libtinyxml2.so.10 => /lib/x86_64-linux-gnu/libtinyxml2.so.10
```

### 7.2 修复步骤

#### Step 1：安装系统版 ROS2 pinocchio 等包

```bash
sudo apt install -y ros-jazzy-pinocchio ros-jazzy-coal ros-jazzy-hpp-fcl \
  ros-jazzy-eigenpy ros-jazzy-sdformat-vendor
```

系统版 pinocchio 用系统 tinyxml2 v10 编译，与 ROS2 其他库 ABI 一致。

#### Step 2：隐藏 mamba 的 tinyxml2 头文件和 CMake 配置

```bash
# 隐藏头文件 → 编译器使用系统 /usr/include/tinyxml2.h (v10)
mv $CONDA_PREFIX/include/tinyxml2.h $CONDA_PREFIX/.tinyxml2_backup/

# 隐藏 CMake config → CMake 使用系统 /usr/lib/.../cmake/tinyxml2/
mv $CONDA_PREFIX/lib/cmake/tinyxml2 $CONDA_PREFIX/.tinyxml2_backup/
```

**保留** mamba 的 `libtinyxml2.so.11` 不删除 → 其他 mamba 包运行时仍需它。

#### Step 3：用正确的 CMake 前缀路径重建

```bash
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH="/opt/ros/jazzy:$CONDA_PREFIX:$CMAKE_PREFIX_PATH"

colcon build --symlink-install \
  --packages-up-to ocs2_mobile_manipulator_ros \
  --cmake-args \
  -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

关键点：
- `CMAKE_PREFIX_PATH` 中 `/opt/ros/jazzy` 排在 `$CONDA_PREFIX` 之前
- `CMAKE_POLICY_VERSION_MINIMUM=3.5` 解决新版 CMake 兼容问题
- `--packages-up-to ocs2_mobile_manipulator_ros` 只编译所需依赖链，跳过无关失败包

### 7.3 验证结果

| 检查项 | 状态 |
|---|---|
| `ldd` 只加载 `libtinyxml2.so.10` (不再加载 v11) | ✓ |
| `readelf -d` NEEDED 为 `libtinyxml2.so.10` | ✓ |
| MPC 节点启动无 SIGSEGV | ✓ |
| MRT loop 收到 initial policy | ✓ |
| PICO 节点数据读取正常 | ✓ |
| 全系统持续运行不崩溃 | ✓ |

---

## 8. 问题排查与修复

### 8.1 command_right 跳变

**现象**：未接 PICO 时 `command_right` TF 仍在跳变。

**根因**：
1. **多节点冲突**：原 VR launch include 了 `mobile_manipulator.launch.py`，同时启动了 `mobile_manipulator_target` (InteractiveMarker) 和 `pico_dual_arm_target`，两个节点往同一 topic 发数据
2. **僵尸进程**：多次 launch 遗留了 4 个旧 PICO 节点进程，同时往 `mobile_manipulator_mpc_target` 发数据
3. **PICO PC Service 缓存**：`RoboticsServiceProcess` 一直在后台运行，提供旧追踪数据

**修复**：
- VR launch 不再 include `mobile_manipulator.launch.py`，直接启动 MPC + MRT + PICO
- 添加 `_pico_data_active()` 检查（`timestamp > 0` 才发布）
- 清理所有僵尸进程
- 重启 RoboticsServiceProcess

### 8.2 command_left 不显示

**现象**：RViz 中看不到 `command_left` TF。

**根因**：`MobileManipulatorDummyVisualization` 的 `enableDualEeTarget_` 成员默认 `false`，MRT 创建 visualization 时未将 task.info 中的 `enableDualEeTarget=true` 传入。

**修复**：
- `MobileManipulatorDummyVisualization.h`：添加 `setEnableDualEeTarget(bool)` setter
- `MobileManipulatorDummyMRT.cpp`：创建 visualization 后调用 `setEnableDualEeTarget(enableDualEeTarget)`

### 8.3 PICO 坐标不对齐

**现象**：PICO 手柄位姿与机器人空间不匹配。

**修复历程**：
1. 第一版：固定 `R_headset_world` 矩阵 + 手动 `pos_bias` / `ori_bias` 参数
2. 最终版：用 `pico_reference_frame` (可配置) 的 world 位姿作为偏移基准 `T_ref`，自动对齐

---

## 9. 完整参数说明

### 9.1 pico_dual_arm_target 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `robot_name` | string | `"mobile_manipulator"` | ROS topic 前缀 |
| `publish_rate` | double | `50.0` | 发布频率 Hz |
| `pico_reference_frame` | string | `""` | 参考校准 link 名，`""` 禁用 |
| `z_adjust_count` | int | `0` | 遗留 Z 轴旋转步数 (0-3, 每步 90° CW) |

### 9.2 task.info 新增字段

```ini
model_information {
  picoReferenceFrame  "waist_yaw_link"   ; PICO 参考帧，设为 "" 禁用校准
}
```

---

## 10. 创建/修改的文件清单

### Python 脚本

| 文件 | 操作 | 说明 |
|---|---|---|
| `.../scripts/pico_dual_arm_target.py` | **新建** | PICO SDK → MPC target 桥接节点，含坐标系变换、TF2 校准、PICO TF 可视化 |

### Launch 文件

| 文件 | 操作 | 说明 |
|---|---|---|
| `.../launch/manipulator_mabi_mobile_WH00003_vr_ar.launch.py` | **新建** | WH00003 VR/AR 启动（直接启动 MPC+MRT+PICO） |
| `.../launch/manipulator_mabi_mobile_wh04aa_sldasm_vr_ar.launch.py` | **新建** | wh04aa VR/AR 启动 |

### C++ 修改

| 文件 | 操作 | 说明 |
|---|---|---|
| `.../MobileManipulatorDummyVisualization.h` | **修改** | 添加 `setEnableDualEeTarget(bool)` setter |
| `.../MobileManipulatorDummyMRT.cpp` | **修改** | 调用 `setEnableDualEeTarget()` 传入配置值 |
| `.../CMakeLists.txt` | **修改** | 增加 scripts 安装规则 |

### 配置文件

| 文件 | 操作 | 说明 |
|---|---|---|
| `config/WH00003/task_only_shuangbi.info` | **修改** | 添加 `picoReferenceFrame` |
| `config/WH00003/task.info` | **修改** | 添加 `picoReferenceFrame` |
| `config/WH00003/task_no_wheel.info` | **修改** | 添加 `picoReferenceFrame` |
| `config/wh04aa_sldasm/task_only_shuangbi.info` | **修改** | 添加 `picoReferenceFrame` |
| `config/wh04aa_sldasm/task.info` | **修改** | 添加 `picoReferenceFrame` |
| `config/wh04aa_sldasm/task_no_wheel.info` | **修改** | 添加 `picoReferenceFrame` |

### 环境修复

| 文件/操作 | 说明 |
|---|---|
| `$CONDA_PREFIX/include/tinyxml2.h` | 隐藏到 `.tinyxml2_backup/` |
| `$CONDA_PREFIX/lib/cmake/tinyxml2/` | 隐藏到 `.tinyxml2_backup/` |
| 系统包安装 | `ros-jazzy-pinocchio`, `ros-jazzy-coal`, `ros-jazzy-hpp-fcl`, `ros-jazzy-eigenpy`, `ros-jazzy-sdformat-vendor` |

所有 Python/Launch/C++ 文件路径前缀：`src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/`

---

## 11. 运行方式

### 编译

```bash
mamba activate ros_env
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export CMAKE_PREFIX_PATH="/opt/ros/jazzy:$CONDA_PREFIX:$CMAKE_PREFIX_PATH"
colcon build --symlink-install \
  --packages-up-to ocs2_mobile_manipulator_ros \
  --cmake-args \
  -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

### 启动

```bash
# WH00003
ros2 launch ocs2_mobile_manipulator_ros manipulator_mabi_mobile_WH00003_vr_ar.launch.py

# wh04aa
ros2 launch ocs2_mobile_manipulator_ros manipulator_mabi_mobile_wh04aa_sldasm_vr_ar.launch.py
```

### 调试命令

```bash
# 查看 MPC target 数据
ros2 topic echo /mobile_manipulator_mpc_target

# 查看右/左手命令 TF
ros2 run tf2_ros tf2_echo world command_right
ros2 run tf2_ros tf2_echo world command_left

# 查看 PICO TF 帧
ros2 run tf2_ros tf2_echo world pico_headset
ros2 run tf2_ros tf2_echo world pico_right
ros2 run tf2_ros tf2_echo world pico_left

# 查看 target topic 发布者数量（应为 1，仅 PICO 节点）
ros2 topic info /mobile_manipulator_mpc_target
```

---

## 12. 恢复 mamba tinyxml2

如需恢复 mamba 的 tinyxml2（例如其他项目编译依赖 mamba tinyxml2）：

```bash
CONDA_PREFIX=/home/zenbot-ljl/.local/share/mamba/envs/ros_env
BACKUP=$CONDA_PREFIX/.tinyxml2_backup

# 恢复头文件
cp $BACKUP/tinyxml2.h.mamba_backup $CONDA_PREFIX/include/tinyxml2.h

# 恢复 CMake 配置
mkdir -p $CONDA_PREFIX/lib/cmake/tinyxml2
cp $BACKUP/tinyxml2-config.cmake $CONDA_PREFIX/lib/cmake/tinyxml2/
cp $BACKUP/tinyxml2-config-version.cmake $CONDA_PREFIX/lib/cmake/tinyxml2/
cp $BACKUP/tinyxml2-shared-targets.cmake $CONDA_PREFIX/lib/cmake/tinyxml2/
cp $BACKUP/tinyxml2-shared-targets-release.cmake $CONDA_PREFIX/lib/cmake/tinyxml2/
```

恢复后如需重新编译 OCS2，务必再次隐藏 mamba tinyxml2 头文件和 cmake。
