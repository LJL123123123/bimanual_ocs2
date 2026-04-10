# ocs2_robotic_examples_WH00003 开发文档

> 版本：开发说明版  
> 适用对象：算法开发、系统集成、调试维护、论文/技术文档整理  
> 关联工程：
>
> - `ocs2_robotic_examples_WH00003`
> - 原版 `ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator`

---

# 目录

- [1. 文档目标](#1-文档目标)
- [2. 工程定位与改造概览](#2-工程定位与改造概览)
- [3. 与原版工程的核心差异](#3-与原版工程的核心差异)
- [4. 系统总体架构](#4-系统总体架构)
- [5. 数据流与控制流](#5-数据流与控制流)
- [6. 目录与模块说明](#6-目录与模块说明)
- [7. 核心改动清单（按文件）](#7-核心改动清单按文件)
- [8. 双臂扩展设计说明](#8-双臂扩展设计说明)
- [9. 腰部直立约束设计说明](#9-腰部直立约束设计说明)
- [10. 目标轨迹接口说明](#10-目标轨迹接口说明)
- [11. Task 配置文件说明](#11-task-配置文件说明)
- [12. ROS 节点说明](#12-ros-节点说明)
- [13. 可视化与调试链路](#13-可视化与调试链路)
- [14. 主要风险点与已知问题](#14-主要风险点与已知问题)
- [15. 开发与修改建议](#15-开发与修改建议)
- [16. 后续扩展建议](#16-后续扩展建议)
- [附录 A：双臂模式关节顺序](#附录-a双臂模式关节顺序)
- [附录 B：双 ee target 维度定义](#附录-b双-ee-target-维度定义)
- [附录 C：流程图与接口关系图占位](#附录-c流程图与接口关系图占位)

---

# 1. 文档目标

本文档用于系统说明 `ocs2_robotic_examples_WH00003` 相比原版 `ocs2_mobile_manipulator` 的开发改动，重点回答以下问题：

1. 该工程相对原版具体改了什么  
2. 每项改动的作用是什么  
3. 双臂能力是如何接入 OCS2 的  
4. 腰部约束是如何接入 OCP 的  
5. ROS 目标输入、DummyMRT、Visualization 如何与双臂模式协同  
6. 当前实现有哪些风险点和后续开发建议

本文档不是用户使用手册，而是**开发维护文档**。

---

# 2. 工程定位与改造概览

## 2.1 原版工程定位

原版 `ocs2_mobile_manipulator` 是一个通用示例工程，主要特点是：

- 面向移动操作器的统一 OCS2 求解框架
- 支持多种示例机器人
- 默认以单个末端位姿目标为主要任务
- 更偏教学/示例，不针对某台具体双臂机器人深度定制

## 2.2 WH00003 工程定位

`ocs2_robotic_examples_WH00003` 面向具体机器人 `WH00003` 的项目化落地需求，目标包括：

- 支持双臂末端位姿控制
- 支持腰部姿态约束
- 支持双 marker 输入左右手目标
- 支持双臂模式的 DummyMRT 与 Visualization
- 提供更适合项目运行的 launch、docker、文档和配置

## 2.3 改造总述

相较原版，本工程的核心改造可以概括为：

- 将**单 ee** tracking 扩展为**可切换单/双 ee**
- 在 OCP 中新增**腰部 roll/pitch 直立约束**
- 将 ROS 交互链路从单目标拓展为双目标
- 将参数和机器人模型全面切换为 WH00003 专用配置
- 将工程从示例性质改造成项目化结构

---

# 3. 与原版工程的核心差异

## 3.1 功能层差异

| 项目 | 原版 `ocs2_mobile_manipulator` | WH00003 版本 |
|---|---|---|
| 机器人配置 | 示例机器人 | WH00003 专用 |
| 末端目标 | 单 ee pose target | 可切换单/双 ee pose target |
| 腰部约束 | 无 | 有，限制 `waist_yaw_link` 的 roll/pitch |
| target 输入 | 单 marker | 双 marker |
| DummyMRT 初始 target | 单手 7 维 | 单/双手 7/14 维切换 |
| Visualization | 单末端显示 | 双末端目标/轨迹显示 |
| 工程形式 | 示例包 | 项目化工程 |

## 3.2 技术层差异

原版默认采用的目标轨迹形式是单个 7 维 pose：

\[
x_{target} = [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
\]

WH00003 双臂模式下改为：

\[
x_{target} =
[p_R(3), q_R(4), p_L(3), q_L(4)] \in \mathbb{R}^{14}
\]

并通过 `targetStateOffset` 机制实现左右手分别读取不同段。

---

# 4. 系统总体架构

## 4.1 架构描述

WH00003 工程可划分为四层：

1. **模型与配置层**
   - URDF
   - `task.info`
   - auto_generated 动态/运动学库

2. **OCP 装配层**
   - `MobileManipulatorInterface`
   - `EndEffectorConstraint`
   - `LinkVerticalConstraint`

3. **ROS 输入输出层**
   - `MobileManipulatorTarget`
   - `MobileManipulatorDummyMRT`
   - `MobileManipulatorDummyVisualization`

4. **运行与部署层**
   - launch
   - docker
   - 文档与脚本

## 4.2 架构关系图（占位）

```text
+-----------------------------+
|        task.info / URDF     |
+-------------+---------------+
              |
              v
+-----------------------------+
| MobileManipulatorInterface  |
| - dynamics                  |
| - cost                      |
| - ee constraints            |
| - waist constraints         |
+-------------+---------------+
              |
              v
+-----------------------------+
|        OCS2 Solver          |
|      (MPC / DDP / SLQ)      |
+-------------+---------------+
              ^
              |
+-------------+---------------+
|    ReferenceManager         |
|  TargetTrajectories         |
+-------------+---------------+
              ^
              |
+-------------+---------------+
| MobileManipulatorTarget     |
| DummyMRT / ROS Interface    |
+-------------+---------------+
              |
              v
+-----------------------------+
| RViz / TF / Visualization   |
+-----------------------------+
```

---

# 5. 数据流与控制流

## 5.1 控制主流程

1. launch 启动系统  
2. 读取 `task.info` 与 URDF  
3. `MobileManipulatorInterface` 构建 OCP  
4. ROS target 节点发布末端目标  
5. `ReferenceManager` 持有 `TargetTrajectories`  
6. MPC/DDP/SLQ 根据目标求解控制输入  
7. Rollout / MRT 更新系统状态  
8. Visualization 发布观测、命令和优化轨迹

## 5.2 双臂模式数据流

### 单臂模式

- 目标维度：7
- 右手启用
- 左臂可固定

### 双臂模式

- 目标维度：14
- 右手读取 `0~6`
- 左手读取 `7~13`

## 5.3 控制流图（占位）

```text
[Interactive Markers]
       |
       v
[Target Node]
       |
       v
[TargetTrajectories]
       |
       v
[ReferenceManager]
       |
       v
[EndEffectorConstraint / WaistConstraint]
       |
       v
[OCS2 Solver]
       |
       v
[Optimal Input]
       |
       v
[Rollout / MRT]
       |
       v
[State Update + Visualization]
```

---

# 6. 目录与模块说明

## 6.1 主要目录

| 目录 | 说明 |
|---|---|
| `ocs2_mobile_manipulator/` | OCP 核心、动力学、约束、配置 |
| `ocs2_mobile_manipulator_ros/` | ROS 目标输入、DummyMRT、Visualization |
| `config/WH00003/` | WH00003 专用 task 配置 |
| `WH00003/urdf/` | WH00003 机器人模型 |
| `launch/` | 启动文件 |
| `auto_generated/WH00003/` | 自动生成运动学/动力学缓存 |
| `docker/` | 容器环境 |
| 文档文件 | 项目说明、问题记录、开发文档 |

---

# 7. 核心改动清单（按文件）

本节列出真正影响行为的主要改动文件。

## 7.1 OCP 与约束核心

| 文件 | 改动性质 | 作用 |
|---|---|---|
| `constraint/EndEffectorConstraint.h` | 修改 | 增加 `targetStateOffset`，支持多 ee target |
| `constraint/EndEffectorConstraint.cpp` | 修改 | 按 offset 读取目标 pose |
| `constraint/LinkVerticalConstraint.h` | 新增 | 定义腰部竖直约束接口 |
| `constraint/LinkVerticalConstraint.cpp` | 新增 | 实现腰部 roll/pitch 约束 |
| `MobileManipulatorInterface.h` | 修改 | 增加双 ee 开关与左右 ee frame 成员 |
| `MobileManipulatorInterface.cpp` | 修改 | 装配双 ee 约束、腰部约束、左臂 fixed/释放逻辑 |
| `CMakeLists.txt` | 修改 | 编译新增约束文件 |

## 7.2 ROS 输入输出与测试链路

| 文件 | 改动性质 | 作用 |
|---|---|---|
| `MobileManipulatorTarget.cpp` | 修改较大 | 支持双 marker，发布 7/14 维目标 |
| `MobileManipulatorDummyMRT.cpp` | 修改 | 初始 target 改为支持双臂 |
| `MobileManipulatorDummyVisualization.h` | 修改 | 增加双臂显示成员 |
| `MobileManipulatorDummyVisualization.cpp` | 修改 | 支持双手 command TF 和双手轨迹显示 |

## 7.3 配置与模型

| 文件 | 改动性质 | 作用 |
|---|---|---|
| `config/WH00003/task.info` | 新增/修改 | 双臂参数、腰部约束参数、速度限制、cost 等 |
| `WH00003/urdf/WH00003.urdf` | 修改 | 增加腰部 upright reference frame |
| `launch/manipulator_mabi_mobile_WH00003.launch` | 修改 | WH00003 默认启动入口（底盘可动 + 腰部可动 + 双臂目标支持），并支持 external observation 注入 |
| `launch/manipulator_mabi_mobile_WH00003_no_wheel.launch` | 修改 | WH00003 固定底盘版本（task_no_wheel：底盘 v/ω 速度限制为 0），并支持 external observation 注入 |
| `launch/manipulator_mabi_mobile_WH00003_only_shuangbi.launch` | 修改 | WH00003 仅双臂版本（task_only_shuangbi：底盘 v/ω=0 且 waist 关节速度=0），并支持 external observation 注入 |
| `launch/include/mobile_manipulator.launch` | 修改 | 通用 include：统一加载 task/urdf/libFolder、启动 MPC/DummyMRT/可视化，并把 external observation 参数注入到 DummyMRT 节点私有参数 |
| `auto_generated/WH00003/*` | 新增 | 自动生成缓存库 |

---

# 8. 双臂扩展设计说明

## 8.1 设计目标

在不推翻原版 OCS2 mobile manipulator 框架的前提下，实现：

- 右手末端位姿控制
- 左手末端位姿控制
- 由 task 文件开关控制单/双 ee 模式

## 8.2 实现策略

采用“**同一约束类，多次实例化，不同 offset**”的方法：

- 右手：
  - `eeFrame = right_wrist_pitch_link`
  - `targetStateOffset = 0`

- 左手：
  - `eeFrame = left_wrist_pitch_link`
  - `targetStateOffset = 7`

这样无需为左右手写两套不同的约束类。

## 8.3 单/双臂模式切换逻辑

通过 task 文件字段控制：

```ini
model_information
{
  enableDualEeTarget   true
  fixLeftArmWhenSingle true
  rightEeFrame         "right_wrist_pitch_link"
  leftEeFrame          "left_wrist_pitch_link"
}
```

### 模式逻辑

- `enableDualEeTarget = false`
  - 只注册右手 ee 约束
  - 左臂可以自动 fixed

- `enableDualEeTarget = true`
  - 注册右手 + 左手 ee 约束
  - 左臂进入优化变量

## 8.4 双臂 OCP 装配图（占位）

```text
                 +----------------------+
                 |  TargetTrajectories  |
                 |  dim = 14            |
                 +----------+-----------+
                            |
               +------------+-------------+
               |                          |
               v                          v
+----------------------------+   +----------------------------+
| rightEndEffectorConstraint |   | leftEndEffectorConstraint  |
| eeFrame = right_*          |   | eeFrame = left_*           |
| offset = 0                 |   | offset = 7                 |
+-------------+--------------+   +-------------+--------------+
              \                            /
               \                          /
                \                        /
                 +----------------------+
                 |   OCP / soft cost    |
                 +----------------------+
```

---

# 9. 腰部直立约束设计说明

## 9.1 设计动机

对于双臂移动操作器，仅依靠末端位姿 tracking 可能导致腰部产生较大 roll/pitch 倾斜。  
为提升作业稳定性，需要对 `waist_yaw_link` 增加姿态限制。

## 9.2 为什么不直接约束 Euler roll/pitch

直接对 Euler 角约束存在问题：

- 参数化有奇异性
- 数值线性化不够稳定
- 在 OCS2 中不如几何约束自然

因此采用几何方式构造“竖直约束”。

## 9.3 几何方法

在 URDF 中为 `waist_yaw_link` 增加一个沿“局部竖直方向”偏移的小 reference link。  
然后构造 root 和 tip 的横向位置偏差约束：

- 若 roll/pitch 为 0，则 tip 相对 root 在世界系 XY 偏差应接近 0
- yaw 不会破坏这一性质

## 9.4 腰部约束接口

在 `task.info` 中可通过如下参数开启：

```ini
waistRollPitch
{
  activate true
  muRoll   20.0
  muPitch  20.0
}
```

## 9.5 腰部约束关系图（占位）

```text
          world z
            ^
            |
            |
      [waist_yaw_upright_link]
            *
            |
            |
            *
      [waist_yaw_link]

目标：upright_link 相对 waist_yaw_link 的世界系 XY 偏移尽量为 0
效果：抑制 roll / pitch，保留 yaw
```

---

# 10. 目标轨迹接口说明

## 10.1 单 ee 模式

目标向量维度为 7：

\[
x_{target} = [p_x, p_y, p_z, q_x, q_y, q_z, q_w]
\]

## 10.2 双 ee 模式

目标向量维度为 14：

\[
x_{target} =
[p_R(3), q_R(4), p_L(3), q_L(4)]
\]

## 10.3 四元数顺序

本工程统一使用 Eigen `coeffs()` 顺序：

\[
[q_x, q_y, q_z, q_w]
\]

这是开发中必须保持一致的约定。

## 10.4 target 插值

`EndEffectorConstraint.cpp` 中对 target trajectory 保留了时间插值逻辑：

- 位置：线性插值
- 姿态：四元数球面插值（slerp）

---

# 11. Task 配置文件说明

## 11.1 核心字段

WH00003 中与双臂模式直接相关的关键字段包括：

```ini
model_information
{
  enableDualEeTarget   true
  fixLeftArmWhenSingle true
  baseFrame            "base"
  rightEeFrame         "right_wrist_pitch_link"
  leftEeFrame          "left_wrist_pitch_link"
}
```

## 11.2 arm 维度变化

### 原单臂模式

- waist 4
- right arm 7

共 11 维

### 双臂模式

- waist 4
- left arm 7
- right arm 7

共 18 维

## 11.3 必须同步扩展到 18 维的项

至少包括：

- `initialState.arm`
- `inputCost.R.arm`
- `jointVelocityLimits.lowerBound.arm`
- `jointVelocityLimits.upperBound.arm`

## 11.4 ee 约束参数

原版单 ee 参数通常为：

```ini
endEffector { ... }
finalEndEffector { ... }
```

双 ee 后应改为：

```ini
rightEndEffector
{
  muPosition      10.0
  muOrientation    5.0
}

finalRightEndEffector
{
  muPosition      10.0
  muOrientation    5.0
}

leftEndEffector
{
  muPosition      10.0
  muOrientation    5.0
}

finalLeftEndEffector
{
  muPosition      10.0
  muOrientation    5.0
}
```

## 11.5 腰部约束参数

```ini
waistRollPitch
{
  activate true
  muRoll   20.0
  muPitch  20.0
}
```

---

# 12. ROS 节点说明

## 12.1 `MobileManipulatorTarget`

### 功能

- 读取 task 文件中的模式开关
- 创建单/双 marker
- 将 marker 姿态打包为 target trajectory
- 发布到 OCS2 接口

### 单臂模式

- 创建 `right_ee_target`
- 发布 7 维 target

### 双臂模式

- 创建 `right_ee_target`
- 创建 `left_ee_target`
- 发布 14 维 target

## 12.2 `MobileManipulatorDummyMRT`

### 功能

- 启动 dummy MRT 测试链路
- 在系统初始化时给出默认 target
- 用于脱离真实传感器和真实上位机的测试

### 改动点

- 初始 target 从固定 7 维改为按模式发布 7/14 维

### External Observation（可选注入）

为对接外部仿真/真实机器人，本工程为 DummyMRT 增加了“外部观测注入”模式：

- 当启用后，Dummy loop 不再调用内部的 `forwardSimulation()`（基于 policy rollout/interpolation 生成观测），而是优先使用外部话题提供的 `ocs2_msgs/mpc_observation` 作为当前观测。
- 如果外部观测话题暂时没有数据，则仍回退到内部 `forwardSimulation()`。

DummyMRT 节点的关键私有参数（`~` 命名空间）：

- `~useExternalObservation`（bool，默认 false）：是否启用外部观测注入。
- `~externalObservationTopic`（string，默认 `mobile_manipulator_external_observation`）：订阅的外部观测话题名。

启用示例（以 WH00003 默认 launch 为例）：

```bash
roslaunch ocs2_mobile_manipulator_ros manipulator_mabi_mobile_WH00003.launch \
  useExternalObservation:=true \
  externalObservationTopic:=/mobile_manipulator_external_observation
```

验证订阅是否创建：

```bash
rosnode info /mobile_manipulator_dummy_mrt_node | grep -A3 Subscriptions
```

相关实现文件：

- `ocs2_ros_interfaces/src/mrt/MRT_ROS_Dummy_Loop.cpp`：在每步循环中优先调用 observationProvider。
- `ocs2_mobile_manipulator_ros/src/MobileManipulatorDummyMRT.cpp`：读取参数并订阅 `ocs2_msgs/mpc_observation`，将其注入 Dummy loop。

## 12.3 `MobileManipulatorDummyVisualization`

### 功能

- 发布观测状态
- 发布 command TF
- 发布优化轨迹

### 双臂化改动

- `command_right`
- `command_left`
- 右手优化轨迹
- 左手优化轨迹

---

# 12.4 WH00003 Launch 入口说明（运行配置）

WH00003 当前常用的 3 个启动入口仅在“加载哪个 task.info”上有本质差异，其余（MPC 节点、DummyMRT、可视化、external observation 参数）走同一套 include。

1) `manipulator_mabi_mobile_WH00003.launch`

- 作用：WH00003 默认入口。
- 加载：`config/WH00003/task.info`
- 特征：底盘可动（v/ω 非零限制）、腰部可动（waist 关节速度非零限制）、支持双臂目标。

2) `manipulator_mabi_mobile_WH00003_no_wheel.launch`

- 作用：固定底盘入口（用于仅测试机械臂/上半身，不允许底盘运动）。
- 加载：`config/WH00003/task_no_wheel.info`
- 特征：底盘速度约束 v/ω 下上界均为 0（即 base 输入被锁死），腰部与双臂仍可动。

3) `manipulator_mabi_mobile_WH00003_only_shuangbi.launch`

- 作用：仅双臂入口（常用于“只看双臂跟踪”，固定底盘+腰部）。
- 加载：`config/WH00003/task_only_shuangbi.info`
- 特征：底盘 v/ω=0 且 waist 关节速度上下界均为 0（腰部锁死），双臂仍可动。

三者共同支持的 launch 参数：

- `rviz`：是否启动 RViz。
- `debug`：是否用 gdb 启动 mpc 节点。
- `useExternalObservation`：是否启用外部观测注入。
- `externalObservationTopic`：外部观测话题名。

说明：external observation 参数最终会被注入到 `/mobile_manipulator_dummy_mrt_node` 的私有参数（`~useExternalObservation` / `~externalObservationTopic`）。

---

# 12.5 如何与外部 DDS 对齐 `/mobile_manipulator_external_observation`

## 12.5.1 需要对齐的 ROS 消息格式

外部需要发布/桥接到 ROS 的消息类型就是：`ocs2_msgs/mpc_observation`。

其字段定义为：

```text
float64        time
ocs2_msgs/mpc_state  state   (float32[] value)
ocs2_msgs/mpc_input  input   (float32[] value)
int8           mode
```

重要结论：它不是“时间列表”，而是“单条观测采样”。外部系统需要以一定频率持续发布该消息流。

## 12.5.2 DDS 侧最小数据结构建议

若你在 DDS 里要定义一个等价数据结构，建议保持与 ROS 字段一一对应：

- `time`：double，单位秒（建议使用单调递增时间；可用系统 monotonic clock 或传感器时间戳换算到秒）。
- `state.value`：float32 数组，长度必须等于 `stateDim`。
- `input.value`：float32 数组，长度必须等于 `inputDim`。
- `mode`：int8（如果你没有 mode 概念，先固定为 0 即可）。

对于 WH00003（WheelBasedMobileManipulator + 双臂）当前维度为：

- `stateDim = 21`
- `inputDim = 20`

更具体的每个维度语义与关节顺序见：`docker/wh00003_state_layout.md`。

## 12.5.3 state/input 的索引布局（必须严格一致）

外部 DDS 发布时必须遵守 OCS2 内部索引约定，否则 MPC 看到的状态/输入语义会错位。

WH00003 的布局概要：

- `state[0..2] = [base_x, base_y, base_yaw]`
- `state[3..]`：按 dofNames 顺序的关节位置（waist + left arm + right arm）
- `input[0..1] = [base_v, base_omega]`
- `input[2..]`：按 dofNames 顺序的关节速度 `dq_*`

详表请直接对照 `docker/wh00003_state_layout.md`（它来自工具打印，避免手抄出错）。

## 12.5.4 频率与时间戳注意事项

- Dummy loop 期望 `time` 单调递增。如果外部发布频率低于 Dummy loop 频率，Dummy loop 会在内部检测到 `time` 未增长时“强行 time += dt”，这会导致状态在若干步内保持不变但时间推进。
- 建议外部发布频率尽量接近 `mrtDesiredFrequency`（WH00003 task 默认 400Hz），或至少保证每条消息 time 递增且不长期停滞。

## 12.5.5 最简单的链路测试（ROS 内自洽）

不改代码验证注入链路时，可用 rosbag 做“输出对齐注入”：

1) 关闭 external，录制原始 dummy 输出：

```bash
roslaunch ocs2_mobile_manipulator_ros manipulator_mabi_mobile_WH00003.launch useExternalObservation:=false
rosbag record -O mpc_obs.bag /mobile_manipulator_mpc_observation
```

2) 开启 external，把 bag 回放 remap 到 external topic：

```bash
roslaunch ocs2_mobile_manipulator_ros manipulator_mabi_mobile_WH00003.launch useExternalObservation:=true \
  externalObservationTopic:=/mobile_manipulator_external_observation
rosbag play mpc_obs.bag /mobile_manipulator_mpc_observation:=/mobile_manipulator_external_observation
```

通过该测试可以确认：外部注入链路、消息类型、维度与索引布局完全兼容。

---

# 13. 可视化与调试链路

## 13.1 可视化内容

建议在 RViz/TF 中至少观察以下内容：

- `command_right`
- `command_left`
- 机器人当前状态
- 右手优化轨迹
- 左手优化轨迹

## 13.2 调试链路建议

排查双臂问题时建议分层检查：

1. target 节点是否在发正确维度的数据  
2. `ReferenceManager` 是否收到正确 target  
3. 右/左手 ee 约束是否都注册  
4. 左臂是否真实进入优化变量  
5. Visualization 是否正确解释 target

## 13.3 调试流程图（占位）

```text
[Marker 不动 / 目标不对]
        |
        v
[检查 MobileManipulatorTarget 是否发布 7/14 维正确数据]
        |
        v
[检查 ReferenceManager 中 target 是否正确]
        |
        v
[检查 EndEffectorConstraint offset 是否正确]
        |
        v
[检查 task.info 中左臂是否仍被 remove]
        |
        v
[检查 Visualization 是否只显示了右手]
```

---

# 14. 主要风险点与已知问题

## 14.1 目标维度不一致

最常见问题之一：

- OCP 已切换到双 ee 模式
- 但 target 节点仍在发布 7 维

后果：

- 读取左手目标时越界
- 左手目标无效
- 系统直接报错

## 14.2 左臂未真正释放

双 ee 模式必须确保左臂不再出现在 `removeJoints` 中。  
否则会出现：

- 左手目标存在
- 左手约束存在
- 但左臂不参与优化

表现为左手永远跟不上目标。

## 14.3 arm 配置未全量扩维

当 arm 从 11 维扩到 18 维时，若以下任一项未扩：

- `initialState.arm`
- `inputCost.R.arm`
- `jointVelocityLimits`

就会出现：

- 左臂输入权重缺失
- 左臂速度约束缺失
- 初始状态不完整

## 14.4 Visualization 误导

系统已正确双臂化，但 visualization 仍只显示右手，这会导致误判控制未生效。

## 14.5 自碰撞配置不完整

双臂模式下必须补充以下碰撞对：

- 左臂与 base
- 左臂与右臂
- 左手腕与右手腕/肘部
- 左肘与右臂链路

否则双臂轨迹可能相互穿透。

## 14.6 auto_generated 缓存失效

修改以下内容后建议清理并重建自动生成缓存：

- URDF
- frame 名
- ee frame
- 自由度数量
- CppAD 库名

---

# 15. 开发与修改建议

## 15.1 修改优先级建议

建议按以下顺序推进修改：

1. 先确保 OCP 核心双臂化
2. 再确保 target 输入双臂化
3. 再确保 DummyMRT 与 Visualization 双臂化
4. 最后补全 self-collision 与工程细节

## 15.2 调试建议

每次只改一层，避免多处同时改动造成定位困难：

- 先验证 14 维 target 是否打通
- 再验证左臂是否进入优化
- 再验证左右手是否都在跟踪
- 再叠加腰部约束
- 最后调 self-collision 和权重

## 15.3 代码维护建议

建议将双 ee 模式下的以下逻辑模块化：

- target packing/unpacking
- ee frame 读取
- 单/双 ee 配置检查
- visualization target parsing

避免这些逻辑在多个文件中重复出现。

---

# 16. 后续扩展建议

后续若继续开发，建议优先完善以下方向：

## 16.1 完整双臂 self-collision

目前这是双臂模式中最容易缺项的部分，应完善：

- 左臂对 base
- 左臂对右臂
- 双腕、双肘之间的碰撞对

## 16.2 统一 target 接口检查

建议增加运行时检查：

- 单 ee 时 target 维度必须为 7
- 双 ee 时 target 维度必须为 14

减少隐藏错误。

## 16.3 增强 visualization

建议在 RViz 中区分显示：

- 右手 target / trajectory
- 左手 target / trajectory
- 腰部姿态参考

## 16.4 task.info 注释化与模块化

建议把 `task.info` 中的 WH00003 双臂配置拆分成更清晰的模块，并补充中文注释，便于长期维护。

---

# 附录 A：双臂模式关节顺序

双臂模式下 `arm` 的推荐顺序如下：

1. `waist_pitch1_joint`
2. `waist_pitch2_joint`
3. `waist_pitch3_joint`
4. `waist_yaw_joint`
5. `left_shoulder_pitch_joint`
6. `left_shoulder_roll_joint`
7. `left_shoulder_yaw_joint`
8. `left_elbow_pitch_joint`
9. `left_wrist_roll_joint`
10. `left_wrist_yaw_joint`
11. `left_wrist_pitch_joint`
12. `right_shoulder_pitch_joint`
13. `right_shoulder_roll_joint`
14. `right_shoulder_yaw_joint`
15. `right_elbow_pitch_joint`
16. `right_wrist_roll_joint`
17. `right_wrist_yaw_joint`
18. `right_wrist_pitch_joint`

---

# 附录 B：双 ee target 维度定义

## B.1 目标向量定义

\[
x_{target} =
[p_R(3), q_R(4), p_L(3), q_L(4)]
\]

## B.2 下标说明

- `0~2`：右手位置
- `3~6`：右手四元数
- `7~9`：左手位置
- `10~13`：左手四元数

## B.3 四元数说明

统一使用：

\[
[q_x, q_y, q_z, q_w]
\]

---

# 附录 C：流程图与接口关系图占位

以下图建议后续补入正式开发文档或论文附录：

## C.1 建议补充的流程图

1. **系统启动流程图**
   - launch
   - task/URDF 加载
   - OCP 初始化
   - ROS 节点启动

2. **双 ee target 数据流图**
   - marker
   - target node
   - target trajectory
   - ee constraint

3. **腰部约束几何关系图**
   - `waist_yaw_link`
   - `waist_yaw_upright_link`
   - 世界坐标系
   - 横向偏移约束

4. **DummyMRT / Visualization 调试链路图**
   - 初始 target
   - policy
   - rollout
   - TF/path 发布

## C.2 建议补充的接口关系图

```text
MobileManipulatorTarget
    -> TargetTrajectories
        -> ReferenceManager
            -> EndEffectorConstraint (right / left)
            -> LinkVerticalConstraint (waist)
                -> MobileManipulatorInterface
                    -> OCS2 Solver
                        -> DummyMRT / Real MRT
                            -> Visualization
```

---
