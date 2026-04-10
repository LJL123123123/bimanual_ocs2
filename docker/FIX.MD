# OCS2 Mobile Manipulator：DOF/底盘不动问题排查与修复记录

> 适用仓库：`/root/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator*`
>
> 目标：
> 1) 排除 URDF/配置导致的自由度(DOF)不一致；
> 2) 解释并定位“手臂能动、底盘不动”的原因；
> 3) 通过最小改动让底盘明显动起来（偏向底盘动、手臂少动）。

---

## 1. 现象与结论概览

### 1.1 现象
- 通过 `manipulator_mabi_mobile*.launch` 启动后：
  - 手臂可以跟随 RViz 交互目标运动；
  - 但底盘（轮式基座）不动或几乎不动。

### 1.2 关键结论
- **DOF 对齐本身不是导致“底盘不动”的主因**（至少在 mabi_mobile 配置下，手臂 DOF 与任务配置匹配）。
- 在当前例子里，RViz 交互目标发布的是 **末端执行器(EE)的 6D 位姿目标（pos+quat）**，而不是底盘位姿目标。
- MPC 里只有 EE 软约束（惩罚 EE 误差）+ 输入惩罚（`inputCost.R`）+ 速度限幅等。
- 因此：**当“动手臂”比“动底盘”更便宜时，优化器会自然选择只动手臂**，底盘保持静止反而是“最优”。

---

## 2. DOF 一致性：需要检查的所有位置

启动链路涉及的 DOF 信息来源主要有三类：

1) **URDF（robot_description）**
- 由 URDF 中的可动关节数决定（轮式模型还会在 Pinocchio 模型里注入 base DOF，见下）。

2) **task.info 的 model_information**
- `manipulatorModelType`：决定底盘/基座 DOF 模型
  - `1 = Wheel-based manipulator`：底盘状态为 `(x, y, yaw)`，输入为 `(v, omega)`。
- `removeJoints`：可将某些关节视为固定。
- `baseFrame` / `eeFrame`：Pinocchio 运动学计算会用到。

3) **代码根据模型类型推导的维度**
- stateDim/inputDim/armDim 由 Pinocchio 模型与 mapping 决定。

实践中：DOF 不一致通常会在启动时/运行时表现为维度断言失败、矩阵维度不匹配、或控制量映射错位。

---

## 3. “底盘不动”的定位方法（非常重要）

底盘不动有两条完全不同的分支，需要先用日志把它分开：

### 分支 A：**优化器输出的底盘命令本来就是 0**（策略选择不动底盘）
表现：
- policy 里 `u0 = [v, omega, dq_arm...]` 的前两维接近 0。

原因通常是：
- `inputCost.R.base` 相对 `inputCost.R.arm` 太大（底盘更“贵”）；
- 速度限幅过小，导致底盘贡献有限、仍然不如手臂划算；
- EE 目标在手臂工作空间内，纯手臂即可完成。

解决：
- 调整 `inputCost.R`，让底盘更便宜、手臂更贵（见第 4 节）。

### 分支 B：**优化器输出了非零底盘命令，但底盘状态不更新**（系统传播/观测链路问题）
表现：
- policy 里 `v/omega` 明显非零，但观测 state 的 `(x,y,yaw)` 不变。

原因可能是：
- 观测源没有把底盘状态更新给 OCS2；
- dummy loop/可视化用的 observation 没有集成 dynamics；
- 其它节点覆盖/重置了 base state。

解决：
- 继续沿“状态来源/集成器/话题”方向查。

### 为了区分 A/B，本次添加了关键日志
在可视化更新周期中打印：
- `baseCmd(v, omega)`：取自 MPC policy 的 `inputTrajectory_.front()` 前两维
- `baseState(x, y, yaw)`：取自当前 observation.state 的前 3 维

这样只要看一眼就能知道究竟是 A 还是 B。

---

## 4. 让“底盘明显动起来”的 task.info 改法（已落地到 AMRS1）

### 4.1 原理
OCS2 这里的结构基本是：
- 末端误差越小越好（EE penalty/constraint）
- 控制输入越小越好（`inputCost.R`）

如果 `R_base` 比 `R_arm` 更大，优化器会倾向少用底盘（因为底盘输入更贵）。

因此要“底盘明显动、手臂少动”：
- **降低底盘输入权重**（让 v/omega 便宜）
- **提高手臂输入权重**（让关节速度更贵）

### 4.2 已应用的修改（文件）
- 已修改文件：
  - `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/AMRS1/task.info`

### 4.3 修改内容（具体数值）
改动位置：`inputCost { R { base.wheelBasedMobileManipulator {...} arm {...} } }`

- 底盘（wheelBasedMobileManipulator）：
  - `scaling 1e-2` → `scaling 1e-3`
  - `(0,0) 5.0`、`(1,1) 5.0` → 都改为 `1.0`
  - 等效权重大约从 `0.05` 降到 `0.001`（便宜约 50 倍）

- 手臂（arm）：
  - `scaling 1e-2` → `scaling 5e-2`
  - 对角从 `1.0` → `5.0`
  - 等效权重大约从 `0.01` 升到 `0.25`（更贵约 25 倍）

综合：手臂相对底盘大约贵 `250x`，会明显偏向用底盘来完成 EE 跟踪。

> 回滚方式：把上述段落改回原来的 scaling/对角值即可。

### 4.4 注意事项
- 底盘速度限幅仍然有效：
  - `jointVelocityLimits.base.wheelBasedMobileManipulator` 目前是 `v ∈ [-0.1, 0.1]`、`omega ∈ [-0.3, 0.3]`。
  - 如果你希望“更快更明显”，可以适度放大该限幅（这会改变系统能力边界，需你确认机器人实际可达）。

---

## 5. 相关代码路径（用于进一步定位）

下面这些是本次排查中确认过的“底盘确实应该能动”的关键实现点（用于你后续深入排查）：

- 目标生成（RViz 交互 marker → TargetTrajectories）
  - `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorTarget.cpp`
  - 目标 stateTrajectory 是 7D：`position(3) + quaternion(4)`（仅 EE 位姿目标）。

- EE 约束解释目标（7D reference → 6D 误差）
  - `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/constraint/EndEffectorConstraint.cpp`

- 轮式底盘动力学（state 有 x,y,yaw；input 有 v,omega）
  - `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/dynamics/WheelBasedMobileManipulatorDynamics.cpp`

- 输入/速度映射（OCS2 input → Pinocchio v / Jacobian）
  - `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/MobileManipulatorPinocchioMapping.cpp`

- 诊断日志（区分 baseCmd 是否为 0）
  - `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorDummyVisualization.cpp`

---

## 6. 验证步骤（推荐）

1) 重新编译并重启（确保读取到新的 task.info）
- 编译：`catkin build ocs2_mobile_manipulator_ros`
- 重启 launch（不要只重启 RViz，task.info 通常随 node 启动读取）。

2) 在 RViz 拖动目标，观察控制台日志
- 重点看：
  - `baseCmd(v, omega)=...`
  - `baseState(x, y, yaw)=...`

3) 判定分支
- 若 `baseCmd` 仍接近 0：继续降低 `R_base` 或提高 `R_arm`；或检查 EE 惩罚是否太小。
- 若 `baseCmd` 非零但 `baseState` 不变：转查状态观测/集成链路。

---

## 7. 常见坑与建议

- **目标不是底盘目标**：当前交互 marker 只给 EE 位姿；如果你想“底盘去某个点”，需要额外的 base tracking cost/constraint（本次没有添加此功能，保持最小改动）。
- **成本权重决定“谁动”**：没有显式 base 目标时，底盘是否动主要取决于“用底盘降低 EE 误差是否划算”。
- **速度限幅太小会让底盘看起来不动**：即使在动，`v=0.05` 这种也可能视觉上不明显。

### 7.1 URDF 含左右臂时：如何更新 task（以 WH00003 为例）

如果你更换的 URDF 同时包含左臂和右臂，但你只想控制其中一条链（本例 `eeFrame = right_wrist_pitch_link`），建议：

- 在 task.info 里用 `model_information.removeJoints` 把不使用的那条臂的关节设为固定（例如固定全部 `left_*_joint`）。
- 将 task.info 中所有“arm 维度相关”的配置同步到**实际控制的关节数**，至少包括：
  - `initialState.arm` 的行数
  - `inputCost.R.arm` 的对角维度
  - `jointVelocityLimits.arm` 的上下界条目数

本仓库中已将 `config/WH00003/task.info` 更新为：控制“腰部 4 轴 + 右臂 7 轴”，并固定左臂 7 个关节。

---

## 8. 变更摘要

- ✅ 已修改：`src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/AMRS1/task.info`
  - 调整 input cost：底盘更便宜、手臂更贵。

- ✅ 已加入诊断日志（用于区分 baseCmd 是否为 0）：
  - `src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorDummyVisualization.cpp`

如果你希望我把同样的“底盘更积极”权重同步到 `mabi_mobile` 对应的 task.info（而不是 AMRS1），告诉我你实际 launch 用的是哪个 task 文件路径（或把 launch 里的 `taskFile:=...` 贴出来）。
