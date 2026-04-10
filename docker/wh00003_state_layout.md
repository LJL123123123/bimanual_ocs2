# WH00003：`SystemObservation.state[]/input[]` 索引布局（用于仿真/驱动对接）

这份文档回答一个很“落地”的问题：当你要把 dummy loop 换成真仿真/真机器人时，**`/mobile_manipulator_mpc_observation` 里的 `observation.state` 每一维到底对应哪个量/哪个关节**，以及 MPC policy 里的 `u`（`observation.input`）每一维的含义。

> 结论先说：当前 WH00003 的 task 配置里 `manipulatorModelType = 1`（Wheel-based），因此
>
> $$x = [x\; y\; \psi\; q_0\; q_1\; \dots]^T,\quad u = [v\; \omega\; \dot q_0\; \dot q_1\; \dots]^T$$
>
> 且 arm 关节角 **永远放在 state 的最后 `armDim` 维**（由 `ManipulatorModelInfo.armDim` 决定）。

---

## 1. WH00003 当前使用的模型类型

- task 文件：
  - [src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/WH00003/task_no_wheel.info](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/WH00003/task_no_wheel.info)
- 关键字段：`model_information.manipulatorModelType 1`
  - `1 = WheelBasedMobileManipulator`

Wheel-based 模型在 OCS2 里的动力学（核心形式）是：

$$\dot x = \cos(\psi) v,\quad \dot y = \sin(\psi) v,\quad \dot\psi = \omega,\quad \dot q = \dot q$$

对应实现：
- [src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/dynamics/WheelBasedMobileManipulatorDynamics.cpp](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/dynamics/WheelBasedMobileManipulatorDynamics.cpp)

---

## 2. `state[]` 布局（Wheel-based）

Wheel-based 时：
- `baseStateDim = 3`
- `stateDim = 3 + armDim`

因此 `observation.state` 的前 3 维是底盘位姿，最后 `armDim` 维是“被 Pinocchio 认为是 arm 的关节角”。

### 2.1 Base 三维
- `state[0]`：`base_x`（world）
- `state[1]`：`base_y`（world）
- `state[2]`：`base_yaw`（world，绕 Z）

### 2.2 Arm 关节角（按 dofNames 顺序）
下面这份顺序来自 task 文件的 `initialState.arm` 注释（也是你目前配置里隐含假定的 arm 顺序）：

- `state[3]`：`waist_pitch1_joint`
- `state[4]`：`waist_pitch2_joint`
- `state[5]`：`waist_pitch3_joint`
- `state[6]`：`waist_yaw_joint`

- `state[7]`：`left_shoulder_pitch_joint`
- `state[8]`：`left_shoulder_roll_joint`
- `state[9]`：`left_shoulder_yaw_joint`
- `state[10]`：`left_elbow_pitch_joint`
- `state[11]`：`left_wrist_roll_joint`
- `state[12]`：`left_wrist_yaw_joint`
- `state[13]`：`left_wrist_pitch_joint`

- `state[14]`：`right_shoulder_pitch_joint`
- `state[15]`：`right_shoulder_roll_joint`
- `state[16]`：`right_shoulder_yaw_joint`
- `state[17]`：`right_elbow_pitch_joint`
- `state[18]`：`right_wrist_roll_joint`
- `state[19]`：`right_wrist_yaw_joint`
- `state[20]`：`right_wrist_pitch_joint`

> 重要：最终以 Pinocchio 解析出来的 `ManipulatorModelInfo.dofNames` 为准。如果你替换了 URDF / removeJoints，这个顺序可能会变化。

---

## 3. `input[]` 布局（Wheel-based）

Wheel-based 时 `inputDim = 2 + armDim`，语义是**底盘速度 + 关节速度**：

- `input[0]`：`base_v`（底盘前向速度，base frame）
- `input[1]`：`base_omega`（底盘转向角速度，yaw rate）
- `input[2 + i]`：第 `i` 个 `dofNames[i]` 的关节速度 `dq`

按上面的关节顺序展开就是：
- `input[2]`：`d(waist_pitch1_joint)`
- …
- `input[19]`：`d(right_wrist_pitch_joint)`

---

## 4. 一键“以代码真实解析结果”为准打印布局

我加了一个小工具可执行文件，会用同一套 task+URDF 解析逻辑输出：
- `stateDim/inputDim/armDim`
- `dofNames` 的真实顺序
- 以及 state/input 的索引解释

源码：
- [src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/tools/print_state_layout.cpp](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/tools/print_state_layout.cpp)

编译：
- `catkin build ocs2_mobile_manipulator`

运行（在 `source devel/setup.bash` 后）：
- `rosrun ocs2_mobile_manipulator ocs2_mobile_manipulator_print_layout --task $(rospack find ocs2_mobile_manipulator)/config/WH00003/task_no_wheel.info --urdf $(rospack find ocs2_robotic_assets)/resources/mobile_manipulator/WH00003/urdf/WH00003.urdf`

---

## 5. 对接仿真/机器人时怎么用这份映射

- 你要“替换 dummy 的 forwardSimulation()”时：
  - 从仿真器读到 `base(x,y,yaw)` + 全部关节角 `q`，按上面索引填进 `observation.state`。
- 你要“把 MPC 输出应用到仿真器/机器人”时：
  - 从 policy 的 `u` 取：
    - `v/omega` → 底盘速度命令（或再换算成轮速）
    - `dq_arm` → 关节速度命令（或积分成位置再发 position command）

这两半缺一不可：只换观测来源会导致系统不受控；只发命令不更新观测会导致 MPC 以为状态没变。

---

## 6. 最小接入方式：让 dummy loop 使用外部观测（可选）

为了把 dummy loop 替换成真仿真/真机器人回传的状态，我加了一个**默认关闭**的开关：

- 节点：[src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorDummyMRT.cpp](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorDummyMRT.cpp)
- 参数（ROS param）：
  - `useExternalObservation`：`false`(默认) / `true`
  - `externalObservationTopic`：默认 `mobile_manipulator_external_observation`
- 消息类型：`ocs2_msgs/mpc_observation`

启用后，dummy loop 将优先使用外部 topic 的观测来更新 `currentObservation`，不再调用内部 `forwardSimulation()`。
