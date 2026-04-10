
# 单目标单手 → 双目标双手（Bimanual）改造手册

本文记录如何把 OCS2 Mobile Manipulator 示例从“单目标（单个 EE 位姿目标）/单手优化”升级为“双目标（左右手各一个 EE 位姿目标）/双手优化”。

适用范围：本工作区的 mobile_manipulator 示例工程。

核心思路：
- 把 MPC 的参考目标向量从 7 维扩展为 14 维：
	- 右手目标：$[p_{r}(3), q_{r}(4)]$，offset = 0
	- 左手目标：$[p_{l}(3), q_{l}(4)]$，offset = 7
- 仍然使用同一条 TargetTrajectories（同一条 target stateTrajectory），但对左右手分别注册两个 EndEffectorConstraint，并通过 offset 从目标向量中取各自子段。

> 说明：这里的“7 维 EE 目标”是 `position(3) + quaternion.coeffs()(4)`。
> `Eigen::Quaternion::coeffs()` 的顺序是 (x, y, z, w)。本仓库里发布 target 和解析 target 都使用 `.coeffs()`，因此顺序是一致的。

---

## 1. 你需要改哪些模块？

从数据流角度看，链路是：

1) RViz 交互拖拽块（InteractiveMarker）生成左右手目标位姿
2) 目标发布节点把位姿拼成 `TargetTrajectories.stateTrajectory`
3) MPC 侧约束/代价读取 `TargetTrajectories` 并计算左右手 EE 误差
4) 优化器通过动力学 + 成本权重决定如何同时满足两个 EE 目标

对应到文件：
- ROS 目标发布节点（interactive marker + 目标拼装 + 发布）
	- [src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorTarget.cpp](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorTarget.cpp)
- MPC 接口：读取 task.info 的左右 EE frame + 注册左右 EE 约束（offset=0/7）
	- [src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp)
- EE 约束实现：从 target 向量中按 offset 解包 position/quaternion
	- [src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/include/ocs2_mobile_manipulator/constraint/EndEffectorConstraint.h](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/include/ocs2_mobile_manipulator/constraint/EndEffectorConstraint.h)
	- [src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/constraint/EndEffectorConstraint.cpp](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/constraint/EndEffectorConstraint.cpp)
- 任务配置：开关 + 左右 EE frame + 左右 EE 约束权重
	- [src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/WH00003/task.info](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/WH00003/task.info)

---

## 2. 第一步：统一“目标向量布局（layout）”

把 target stateTrajectory 的 layout 视为一个协议；ROS 和 MPC 两侧必须完全一致。

### 2.1 单目标（单手）layout

长度 = 7：

1. `target[0:3]`：EE position
2. `target[3:7]`：EE quaternion coeffs（x,y,z,w）

对应实现（ROS 拼装）：
- [MobileManipulatorTarget.cpp](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorTarget.cpp) 的 `buildSingleTarget()`

对应实现（MPC 解包）：
- [EndEffectorConstraint.cpp](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/constraint/EndEffectorConstraint.cpp) 使用 `targetStateOffset_ = 0`

### 2.2 双目标（双手）layout

长度 = 14：

1. `target[0:7]`：右手（offset = 0）
2. `target[7:14]`：左手（offset = 7）

约定：
- 右手 position：`target.segment<3>(0)`
- 右手 quat：`target.segment<4>(3)`
- 左手 position：`target.segment<3>(7)`
- 左手 quat：`target.segment<4>(10)`

对应实现（ROS 拼装）：
- [MobileManipulatorTarget.cpp](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorTarget.cpp) 的 `buildDualTarget()`

对应实现（MPC 解包）：
- 右手约束：`targetStateOffset_ = 0`
- 左手约束：`targetStateOffset_ = 7`

---

## 3. 第二步：ROS 侧把“一个 marker”扩展为“两个 marker”

### 3.1 两个 InteractiveMarker

在 [MobileManipulatorTarget.cpp](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/src/MobileManipulatorTarget.cpp) 中：
- 始终创建右手 marker（例如 `right_ee_target`）
- 当 `model_information.enableDualEeTarget=true` 时，再创建左手 marker（例如 `left_ee_target`）

### 3.2 兼容 RViz 配置（拖拽块“消失”的最常见原因）

RViz 配置 [src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/rviz/mobile_manipulator.rviz](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/rviz/mobile_manipulator.rviz) 默认订阅：
- `Update Topic: /simple_marker/update`

因此 target 节点创建 `InteractiveMarkerServer` 时应保持 server 名称为 `simple_marker`，否则 RViz 订阅不到 update，看起来就像“拖拽块没了”。

### 3.3 发布的 TargetTrajectories 维度必须与开关一致

单手：发布 7 维；双手：发布 14 维。

建议做法：
- 在 target 节点里根据 `enableDualEeTarget_` 选择 `buildSingleTarget()` / `buildDualTarget()`。

---

## 4. 第三步：MPC 侧注册左右手 EE 约束（关键点：offset）

约束注册位置在 [MobileManipulatorInterface.cpp](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp)。

### 4.1 从 task.info 读取左右 EE frame

需要的参数：
- `model_information.enableDualEeTarget`：是否启用双手目标
- `model_information.rightEeFrame`：右手 EE link
- `model_information.leftEeFrame`：左手 EE link（仅在 enableDualEeTarget=true 时使用）

### 4.2 注册两套约束（stateSoftConstraint + finalSoftConstraint）

单手（右手）：
- `rightEndEffector`（offset=0）
- `finalRightEndEffector`（offset=0）

双手（额外增加左手）：
- `leftEndEffector`（offset=7）
- `finalLeftEndEffector`（offset=7）

你可以直接对照当前代码中的注册逻辑：
- [MobileManipulatorInterface.cpp](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/MobileManipulatorInterface.cpp)

### 4.3 EndEffectorConstraint 必须按 offset 解包目标

约束实现要保证：
- 当 `targetStateOffset_ + 7 > target.size()` 时，给出明确错误（避免 silent bug）
- quaternion 读取使用 `.coeffs()` 并做归一化（避免数值问题）

对应实现见：
- [EndEffectorConstraint.h](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/include/ocs2_mobile_manipulator/constraint/EndEffectorConstraint.h)
- [EndEffectorConstraint.cpp](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/src/constraint/EndEffectorConstraint.cpp)

---

## 5. 第四步：task.info 增加双手开关与左右约束权重

以 WH00003 为例，配置在：
- [src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/WH00003/task.info](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/WH00003/task.info)

### 5.1 model_information

最小必需项：
- `enableDualEeTarget true`
- `rightEeFrame "..."`
- `leftEeFrame "..."`

注意：
- 若你要真正做“双手优化”，URDF 中左臂关节不能被加入 `removeJoints`，并且 arm 维度相关的配置（initialState/inputCost/limits）要覆盖左右臂全部自由度。

### 5.2 左右手约束参数块

本仓库的读取方式是以 prefix 为键读取：
- 右手：`rightEndEffector.muPosition`、`rightEndEffector.muOrientation`
- 左手：`leftEndEffector.muPosition`、`leftEndEffector.muOrientation`
- 终端项同理：`finalRightEndEffector.*`、`finalLeftEndEffector.*`

如果你希望左右手权重不同，这是最直接的入口（分别调 `muPosition/muOrientation`）。

---

## 6. 第五步：验证流程（建议按这个顺序排错）

1) 编译
- `catkin build ocs2_mobile_manipulator_ros`

2) 启动
- `roslaunch ocs2_mobile_manipulator_ros manipulator_mabi_mobile_WH00003.launch`

3) RViz 检查
- Fixed Frame 是否为 `world`（默认 rviz 配置就是 world）
- Displays → InteractiveMarkers 的 `Update Topic` 是否为 `/simple_marker/update`

4) 运行时维度一致性
- enableDualEeTarget=false：target state 应为 7 维
- enableDualEeTarget=true：target state 应为 14 维

如果你看到 MPC/约束报维度不匹配，优先检查：
- ROS 侧是否真的在发布 14 维 target
- MPC 侧是否真的注册了 leftEndEffector（enableDualEeTarget 是否被 task.info 正确读取）

---

## 7. 常见坑

1) “拖拽块不见了”
- 99% 是 InteractiveMarker 的 update topic 不匹配：RViz 订阅 `/simple_marker/update`，但 server 名称变了。

2) quaternion 顺序错
- 混用 `wxyz` 和 `xyzw` 会导致姿态追踪异常。
- 本仓库按 `.coeffs()` 走的是 `xyzw`，保持一致即可。

3) DOF/配置没有同步
- 双手优化必须保证：URDF 里的左臂关节没有被 remove，且 task.info 的 arm 相关向量/矩阵维度覆盖左右臂。

4) “双目标”只改 ROS 没改 MPC
- 只发布 14 维目标但 MPC 仍只有一个 EE 约束，会导致左手目标完全不起作用。
- 必须同时添加 leftEndEffector 约束，并设置 offset=7。

