# ROS1 rostopic ↔ 原生 DDS 桥接：所需信息清单（WH00003）

适用范围：你的“对端是原生 DDS 应用（非 ROS2）”。

结论先写在前面：ROS1 和原生 DDS **不能直接通信**，必须有桥接进程（ROS1 subscriber/publisher + DDS subscriber/publisher），并完成 **类型(IDL) + 语义 + QoS** 的映射。

---

## 1. 你必须掌握/提供的最小信息（做桥接必需）

对每一个要桥的 topic，至少要有以下信息，否则只能做到“收得到包”，但很难保证“可用/稳定/可维护”。

### 1.1 类型与字段结构（必须）

- ROS1 侧：`rostopic type <topic>` 得到 `pkg/MsgName`
- ROS1 字段：`rosmsg show pkg/MsgName` 或直接看 `.msg` 文件
- DDS 侧：需要把 ROS `.msg` **等价转换为 IDL**（字段类型、数组/序列、嵌套结构都要一一对应）

你当前工作区自定义消息的位置：
- `ocs2_msgs`：`/root/ocs2_ws/src/ocs2/ocs2_msgs/msg/`

### 1.2 语义（强烈建议必须）

同一个结构，不同语义会导致“通了但错”。需要明确：
- 单位：m / rad / s / N / Nm 等
- 坐标系：frame 定义、右手系/左手系、base/world/map 的含义
- 数组含义：例如 `state.value[]`、`input.value[]` 的维度、关节顺序、每个 index 对应哪个量
- mode 的枚举含义（`int8 mode`）

### 1.3 发布行为与带宽（必须）

- 是否持续：连续流（控制/状态）还是触发式（目标更新/交互）
- 频率范围：平均 Hz、峰值 Hz、一次消息大概字节数
- 对端 DDS 能否承受：CPU、带宽、队列深度、丢包策略

特别注意：你当前环境里 `/tf` 频率非常高（`rostopic hz /tf` 能到 ~1.6kHz 量级），原生 DDS 侧通常需要节流/过滤，否则很容易把网络与 CPU 打满。

### 1.4 QoS 映射策略（DDS 必需）

ROS1 没有显式 QoS，但 DDS 必须配置（至少要决定）：
- Reliable vs BestEffort
- History: KeepLast(depth) / KeepAll
- Durability: Volatile / TransientLocal（类似 ROS latch 的语义）
- ResourceLimits / MaxSamples

桥接时要为每个 topic 定一个 QoS 模板。

### 1.5 话题命名与发现（必须）

- DDS 侧 topic name 采用什么命名（是否保留 ROS 名称 `/xxx`）
- DDS Domain ID、参与者发现方式、是否跨网段、是否需要静态发现
- 序列化：CDR（FastDDS/Connext/Cyclone 各实现兼容性问题）

---

## 2. 针对 WH00003：建议优先桥接哪些 topic

从“能让对端做事情”的角度，通常只需要桥接 OCS2 主链路相关：

- 建议桥：
  - `/mobile_manipulator_mpc_observation`（对端想监控/记录/闭环时需要）
  - `/mobile_manipulator_mpc_target`（对端要下发目标时需要）
  - `/mobile_manipulator_mpc_policy`（对端要消费 MPC 输出/性能指标时需要）

- 通常不桥或谨慎桥：
  - `/tf`、`/tf_static`：数据量大且语义复杂（frame 树），更建议对端只拿“关键 frame”的位姿或由对端自行维护坐标系
  - RViz 工具/交互相关：`/clicked_point`、`/initialpose`、`/move_base_simple/goal`、`/simple_marker/*`（除非对端也要实现 RViz 类似交互）
  - 纯可视化：`/mobile_manipulator/optimized*`、`/distance_markers`

---

## 3. 逐 topic 的桥接信息（单行表格：桥接视角）

说明：本表更多是“桥接需要补充什么关键信息”。如果你只桥核心链路，把非核心行忽略即可。

| topic | ROS type | 结构文件/命令 | DDS 侧要定义/确认 | 桥接注意事项 |
|---|---|---|---|---|
| /mobile_manipulator_mpc_observation | ocs2_msgs/mpc_observation | /root/ocs2_ws/src/ocs2/ocs2_msgs/msg/mpc_observation.msg | 需要 IDL：time + state.value(seq<float>) + input.value(seq<float>) + mode(int8) | 必须确认 state/input 向量维度与 index 含义（关节顺序、是否含 base/waist） |
| /mobile_manipulator_mpc_target | ocs2_msgs/mpc_target_trajectories | /root/ocs2_ws/src/ocs2/ocs2_msgs/msg/mpc_target_trajectories.msg | IDL：timeTrajectory(seq<double>) + stateTrajectory(seq<State>) + inputTrajectory(seq<Input>) | 触发式更新；确认轨迹长度上限与采样时间间隔；对端下发时要与 MPC horizon 对齐 |
| /mobile_manipulator_mpc_policy | ocs2_msgs/mpc_flattened_controller | /root/ocs2_ws/src/ocs2/ocs2_msgs/msg/mpc_flattened_controller.msg | IDL：controllerType + initObservation + planTargetTrajectories + state/input/time traj + performanceIndices 等 | 结构较大；若 DDS 带宽紧张，可拆分成“策略轨迹”和“性能指标”两路 topic |
| /mobile_manipulator_mode_schedule | ocs2_msgs/mode_schedule | /root/ocs2_ws/src/ocs2/ocs2_msgs/msg/mode_schedule.msg | eventTimes(seq<double>) + modeSequence(seq<int8>) | 你当前无 publisher；只有在混合模式/接触切换时才需要桥 |
| /tf | tf2_msgs/TFMessage | rosmsg show tf2_msgs/TFMessage | IDL 要实现 TransformStamped 序列（含 frame_id 字符串） | 高频大流量；强烈建议桥“关键 frame 的 pose”而不是全量 TF |
| /tf_static | tf2_msgs/TFMessage | rosmsg show tf2_msgs/TFMessage | 同上 | 语义类似“静态/持久化”；DDS 侧通常用 TransientLocal 才能让新订阅者拿到 |
| /joint_states | sensor_msgs/JointState | rosmsg show sensor_msgs/JointState | name(seq<string>), position/velocity/effort(seq<double>) | 你当前无 publisher；若对端要驱动机器人，优先桥关节状态/命令而非 TF |
| /simple_marker/update | visualization_msgs/InteractiveMarkerUpdate | rosmsg show visualization_msgs/InteractiveMarkerUpdate | 一般不建议桥 | RViz 专用交互协议，非必要 |
| /simple_marker/feedback | visualization_msgs/InteractiveMarkerFeedback | rosmsg show visualization_msgs/InteractiveMarkerFeedback | 一般不建议桥 | 同上 |
| /mobile_manipulator/optimizedPoseTrajectory | geometry_msgs/PoseArray | rosmsg show geometry_msgs/PoseArray | 一般不建议桥 | 可视化用途；对端若需要，建议桥更紧凑的轨迹结构 |
| /mobile_manipulator/optimizedStateTrajectory | visualization_msgs/MarkerArray | rosmsg show visualization_msgs/MarkerArray | 不建议桥 | MarkerArray 本质是 RViz 渲染数据 |
| /distance_markers | visualization_msgs/MarkerArray | rosmsg show visualization_msgs/MarkerArray | 不建议桥 | 同上 |
| /clicked_point | geometry_msgs/PointStamped | rosmsg show geometry_msgs/PointStamped | 不建议桥 | RViz 工具输入 |
| /initialpose | geometry_msgs/PoseWithCovarianceStamped | rosmsg show geometry_msgs/PoseWithCovarianceStamped | 不建议桥 | RViz 工具输入 |
| /move_base_simple/goal | geometry_msgs/PoseStamped | rosmsg show geometry_msgs/PoseStamped | 视需求 | 若对端要发 goal，建议直接定义“目标位姿”IDL，更清晰 |
| /rosout | rosgraph_msgs/Log | rosmsg show rosgraph_msgs/Log | 不建议桥 | 调试日志，不是业务数据 |
| /rosout_agg | rosgraph_msgs/Log | rosmsg show rosgraph_msgs/Log | 不建议桥 | 同上 |

---

## 4. 建议的桥接实现形态（最小可行）

- 一个桥接进程（C++/Python 均可）：
  - ROS1 side：订阅/发布对应 topic（使用 `roscpp`/`rospy`）
  - DDS side：用你选定的 DDS 实现（FastDDS/Connext/Cyclone 的原生 API）订阅/发布
  - 中间做转换：ROS msg ↔ DDS IDL struct

- 关键设计点：
  - 只桥“业务数据”topic（MPC observation/target/policy），其余尽量不要桥
  - 为每个 topic 明确 QoS（可靠性、深度、持久化）
  - 做节流与过滤（尤其 TF）

---

## 5. 快速获取桥接信息的命令/脚本

你已经有脚本可快速查看 type/pub-sub/结构位置/发送模式：

- 脚本：`/root/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/docker/topic_inspect.py`

生成 Markdown 表格：

```bash
cd /root/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/docker
source /root/ocs2_ws/devel/setup.bash
python3 topic_inspect.py --md > wh00003_topics_runtime.md
```

只看核心链路：

```bash
python3 topic_inspect.py --topics /mobile_manipulator_mpc_observation /mobile_manipulator_mpc_target /mobile_manipulator_mpc_policy
```

---

## 6. 你接下来需要给我/确认的信息（用于把清单变成“可落地的 IDL/QoS 规格”）

如果你希望我继续把它细化到“DDS IDL 草案 + 每个 topic 的 QoS 推荐 + 映射规则”，需要你确认：

1) 你对端 DDS 具体实现（FastDDS / Connext / CycloneDDS / 其他）与版本
2) 对端想消费/发送哪些 topic（只要 MPC 三个？是否需要 joint_states？）
3) state/input 的维度与索引定义（有没有一份在代码/配置里写死的顺序）
4) 网络约束（同机/局域网/跨网段）与可接受带宽
