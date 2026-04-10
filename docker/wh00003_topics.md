# WH00003 启动 topic 一览（单行表格）

补充：如果你在做“真仿真/真机器人对接”，经常需要知道 `SystemObservation.state[]/input[]` 的每个索引对应哪个关节/量。索引映射见：
- [src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/docker/wh00003_state_layout.md](src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/docker/wh00003_state_layout.md)

表格列定义：

- `语义`：脚本内置的少量解释（无法从 ROS 自动推断的部分会留空）
- `字段结构(压缩)`：来自 `rosmsg show`，压缩成单行便于表格查看
- `平均大小/带宽估计`：来自对 topic 的采样估计（取决于 `--duration` 和当时系统是否正在发布）

<!-- TOPIC_TABLE:BEGIN -->

| topic | 语义 | type | 字段结构(压缩) | .msg位置/命令 | 谁发谁收 | 发布行为 | 平均大小 | 带宽估计 |
|---|---|---|---|---|---|---|---:|---:|
| /clicked_point | RViz 工具: Publish Point | geometry_msgs/PointStamped | std_msgs/Header header; uint32 seq; time stamp; string frame_id; geometry_msgs/Point point; float64 x; float64 y; float64 z | /opt/ros/noetic/share/geometry_msgs/msg/PointStamped.msg | pub:/rviz; sub:- | 触发/空闲(采样0.5s无数据) | - | - |
| /distance_markers | 距离/约束等可视化(MarkerArray) | visualization_msgs/MarkerArray | visualization_msgs/Marker[] markers; std_msgs/Header header; uint32 seq; time stamp; string frame_id; string ns; int32 id; int32 type; int32 action; geometry_msgs/Pose pose; geometry_msgs/Point position; float64 x; floa… | /opt/ros/noetic/share/visualization_msgs/msg/MarkerArray.msg | pub:/mobile_manipulator_dummy_mrt_node; sub:/rviz | 持续(~401.0 Hz) | 22932 B | 8979.1 KiB/s |
| /initialpose | RViz 工具: 2D Pose Estimate | geometry_msgs/PoseWithCovarianceStamped | std_msgs/Header header; uint32 seq; time stamp; string frame_id; geometry_msgs/PoseWithCovariance pose; geometry_msgs/Pose pose; geometry_msgs/Point position; float64 x; float64 y; float64 z; geometry_msgs/Quaternion or… | /opt/ros/noetic/share/geometry_msgs/msg/PoseWithCovarianceStamped.msg | pub:/rviz; sub:- | 触发/空闲(采样0.5s无数据) | - | - |
| /joint_states | 关节状态(通常由驱动/仿真发布) | sensor_msgs/JointState | std_msgs/Header header; uint32 seq; time stamp; string frame_id; string[] name; float64[] position; float64[] velocity; float64[] effort | /opt/ros/noetic/share/sensor_msgs/msg/JointState.msg | pub:-; sub:/robot_state_publisher | 无发布者 | - | - |
| /mobile_manipulator/optimizedPoseTrajectory | 优化轨迹可视化(PoseArray) | geometry_msgs/PoseArray | std_msgs/Header header; uint32 seq; time stamp; string frame_id; geometry_msgs/Pose[] poses; geometry_msgs/Point position; float64 x; float64 y; float64 z; geometry_msgs/Quaternion orientation; float64 x; float64 y; flo… | /opt/ros/noetic/share/geometry_msgs/msg/PoseArray.msg | pub:/mobile_manipulator_dummy_mrt_node; sub:/rviz | 持续(~399.4 Hz) | 249 B | 97.1 KiB/s |
| /mobile_manipulator/optimizedStateTrajectory | 优化状态可视化(MarkerArray) | visualization_msgs/MarkerArray | visualization_msgs/Marker[] markers; std_msgs/Header header; uint32 seq; time stamp; string frame_id; string ns; int32 id; int32 type; int32 action; geometry_msgs/Pose pose; geometry_msgs/Point position; float64 x; floa… | /opt/ros/noetic/share/visualization_msgs/msg/MarkerArray.msg | pub:/mobile_manipulator_dummy_mrt_node; sub:/rviz | 持续(~399.1 Hz) | 542 B | 211.3 KiB/s |
| /mobile_manipulator_external_observation | - | ocs2_msgs/mpc_observation | float64 time; ocs2_msgs/mpc_state state; float32[] value; ocs2_msgs/mpc_input input; float32[] value; int8 mode | /root/ocs2_ws/src/ocs2/ocs2_msgs/msg/mpc_observation.msg | pub:-; sub:/mobile_manipulator_dummy_mrt_node | 无发布者 | - | - |
| /mobile_manipulator_mode_schedule | (可选) mode schedule 输入 | ocs2_msgs/mode_schedule | float64[] eventTimes; int8[] modeSequence | /root/ocs2_ws/src/ocs2/ocs2_msgs/msg/mode_schedule.msg | pub:-; sub:/mobile_manipulator_mpc_node | 无发布者 | - | - |
| /mobile_manipulator_mpc_observation | MPC 观测(当前 time/state/input/mode) | ocs2_msgs/mpc_observation | float64 time; ocs2_msgs/mpc_state state; float32[] value; ocs2_msgs/mpc_input input; float32[] value; int8 mode | /root/ocs2_ws/src/ocs2/ocs2_msgs/msg/mpc_observation.msg | pub:/mobile_manipulator_dummy_mrt_node; sub:/mobile_manipulator_target,/mobile_manipulator_mpc_node | 持续(~99.8 Hz) | 181 B | 17.6 KiB/s |
| /mobile_manipulator_mpc_policy | MPC 求解结果(策略/轨迹/性能指标) | ocs2_msgs/mpc_flattened_controller | uint8 controllerType; ocs2_msgs/mpc_observation initObservation; float64 time; ocs2_msgs/mpc_state state; float32[] value; ocs2_msgs/mpc_input input; float32[] value; int8 mode; ocs2_msgs/mpc_target_trajectories planTar… | /root/ocs2_ws/src/ocs2/ocs2_msgs/msg/mpc_flattened_controller.msg | pub:/mobile_manipulator_mpc_node; sub:/mobile_manipulator_dummy_mrt_node | 持续(~101.7 Hz) | 1459 B | 145.0 KiB/s |
| /mobile_manipulator_mpc_target | MPC 目标轨迹(TargetTrajectories) | ocs2_msgs/mpc_target_trajectories | float64[] timeTrajectory; ocs2_msgs/mpc_state[] stateTrajectory; float32[] value; ocs2_msgs/mpc_input[] inputTrajectory; float32[] value | /root/ocs2_ws/src/ocs2/ocs2_msgs/msg/mpc_target_trajectories.msg | pub:/mobile_manipulator_target; sub:/mobile_manipulator_mpc_node | 触发/空闲(采样0.5s无数据) | - | - |
| /move_base_simple/goal | RViz 工具: 2D Nav Goal | geometry_msgs/PoseStamped | std_msgs/Header header; uint32 seq; time stamp; string frame_id; geometry_msgs/Pose pose; geometry_msgs/Point position; float64 x; float64 y; float64 z; geometry_msgs/Quaternion orientation; float64 x; float64 y; float6… | /opt/ros/noetic/share/geometry_msgs/msg/PoseStamped.msg | pub:/rviz; sub:- | 触发/空闲(采样0.5s无数据) | - | - |
| /rosout | ROS 日志 | rosgraph_msgs/Log | byte DEBUG=1; byte INFO=2; byte WARN=4; byte ERROR=8; byte FATAL=16; std_msgs/Header header; uint32 seq; time stamp; string frame_id; byte level; string name; string msg; string file; string function; uint32 line; strin… | /opt/ros/noetic/share/rosgraph_msgs/msg/Log.msg | pub:/robot_state_publisher,/mobile_manipulator_mpc_node,/mobile_manipulator_target,/mobile_manipulator_dummy_mrt_node,/rviz; sub:/rosout | 触发/空闲(采样0.5s无数据) | - | - |
| /rosout_agg | ROS 日志聚合 | rosgraph_msgs/Log | byte DEBUG=1; byte INFO=2; byte WARN=4; byte ERROR=8; byte FATAL=16; std_msgs/Header header; uint32 seq; time stamp; string frame_id; byte level; string name; string msg; string file; string function; uint32 line; strin… | /opt/ros/noetic/share/rosgraph_msgs/msg/Log.msg | pub:/rosout; sub:- | 触发/空闲(采样0.5s无数据) | - | - |
| /simple_marker/feedback | InteractiveMarker 交互反馈(RViz->server) | visualization_msgs/InteractiveMarkerFeedback | std_msgs/Header header; uint32 seq; time stamp; string frame_id; string client_id; string marker_name; string control_name; uint8 event_type; geometry_msgs/Pose pose; geometry_msgs/Point position; float64 x; float64 y; … | /opt/ros/noetic/share/visualization_msgs/msg/InteractiveMarkerFeedback.msg | pub:/rviz; sub:/mobile_manipulator_target | 触发/空闲(采样0.5s无数据) | - | - |
| /simple_marker/update | InteractiveMarker 增量更新(server->RViz) | visualization_msgs/InteractiveMarkerUpdate | string server_id; uint64 seq_num; uint8 type; visualization_msgs/InteractiveMarker[] markers; std_msgs/Header header; uint32 seq; time stamp; string frame_id; geometry_msgs/Pose pose; geometry_msgs/Point position; float… | /opt/ros/noetic/share/visualization_msgs/msg/InteractiveMarkerUpdate.msg | pub:/mobile_manipulator_target; sub:/rviz | 持续(~2.0 Hz) | 51 B | 0.1 KiB/s |
| /simple_marker/update_full | InteractiveMarker 全量初始化(server->client) | visualization_msgs/InteractiveMarkerInit | string server_id; uint64 seq_num; visualization_msgs/InteractiveMarker[] markers; std_msgs/Header header; uint32 seq; time stamp; string frame_id; geometry_msgs/Pose pose; geometry_msgs/Point position; float64 x; float6… | /opt/ros/noetic/share/visualization_msgs/msg/InteractiveMarkerInit.msg | pub:/mobile_manipulator_target; sub:- | 持续(~2.0 Hz) | 1358 B | 2.6 KiB/s |
| /tf | 动态 TF | tf2_msgs/TFMessage | geometry_msgs/TransformStamped[] transforms; std_msgs/Header header; uint32 seq; time stamp; string frame_id; string child_frame_id; geometry_msgs/Transform transform; geometry_msgs/Vector3 translation; float64 x; float… | /opt/ros/noetic/share/tf2_msgs/msg/TFMessage.msg | pub:/robot_state_publisher,/mobile_manipulator_dummy_mrt_node; sub:/rviz | 持续(~1596.1 Hz) | 594 B | 925.5 KiB/s |
| /tf_static | 静态 TF | tf2_msgs/TFMessage | geometry_msgs/TransformStamped[] transforms; std_msgs/Header header; uint32 seq; time stamp; string frame_id; string child_frame_id; geometry_msgs/Transform transform; geometry_msgs/Vector3 translation; float64 x; float… | /opt/ros/noetic/share/tf2_msgs/msg/TFMessage.msg | pub:/robot_state_publisher,/mobile_manipulator_dummy_mrt_node; sub:/rviz | 持续(~4.0 Hz) | 116 B | 0.5 KiB/s |

<!-- TOPIC_TABLE:END -->

## 结构文件位置补充

- 自定义 OCS2 消息包路径：`rospack find ocs2_msgs`（当前工作区为 `/root/ocs2_ws/src/ocs2/ocs2_msgs`）
- OCS2 `.msg` 文件目录：`/root/ocs2_ws/src/ocs2/ocs2_msgs/msg/`
- 标准消息包路径：
  - `rospack find geometry_msgs` → `/opt/ros/noetic/share/geometry_msgs`
  - `rospack find sensor_msgs` → `/opt/ros/noetic/share/sensor_msgs`
  - `rospack find visualization_msgs` → `/opt/ros/noetic/share/visualization_msgs`

## 生成/刷新表格（推荐用脚本）

在有 roscore+节点运行时执行：

```bash
cd /root/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/docker
source /root/ocs2_ws/devel/setup.bash

# 生成一个 runtime 表格文件
python3 topic_inspect.py --md > wh00003_topics_runtime.md

# 直接把表格写回本文件(覆盖标记段)
python3 topic_inspect.py --update-md wh00003_topics.md
```

如果只想看部分 topic：

```bash
cd /root/ocs2_ws/src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator_ros/docker
source /root/ocs2_ws/devel/setup.bash
python3 topic_inspect.py --topics /mobile_manipulator_mpc_observation /mobile_manipulator_mpc_policy --md
```
