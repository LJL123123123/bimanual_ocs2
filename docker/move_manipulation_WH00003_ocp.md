# move_manipulation_WH00003 OCP 说明文档

## 1. 文档目的

本文档说明 `ocs2_mobile_manipulator` 在 `WH00003` 工程中实际构建的最优控制问题（OCP, Optimal Control Problem），并结合代码结构与数学公式解释各个组成部分。

本文档覆盖以下内容：

- 状态与输入定义
- 不同机器人模型对应的动力学
- 输入代价
- 关节限位与速度软限制
- 末端位姿约束
- 自碰撞避免约束
- 腰部 roll/pitch 约束
- 质心多边形约束
- 终端末端约束
- 最终完整 OCP 形式

当前 `WH00003` 的 OCP 可概括为：

**输入代价 + 关节限位 + 速度软限制 + 末端位姿 + 碰撞避免 + 腰部 roll/pitch + 质心多边形 + 终端末端约束**

---

## 2. OCP 总体形式

`ocs2_mobile_manipulator` 构造的是一个**连续时间、有限时域、带软约束的轨迹优化问题**：

\[
\begin{aligned}
\min_{x(\cdot),u(\cdot)} \quad
& \Phi\big(x(T),T\big) + \int_{t_0}^{T} L\big(x(t),u(t),t\big)\,dt \\
\text{s.t.}\quad
& \dot{x}(t) = f\big(x(t),u(t)\big), \\
& x(t_0) = x_0.
\end{aligned}
\]

其中：

- \(x(t)\)：状态轨迹
- \(u(t)\)：输入轨迹
- \(f(x,u)\)：系统动力学
- \(L(x,u,t)\)：运行代价
- \(\Phi(x(T),T)\)：终端代价或终端软约束代价

在 `ocs2_mobile_manipulator` 中，问题是通过 `MobileManipulatorInterface.cpp` 装配的，主要入口包括：

- `problem_.dynamicsPtr.reset(...)`
- `problem_.costPtr->add(...)`
- `problem_.softConstraintPtr->add(...)`
- `problem_.stateSoftConstraintPtr->add(...)`
- `problem_.finalSoftConstraintPtr->add(...)`

---

## 3. 状态与输入定义

该工程支持多种机器人模型类型，对应不同的状态和输入定义。相关代码主要位于：

- `src/FactoryFunctions.cpp`
- `include/ocs2_mobile_manipulator/ManipulatorModelInfo.h`
- `src/dynamics/*.cpp`

### 3.1 DefaultManipulator

这是最简单的模型。

状态：

\[
x = q
\]

输入：

\[
u = \dot{q}
\]

动力学：

\[
\dot{x} = u
\]

对应代码：

- `src/dynamics/DefaultManipulatorDynamics.cpp`

```cpp
return input;
```

---

### 3.2 FloatingArmManipulator

状态由浮动基和机械臂关节组成：

\[
x = \begin{bmatrix} x_b \\ q \end{bmatrix}, \qquad x_b \in \mathbb{R}^6
\]

输入只控制机械臂关节：

\[
u = \dot{q}
\]

动力学：

\[
\dot{x}=
\begin{bmatrix}
0 \\
u
\end{bmatrix}
\]

对应代码：

- `src/dynamics/FloatingArmManipulatorDynamics.cpp`

---

### 3.3 FullyActuatedFloatingArmManipulator

状态：

\[
x = \begin{bmatrix} x_b \\ q \end{bmatrix}
\]

输入：

\[
u = \begin{bmatrix} \dot{x}_b \\ \dot{q} \end{bmatrix}
\]

动力学：

\[
\dot{x} = u
\]

对应代码：

- `src/dynamics/FullyActuatedFloatingArmManipulatorDynamics.cpp`

---

### 3.4 WheelBasedMobileManipulator

这是 `WH00003` 这类移动操作器最关键的模型。

状态定义为：

\[
x=
\begin{bmatrix}
p_x \\
p_y \\
\psi \\
q
\end{bmatrix}
\]

其中：

- \(p_x,p_y\)：底盘在世界系中的位置
- \(\psi\)：底盘 yaw
- \(q\)：机械臂关节角

输入定义为：

\[
u=
\begin{bmatrix}
v \\
\omega \\
\dot{q}
\end{bmatrix}
\]

其中：

- \(v\)：底盘前向速度
- \(\omega\)：底盘角速度
- \(\dot{q}\)：机械臂关节速度

动力学为：

\[
\dot{x}=
\begin{bmatrix}
\cos\psi \, v \\
\sin\psi \, v \\
\omega \\
\dot{q}
\end{bmatrix}
\]

对应代码：

- `src/dynamics/WheelBasedMobileManipulatorDynamics.cpp`

这说明该工程中的 mobile manipulator 采用的是**一阶运动学模型**，而不是刚体动力学模型。

---

## 4. 输入代价

### 4.1 数学形式

系统添加了输入二次型代价：

\[
L_u(x,u,t)=\frac12 (u-u_d(t))^\top R (u-u_d(t))
\]

大多数情况下，desired input \(u_d(t)\) 为零或不显式使用，因此可近似看成：

\[
L_u(x,u,t)=\frac12 u^\top R u
\]

其中：

- \(R \succeq 0\)：输入权重矩阵
- 从 `task.info` 读取

### 4.2 代码位置

- `src/MobileManipulatorInterface.cpp`
- `include/ocs2_mobile_manipulator/cost/QuadraticInputCost.h`

装配代码：

```cpp
problem_.costPtr->add("inputCost", getQuadraticInputCost(taskFile));
```

### 4.3 物理意义

输入代价的作用是：

- 抑制过大的底盘速度
- 抑制过大的关节速度
- 提高轨迹平滑性
- 在多个可行解中偏向控制更小的解

---

## 5. 关节限位与速度软限制

### 5.1 代码位置

- `src/MobileManipulatorInterface.cpp`

装配代码：

```cpp
problem_.softConstraintPtr->add("jointLimits", getJointLimitSoftConstraint(...));
```

---

### 5.2 关节位置约束

若 `jointPositionLimits.activate = true`，则根据 URDF 和 task 参数生成状态盒约束：

\[
q_i^{\min} \le q_i \le q_i^{\max}
\]

它不是硬约束，而是以 relaxed barrier 的形式转成软约束代价：

\[
L_{\text{joint-pos}}(x)=\sum_i \beta_q(q_i)
\]

其中 \(\beta_q\) 是 relaxed barrier 型惩罚。

---

### 5.3 关节速度约束

系统还会给输入加速度盒约束：

\[
u_i^{\min} \le u_i \le u_i^{\max}
\]

对 wheel-based mobile manipulator 来说，这包括：

- 底盘线速度约束
- 底盘角速度约束
- 各关节速度约束

其软约束形式可写为：

\[
L_{\text{joint-vel}}(u)=\sum_i \beta_u(u_i)
\]

---

## 6. 末端位姿约束

末端位姿约束是 OCP 的核心任务项之一。

### 6.1 代码位置

装配位置：

- `src/MobileManipulatorInterface.cpp`

约束实现：

- `src/constraint/EndEffectorConstraint.cpp`

装配形式：

```cpp
problem_.stateSoftConstraintPtr->add(...);
problem_.finalSoftConstraintPtr->add(...);
```

---

### 6.2 数学形式

设某个末端执行器 frame 的正运动学为：

- 位置：\(p_{ee}(x)\in\mathbb{R}^3\)
- 姿态：\(R_{ee}(x)\in SO(3)\)

其期望目标为：

- \(p_d(t)\)
- \(R_d(t)\)

则残差定义为：

\[
e_{ee}(x,t)=
\begin{bmatrix}
p_{ee}(x)-p_d(t) \\
e_R(R_{ee}(x),R_d(t))
\end{bmatrix}
\in \mathbb{R}^6
\]

其中 \(e_R\) 是姿态误差向量。

对应的软约束代价可理解为：

\[
L_{ee}(x,t)=
\frac12 \mu_p \|p_{ee}(x)-p_d(t)\|^2
+
\frac12 \mu_R \|e_R(R_{ee}(x),R_d(t))\|^2
\]

---

### 6.3 双 ee 扩展

在 `WH00003` 工程中，系统已支持双末端目标：

- 右手 `right_wrist_pitch_link`
- 左手 `left_wrist_pitch_link`

其目标轨迹向量按如下方式组织：

\[
x_{target}=
[p_R(3), q_R(4), p_L(3), q_L(4)] \in \mathbb{R}^{14}
\]

因此末端约束可以扩展为：

\[
L_{ee}(x,t)=L_R(x,t)+L_L(x,t)
\]

---

## 7. 自碰撞避免约束

### 7.1 代码位置

- `src/MobileManipulatorInterface.cpp`

装配代码：

```cpp
problem_.stateSoftConstraintPtr->add("selfCollision", ...);
```

### 7.2 数学形式

设若干碰撞对的最小距离为：

\[
d_k(x), \quad k=1,\dots,N_c
\]

要求：

\[
d_k(x)\ge d_{\min}
\]

通过 relaxed barrier 方式转为软约束：

\[
L_{\text{collision}}(x)=\sum_{k=1}^{N_c}\beta_c\big(d_k(x)-d_{\min}\big)
\]

其中：

- \(d_k(x)\)：第 \(k\) 个碰撞对的最小距离
- \(d_{\min}\)：最小安全距离

---

## 8. 腰部 roll/pitch 约束

这是 `WH00003` 工程中新增加的约束。

### 8.1 设计目标

约束 `waist_yaw_link` 保持近似直立，即：

- 抑制 roll
- 抑制 pitch
- 不限制 yaw

### 8.2 数学含义

在 URDF 中为 `waist_yaw_link` 增加一个沿“局部竖直方向”偏移的参考点。
通过约束该参考点在世界系下相对根 frame 的横向偏移为零，实现“竖直”效果。

若定义误差向量为：

\[
e_{\text{waist}}(x)=
\begin{bmatrix}
e_r(x) \\
e_p(x)
\end{bmatrix}
\]

则约束代价可写为：

\[
L_{\text{waist}}(x)=
\frac12 \mu_r e_r(x)^2 + \frac12 \mu_p e_p(x)^2
\]

### 8.3 代码位置

- `include/ocs2_mobile_manipulator/constraint/LinkVerticalConstraint.h`
- `src/constraint/LinkVerticalConstraint.cpp`
- `src/MobileManipulatorInterface.cpp`

---

## 9. 质心多边形约束

这是在 `WH00003` OCP 中新加入的约束。

### 9.1 设计目标

要求机器人质心在 `task.info` 中配置的 `baseFrame` 局部坐标系下，落在某个凸多边形内部。
该多边形：

- 以 `baseFrame` 为参考
- 顶点数量由 `task.info` 配置
- 每个顶点相对 `baseFrame` 的二维偏置由 `task.info` 配置

### 9.2 质心坐标变换

设：

- 世界系下 `baseFrame` 位姿为 \(T_{WB}=(R_{WB},p_{WB})\)
- 世界系下质心位置为 \(p_{com}^W\)

则质心在 `baseFrame` 下的坐标为：

\[
p_{com}^B = R_{WB}^\top (p_{com}^W - p_{WB})
\]

取其二维投影：

\[
c=
\begin{bmatrix}
c_x \\
c_y
\end{bmatrix}
=
\begin{bmatrix}
p_{com,x}^B \\
p_{com,y}^B
\end{bmatrix}
\]

---

### 9.3 多边形半空间表示

设 task 文件中按逆时针（CCW）顺序给出的顶点为：

\[
v_i=
\begin{bmatrix}
x_i \\
y_i
\end{bmatrix},
\quad i=0,\dots,N-1
\]

定义边：

\[
e_i = v_{i+1}-v_i
\]

对于 CCW 多边形，其外法向量取为：

\[
n_i=
\begin{bmatrix}
e_{i,y} \\
- e_{i,x}
\end{bmatrix}
\]

则第 \(i\) 条边对应的约束为：

\[
g_i(c)=n_i^\top(c-v_i)\le 0
\]

整个 CoM polygon 约束写成：

\[
g(c)=
\begin{bmatrix}
g_0(c)\\
g_1(c)\\
\vdots\\
g_{N-1}(c)
\end{bmatrix}
\le 0
\]

---

### 9.4 one-sided penalty 形式

为了只在越界时惩罚，而在多边形内部不惩罚，定义 one-sided quadratic penalty：

\[
\phi(h)=\frac12 \mu \max(h,0)^2
\]

因此 CoM polygon 软约束项为：

\[
L_{\text{com-poly}}(x)=
\sum_{i=0}^{N-1}
\frac12 \mu \max(g_i(c(x)),0)^2
\]

满足：

- \(g_i(c)\le 0\)：不罚
- \(g_i(c)>0\)：越界，按二次项惩罚

### 9.5 代码位置

新增文件：

- `include/ocs2_mobile_manipulator/constraint/ComPolygonConstraint.h`
- `src/constraint/ComPolygonConstraint.cpp`
- `include/ocs2_mobile_manipulator/cost/OneSidedQuadraticPenalty.h`

装配位置：

- `src/MobileManipulatorInterface.cpp`

task 参数：

- `comPolygon.activate`
- `comPolygon.numVertices`
- `comPolygon.vertices`
- `comPolygon.mu`

---

## 10. 终端末端约束

除了路径上的 ee tracking 外，系统还对终端时刻增加末端跟踪项。
其作用是保证到达时刻的末端位置与姿态更接近期望。

数学上可记为：

\[
\Phi_{\text{ee}}(x(T),T)
=
\sum_{m \in \mathcal{E}}
\left[
\frac12 \mu_{p,m}^{f}\|p_m(x(T))-p_{d,m}(T)\|^2
+
\frac12 \mu_{R,m}^{f}\|e_{R,m}(x(T),T)\|^2
\right]
\]

其中 \(\mathcal{E}\) 是启用的末端集合。

代码中对应：

```cpp
problem_.finalSoftConstraintPtr->add(...);
```

---

## 11. 最终 WH00003 OCP 形式

综合以上各项，当前 `WH00003` 工程中的 OCP 可写为：

\[
\begin{aligned}
\min_{x(\cdot),u(\cdot)}\quad
&
\int_{t_0}^{T}
\Big[
\underbrace{\frac12 (u-u_d)^\top R (u-u_d)}_{\text{输入代价}}
+
\underbrace{\sum_i \beta_q(q_i)}_{\text{关节位置软限制}}
+
\underbrace{\sum_j \beta_u(u_j)}_{\text{速度软限制}}
\\
&\qquad
+
\underbrace{\sum_{m\in\mathcal{E}}
\left(
\frac12 \mu_{p,m}\|p_m(x)-p_{d,m}\|^2
+
\frac12 \mu_{R,m}\|e_{R,m}(x)\|^2
\right)}_{\text{末端位姿约束}}
\\
&\qquad
+
\underbrace{\sum_k \beta_c(d_k(x)-d_{\min})}_{\text{碰撞避免}}
+
\underbrace{\frac12 \mu_r e_r(x)^2 + \frac12 \mu_p e_p(x)^2}_{\text{腰部 roll/pitch}}
\\
&\qquad
+
\underbrace{\sum_{\ell=0}^{N-1}\frac12 \mu_{com}\max(g_\ell(c(x)),0)^2}_{\text{质心多边形}}
\Big]dt
\\
&\quad
+
\underbrace{\Phi_{\text{ee}}(x(T),T)}_{\text{终端末端约束}}
\\[1mm]
\text{s.t.}\quad
& \dot{x}=f(x,u), \\
& x(t_0)=x_0.
\end{aligned}
\]

其中：

- \(f(x,u)\)：由机器人模型类型决定
- \(\mathcal{E}\)：启用的末端集合
  - 单 ee 模式：通常只有右手
  - 双 ee 模式：右手 + 左手
- \(g_\ell(c(x))\le 0\)：CoM 落在 baseFrame 局部多边形内部

---

## 12. 从代码结构理解 OCP 装配

在 `MobileManipulatorInterface.cpp` 中，OCP 的装配逻辑大致如下：

1. **读取模型与配置**
   - URDF
   - `task.info`
   - 机器人模型类型
   - 初始状态

2. **设置动力学**
   - `problem_.dynamicsPtr.reset(...)`

3. **添加输入代价**
   - `problem_.costPtr->add("inputCost", ...)`

4. **添加关节限位与速度软限制**
   - `problem_.softConstraintPtr->add("jointLimits", ...)`

5. **添加末端位姿约束**
   - `problem_.stateSoftConstraintPtr->add(...)`
   - `problem_.finalSoftConstraintPtr->add(...)`

6. **添加碰撞避免**
   - `problem_.stateSoftConstraintPtr->add("selfCollision", ...)`

7. **添加腰部 roll/pitch 约束**
   - `problem_.stateSoftConstraintPtr->add("waistRollPitch", ...)`

8. **添加质心多边形约束**
   - `problem_.stateSoftConstraintPtr->add("comPolygon", ...)`

---

## 13. 对该 OCP 的理解总结

当前 `WH00003` 工程构建的不是刚体动力学层面的力矩优化问题，而是一个**基于运动学模型、几何任务与软约束的最优控制问题**。

其本质特征包括：

1. **输入是速度型输入**
   - 底盘速度
   - 关节速度

2. **动力学是一阶运动学模型**
   - 特别是 wheel-based 模型使用非完整底盘运动学

3. **任务主要以几何约束实现**
   - 末端位姿
   - 自碰撞
   - 腰部姿态
   - CoM polygon

4. **大部分约束使用 soft constraint**
   - 约束违反时允许存在，但会产生较大代价

因此，这个 OCP 更准确地说是：

> 一个以速度控制为输入、以末端任务和姿态/几何软约束为核心的移动操作器轨迹优化问题。

---

## 14. 最值得关注的代码文件

若要继续深入理解或修改该 OCP，最关键的代码文件包括：

1. `src/MobileManipulatorInterface.cpp`
   - OCP 的主装配入口

2. `src/FactoryFunctions.cpp`
   - 状态维度、输入维度、模型类型定义

3. `src/dynamics/WheelBasedMobileManipulatorDynamics.cpp`
   - 移动底盘 + 机械臂的一阶运动学模型

4. `src/constraint/EndEffectorConstraint.cpp`
   - 末端位姿约束

5. `src/constraint/LinkVerticalConstraint.cpp`
   - 腰部直立约束

6. `src/constraint/ComPolygonConstraint.cpp`
   - 质心多边形约束

7. `getJointLimitSoftConstraint(...)`
   - 关节限位和速度限制

8. `getSelfCollisionConstraint(...)`
   - 自碰撞软约束

---

## 15. 后续可扩展方向

从 OCP 角度，后续还可以继续扩展：

- 解析 Jacobian 版本的 CoM polygon 约束
- 终端 CoM polygon 约束
- 非对称关节代价
- 更完整的双臂自碰撞对
- 质心速度/姿态耦合约束
- 地面接触或支撑多边形动态变化

这些扩展都可以继续沿用当前工程的：

- `StateSoftConstraint`
- `StateCost`
- `StateInputCost`

这一套 OCS2 装配风格。

---
