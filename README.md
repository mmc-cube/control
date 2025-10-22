# DM Balance Wheel-Legged 控制算法概览

## 1. 项目概述
- 平台：STM32H7 + FreeRTOS + BMI088 + DM4310/DM6215 电机，采用 MIT 模式力矩控制。
- 工程结构：`Core`/`Drivers`/`Middlewares` 提供基础硬件与 RTOS 支撑，核心业务集中在 `User` 目录。
- 控制目标：实现轮腿机器人在平地行驶、姿态保持、跳跃以及倒地自起等功能，强调姿态估计与虚拟模型控制 (VMC) 的闭环协同。

## 2. 系统任务与数据流
```
BMI088 → INS_task → 传感器融合 (Mahony / EKF) → 姿态/加速度
                           ↓
                    observe_task → 速度/位移估计 (KF)
                           ↓
 PS2 / SBUS → remote_task, ps2_task → 操作模式 + 期望指令
                           ↓
          chassisR_task & chassisL_task → LQR + VMC + PID → 扭矩指令 → CAN (DM 电机)
```
- `chassis_move`（定义于 `User/Controller`）为多任务共享的状态容器，包含姿态/速度估计、设定值、模式标志与跳跃状态机信息。
- 任务周期：INS ≈1 ms，Observe ≈3 ms，Chassis R/L ≈1 ms，Remote/PS2 ≈10 ms，Watch ≈2 s。

## 3. Algorithm 目录要点
### 3.1 EKF (`User/Algorithm/EKF/QuaternionEKF.c`)
- 基于扩展卡尔曼滤波进行四元数姿态估计，并同时估计陀螺零偏。
- 自定义 `User_FuncX_f` 扩展 Kalman 流程：线性化 F/H、应用 fading 技术、卡方检验拒绝异常量测（`QuaternionEKF.c:34` 起）。
- 适用于运动平稳阶段的精确姿态修正；目前 `INS_task` 主要调用 Mahony，但保留 EKF 以便切换或用于离线调试。

### 3.2 Mahony (`User/Algorithm/mahony/mahony_filter.c`)
- 实现 Mahony 互补滤波器，通过加速度矢量约束陀螺漂移。
- 提供 `mahony_input/update/output` 以及旋转矩阵更新函数，被 `INS_task` 直接调用获取四元数与欧拉角。

### 3.3 Kalman (`User/Algorithm/kalman/kalman_filter.c`)
- 通用矩阵化 EKF 框架，支持配置状态维度、量测维度、过程/观测噪声以及用户自定义 Hook。
- 广泛用于姿态 EKF 与 `observe_task` 中的速度/位移融合。

### 3.4 PID (`User/Algorithm/PID/pid.c`)
- 提供位置式/增量式 PID，支持积分限幅、导数滤波、输出 RC 滤波、误差处理等增强特性。
- `chassisR_task`、`chassisL_task` 的腿长调节、横滚补偿、防劈叉补偿均调用该模块。

### 3.5 VMC (`User/Algorithm/VMC/VMC_calc.c`)
- 虚拟模型控制核心：根据几何关系推导腿长 L₀、姿态角 θ 及一阶导数，并将足端力/力矩映射至关节扭矩。
- 包含：
  - `VMC_calc_1_left/right`：完成几何逆解、低通滤波与状态导数计算。
  - `VMC_calc_2`：利用雅可比矩阵求解关节扭矩 (`VMC_calc.c:60` 起)。
  - `ground_detectionL/R`：通过足端法向力估计实现离地判定。
  - `LQR_K_calc`：按腿长查表计算多项式拟合的 LQR 系数。

## 4. APP 目录任务说明
### 4.1 `INS_task.c`
- 初始化 Mahony 滤波，读取 BMI088 三轴加速度/角速度，输出四元数、欧拉角以及去重力后的运动加速度 (`User/APP/INS_task.c:34`)。
- 对加速度进行自适应低通与阈值截断，计算速度、位移并保持航向环绕计数。

### 4.2 `observe_task.c`
- 结合左右轮速、腿部运动学和 IMU 加速度，使用二维 Kalman 滤波得到机体前向速度与位移 (`User/APP/observe_task.c:34`)。
- 更新 `chassis_move.v_filter`、`x_filter`，并为跳跃/离地逻辑提供速度依据。

### 4.3 `remote_task.c` 与 `ps2_task.c`
- 解析 SBUS 与 PS2 遥控器，输出速度、转向、腿长、滚转等设定值，并管理启停、跳跃、倒地自起等状态机。
- `remote_task.c` 处理供电模式、自定义按键组合和跳跃三阶段触发；`ps2_task.c` 提供手柄特有的按键映射与冗余校验。

### 4.4 `chassisR_task.c` 与 `chassisL_task.c`
- 分别管控左右腿：初始化 CAN 上的 DM4310/DM6215，读取反馈后调用 VMC + LQR + PID 生成扭矩指令 (`User/APP/chassisR_task.c:52`, `User/APP/chassisL_task.c:50`)。
- 跳跃逻辑划分压缩/上升/缩腿三阶段，对应不同 L₀ 目标与扭矩限幅，双腿通过 `jump_flag`/`jump_flag2` 协同。
- 离地检测成功时将轮毂扭矩置零，仅保留髋关节姿态补偿；倒地自起模式下重置腿部输出。

### 4.5 其他任务
- `watch_task.c`：监控总线电压，触发蜂鸣器预警。
- `chassis_move`相关辅助函数位于 `User/Controller/controller.c/h`，涵盖模糊 PID、前馈控制、MIT 协议报文打包等。

## 5. 控制流程拆解
1. **传感器融合**：Mahony 输出姿态，结合重力补偿得到机体系加速度；必要时可切换 EKF 以提升静态精度。
2. **状态观测**：`observe_task` 将 IMU 与 VMC 推导的足端速度组合，得到更平滑的前向速度/位移。
3. **遥控解析**：根据不同遥控器来源设置 `chassis_move` 的速度、姿态、腿长及模式标志。
4. **力矩求解**：
   - LQR：采用腿长相关的三次多项式系数，针对 θ、θ̇、x、ẋ、Pitch、Pitcḣ 计算轮毂扭矩与髋关节力矩。
   - VMC：根据几何关系逆解 `phi0~phi4` 与 L₀，再由期望足端力 F₀、髋关节力矩 Tp 映射至关节扭矩。
   - PID/补偿：腿长保持、滚转/偏航补偿、防劈叉等通过 PID 实现；跳跃期间调整前馈与限幅。
5. **执行输出**：通过 `mit_ctrl`/`mit_ctrl2` 将扭矩命令下发至各 DM 电机，实现左右腿同步执行。

## 6. 调参与扩展建议
- **姿态估计**：Mahony 的 `Kp/Ki`、`AccelLPF` 直接影响响应速度与抗冲击能力；若需增强静态精度可启用 `QuaternionEKF` 并调整过程/测量噪声。
- **LQR 系数**：`Poly_Coefficient[12][4]` 控制不同腿长下的反馈增益，可通过离线仿真优化，需同步左右两侧。
- **PID**：`LEG_PID_*`、`ROLL_PID_*` 等定义于头文件，配合 `PID_init` 的积分/输出限幅，避免跳跃阶段超调。
- **跳跃逻辑**：`jump_flag` 系列决定阶段切换，建议配合地面检测阈值 (`ground_detection` 中的 3 N) 做好异常保护。
- **扩展接口**：保留 EKF 与模糊 PID，可扩展到更高自由度或加入力矩传感器；`controller` 模块的前馈结构支持进一步的模型预测控制。

## 7. Git 提交流程
- 文档位于 `docs/algorithm-overview.md`，后续若调整算法或参数，请同步更新该文档，保持对外说明的一致性。

