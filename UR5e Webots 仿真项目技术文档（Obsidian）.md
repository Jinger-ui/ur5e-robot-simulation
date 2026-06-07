---
title: UR5e Webots 仿真项目技术文档
aliases:
  - UR5e 仿真技术文档
  - Webots UR5e 项目总结
  - 机器人仿真项目文档
tags:
  - webots
  - ur5e
  - robotics
  - simulation
  - pick-and-place
  - inverse-kinematics
  - python
  - gaiahand
  - ros2
created: 2026-06-07
---

# UR5e Webots 仿真项目技术文档

## 项目概述

本项目基于 **Webots R2025a** 仿真平台，构建了一个 **UR5e 工业机器人臂** 的完整 Pick-and-Place（抓取-放置）仿真系统。项目包含完整的工厂场景建模、逆运动学求解、轨迹规划、传感器集成与有限状态机控制逻辑。此外还集成了 **GaiaHand 开源灵巧手** 与 **webots_ros2** 作为扩展参考。

## 项目结构

```
solidworks/
├── robot_arm_3d_v1.scr            # AutoCAD 脚本：6-DOF 机械臂 + 夹爪
├── humanoid_hand_3d.scr           # AutoCAD 脚本：5 指仿人手
├── webots_ur5e_project/           # 核心仿真项目
│   ├── worlds/
│   │   ├── ur5e_complete.wbt      # 完整工厂场景（主用）
│   │   └── ur5e_demo.wbt          # 简化演示场景
│   └── controllers/
│       ├── ur5e_complete_controller/
│       │   ├── ur5e_complete_controller.py   # 完整控制器（18 状态 FSM）
│       │   └── config.json                   # JSON 配置文件
│       └── ur5e_controller/
│           └── ur5e_controller.py            # 基础控制器
├── webots_ros2/                   # Webots-ROS2 集成包（cyberbotics）
├── open_source_hand/              # GaiaHand 开源灵巧手
│   ├── GaiaHand/                  # 主仓库（硬件/固件/SDK）
│   ├── convert_stl_to_dxf.py     # STL → DXF 转换脚本
│   └── import_gaiahand_*.scr     # SolidWorks 导入脚本
├── SOLIDWORKS AI Overview - Obsidian.md
└── SOLIDWORKS 2025 安装步骤（Obsidian）.md
```

---

## 一、Webots 仿真场景

### 1.1 完整工厂场景（`ur5e_complete.wbt`）

| 参数 | 值 |
|---|---|
| Webots 版本 | R2025a |
| 仿真步长 | 8 ms |
| 场景尺寸 | 10 × 8 m |
| 物理引擎 | 内置（含碰撞属性） |

#### 场景元素

- **UR5e 机器人臂**：安装在 0.8m 高的金属底座上，朝向 -90°
- **Robotiq 3-Finger Gripper**：三指自适应抓手
- **传感器组**
  - `arm_camera`：320×240 摄像头，FOV 0.785 rad
  - `gripper_sensor`：DistanceSensor，安装于末端
  - `gripper_touch`：TouchSensor，碰撞检测
- **抓取目标（5 个）**
  - `target_red`：红色方块 (Box 0.05³)，0.2 kg
  - `target_green`：绿色圆柱 (h=0.06, r=0.025)，0.15 kg
  - `target_blue`：蓝色长方体 (0.04×0.04×0.06)，0.18 kg
  - `target_yellow`：黄色圆柱 (h=0.05, r=0.028)，0.14 kg
  - `target_orange`：橙色球体 (r=0.025)，0.12 kg
- **工作台**
  - `pick_table`：抓取台（x=0.65），灰白色
  - `place_table`：放置台（x=-0.65），蓝灰色
- **工厂环境**
  - 四面围墙 + 黄色安全围栏
  - 信号灯柱（红/黄/绿三灯）
  - 传送带（speed=0.15 m/s）
  - 木托盘堆、金属货架、纸箱
  - 灭火器
- **物理接触属性**
  - gripper-target 摩擦系数 = 2（增强抓取稳定性）
  - 默认 bounce = 0.05

### 1.2 简化演示场景（`ur5e_demo.wbt`）

与完整场景使用相同的 PROTO 和基本布局，但省略了部分环境装饰细节，适合快速调试。

---

## 二、控制器架构

### 2.1 完整控制器（`ur5e_complete_controller.py`）

这是项目的核心，实现了工业级的 Pick-and-Place 控制流程。

#### 技术栈

| 组件 | 说明 |
|---|---|
| Python | 控制器语言 |
| ikpy | 逆运动学求解库 |
| NumPy | 数学计算 |
| JSON | 外部配置 |
| Webots Controller API | 机器人交互 |

#### 运动学模型

使用 ikpy 构建 UR5e 的 DH 参数链：

| 关节 | 平移 (m) | 旋转轴 |
|---|---|---|
| shoulder_pan | [0, 0, 0.1625] | Z |
| shoulder_lift | [0, 0, 0] | Y（+π/2 偏转） |
| elbow | [0, -0.4250, 0] | Y |
| wrist_1 | [0, -0.3922, 0] | Y |
| wrist_2 | [0, 0, 0.1333] | Z |
| wrist_3 | [0, 0, 0.0997] | Y |
| ee_fixed | [0, -0.0996, 0] | — |

#### 逆运动学求解策略

- **多种子求解**：最多 5 次尝试（`ik_max_attempts`）
- **种子来源**：用户提供 seed → 零位 → 当前关节位置 → 随机采样
- **验证**：FK 反算误差 < 0.05 m 才接受
- **降级方案**：ikpy 不可用时，使用预定义的 fallback 关节角

#### 轨迹规划

实现了三种轨迹插值方式：

1. **梯形速度轨迹**（`trapezoidal_trajectory`）
   - 加速段 → 匀速段 → 减速段
   - 参数：`max_velocity = 1.5 rad/s`，`max_acceleration = 2.0 rad/s²`
   - 自动处理短距离（无匀速段）情况

2. **笛卡尔空间线性插值**（`cartesian_linear_path`）
   - 起点到终点均匀采样 → 逐点 IK → 梯形速度拼接
   - 适用于精确直线路径

3. **二次贝塞尔弧线**（`arc_path`）
   - 三点（起点、经由点、终点）定义曲线
   - 适用于需要绕过障碍的弧形路径

4. **避障路径**（`plan_obstacle_avoidance`）
   - 通过中间经由点（默认为修正后的 home 位姿）分段规划
   - 避免从 pick 侧到 place 侧的直接碰撞

#### 有限状态机（18 状态）

```
INIT → SENSOR_WARMUP → CALIBRATE → CALIBRATE_MOVE → IDLE
  ↓
PLAN_PICK → APPROACH → ALIGN → DESCEND → GRASP → VERIFY_GRASP
  ↓                                          ↓ (失败)
  ↓                                     RETRY_OPEN → DESCEND (重试)
  ↓
LIFT → TRANSPORT → PLACE_DESCEND → RELEASE → RETREAT → RETURN_HOME → IDLE
  ↓
FINAL_HOME → STANDBY
```

**关键状态说明**

| 状态 | 功能 |
|---|---|
| SENSOR_WARMUP | 传感器预热（25 步） |
| CALIBRATE | 运动至 HOME 位姿，打开夹爪 |
| PLAN_PICK | 计算 approach + pick 关节角（IK 或 fallback） |
| ALIGN | 接近后对齐检查（距离传感器） |
| VERIFY_GRASP | 触觉 + 力反馈双重验证抓取是否成功 |
| RETRY_OPEN | 抓取失败后松开重试（最多 3 次） |
| TRANSPORT | 使用避障路径从 pick 侧运输到 place 侧 |

#### 传感器集成

| 传感器 | 用途 | 阈值 |
|---|---|---|
| DistanceSensor | 接近检测 | < 100.0 |
| TouchSensor | 接触检测 | > 0.1 |
| Camera | 亮度估算（抽样 80 像素） | — |
| Force Feedback | 夹爪力反馈 | > 5.0 N |

抓取验证使用 OR 逻辑：`touch > 阈值` **或** `gripper_force > 阈值` 即判定抓取成功。

#### 坐标变换

世界坐标 → 机器人本体坐标的转换考虑了：
- 基座位置偏移 `[0, 0, 0.8]`
- 基座绕 Z 轴旋转 `-π/2`

```python
dx, dy, dz = world - base_position
local_x = cos(-angle) * dx - sin(-angle) * dy
local_y = sin(-angle) * dx + cos(-angle) * dy
local_z = dz
```

#### 状态显示

每 60 步输出一次状态摘要，包含：当前 FSM 状态、目标进度、关节角、FK 位置、传感器读数、轨迹执行进度。

### 2.2 基础控制器（`ur5e_controller.py`）

较早版本的控制器，功能相对简化：
- **11 状态 FSM**（IDLE → DONE）
- **三次插值**（cubic ease-in-out）替代梯形速度
- **预定义关节角**：4 个目标的 pick/place 位姿直接硬编码
- **无外部配置文件**
- **无力反馈/触觉验证**
- **无避障路径**

作为入门参考或快速测试使用。

### 2.3 JSON 配置文件（`config.json`）

配置文件将所有可调参数外部化，方便修改而无需改代码：

| 配置段 | 参数项 |
|---|---|
| `robot` | 基座位置/朝向、速度/加速度上限、夹爪开合角度、力阈值、关节限位 |
| `task` | 5 个目标的世界坐标 + 类型/颜色、5 个放置位置、HOME 关节角、接近高度、抓取偏移 |
| `sensors` | 摄像头参数（320×240）、距离/触觉阈值 |
| `motion` | IK 尝试次数、抓取/释放/稳定等待步数 |

---

## 三、GaiaHand 开源灵巧手集成

### 3.1 项目概况

GaiaHand 是一款模块化的开源五指灵巧手，提供了完整的硬件、固件和软件栈。

| 项 | 说明 |
|---|---|
| 自由度 | 15 DOF（每指 3 DOF） |
| 驱动 | 无刷直流电机 + 减速器 |
| 通信 | Serial (UART) |
| 结构 | 3D 打印 (PLA+/PETG/ABS) + 金属件 |
| 软件 | Python SDK (`GaiahandSDK`) |
| 模型格式 | STEP (SolidWorks 兼容) |
| 许可证 | MIT |

### 3.2 与本项目的关联

- 提供了 **STL → DXF 转换脚本** (`convert_stl_to_dxf.py`) 和 **SolidWorks 导入脚本** (`import_gaiahand_*.scr`)
- 有完整的 **URDF** 描述（左/右手），可直接用于 ROS/Gazebo 仿真
- 提供了 **Python SDK** 用于电机控制和手指关节插值运动
- 硬件 STEP 文件可在 SolidWorks 中进行机械集成设计

### 3.3 SDK 核心能力

- 单电机/多电机位置控制
- 关节位置插值运动（线性插值）
- 手指预设姿态（如张开、握拳、各种抓取手势）
- 串口通信管理

---

## 四、webots_ros2 集成

项目中包含了 [webots_ros2](https://github.com/cyberbotics/webots_ros2) 官方包，提供了 Webots 与 ROS2 的桥接能力：

- **UR5e 专用包**（`webots_ros2_universal_robot`）
  - 包含 MoveIt2 配置（SRDF、运动学参数、控制器配置）
  - RViz 可视化配置
  - `follow_joint_trajectory_client` 接口
  - Robotiq 3-Finger Gripper 的 URDF/xacro
- **通用 Webots 驱动**（`webots_ros2_driver`）
- **控制接口**（`webots_ros2_control`）
- 支持的其他机器人：TurtleBot、Tesla、Mavic、e-puck、TIAGo 等

---

## 五、关键技术决策与设计权衡

### 5.1 IK 求解 — ikpy vs. 解析解

选择 ikpy（数值迭代法）而非 UR5e 的解析逆解：
- **优点**：实现简单、通用性强、可扩展到其他机械臂
- **代价**：求解速度较慢、可能收敛到非最优解
- **缓解**：多种子策略 + FK 验证 + fallback 机制

### 5.2 梯形速度 vs. 三次插值

完整控制器升级为梯形速度曲线（基础版使用三次插值）：
- 梯形速度更接近工业机器人的实际运动曲线
- 加速度可控，对物理仿真更友好
- 参数化（v_max, a_max）更直观

### 5.3 传感器融合策略

抓取验证使用 OR 逻辑（触觉 + 力反馈任一满足即可），而非 AND 逻辑：
- 仿真中单一传感器可能不稳定
- OR 策略更鲁棒，减少假阴性

### 5.4 配置外部化

从硬编码升级为 JSON 配置文件：
- 修改目标位置、运动参数无需改代码
- 便于多场景切换
- 支持运行时加载失败时自动 fallback 到默认值

---

## 六、依赖与环境

### 运行环境

| 依赖 | 版本/说明 |
|---|---|
| Webots | R2025a |
| Python | 3.x（Webots 内置） |
| ikpy | `pip install ikpy`（可选，无则使用 fallback） |
| NumPy | `pip install numpy` |

### 可选扩展

| 依赖 | 用途 |
|---|---|
| ROS2 (Humble/Iron) | webots_ros2 桥接 |
| MoveIt2 | 运动规划 |
| SolidWorks 2025 | GaiaHand 机械设计 |
| GaiahandSDK | 灵巧手控制 |

---

## 七、运行方式

### 7.1 仿真运行

1. 打开 Webots R2025a
2. 加载 `webots_ur5e_project/worlds/ur5e_complete.wbt`
3. 控制器自动运行 `ur5e_complete_controller`
4. 观察控制台输出状态信息

### 7.2 配置修改

编辑 `controllers/ur5e_complete_controller/config.json`：
- 添加/修改目标物体的 `world_position`
- 调整 `max_velocity`、`max_acceleration` 控制运动速度
- 修改 `gripper_closed` 值适配不同尺寸物体

---

## 八、已完成功能清单

- [x] Webots R2025a 完整工厂场景搭建
- [x] UR5e + Robotiq 3-Finger Gripper 集成
- [x] 基于 ikpy 的逆运动学求解（含多种子策略）
- [x] 正运动学验证
- [x] 梯形速度轨迹规划
- [x] 笛卡尔空间线性插值
- [x] 二次贝塞尔弧线路径
- [x] 避障路径规划
- [x] 18 状态有限状态机
- [x] 多目标（5 个不同形状/颜色物体）自动抓放循环
- [x] 摄像头、距离传感器、触觉传感器集成
- [x] 力反馈 + 触觉融合的抓取验证
- [x] 抓取失败重试机制（最多 3 次）
- [x] JSON 外部配置文件
- [x] 实时状态显示
- [x] 世界坐标 ↔ 机器人本体坐标变换
- [x] IK 不可用时的 fallback 关节角
- [x] GaiaHand 灵巧手 STEP/URDF 模型集成
- [x] SolidWorks 导入脚本
- [x] webots_ros2 官方包集成

---

## 九、AutoCAD 参数化建模脚本

项目根目录包含两个 AutoCAD `.scr` 脚本，用于程序化生成 3D 几何模型：

### `robot_arm_3d_v1.scr`
- 6-DOF 串联机械臂 + 平行夹爪
- 图层划分：`ARM_BASE`、`ARM_LINK`、`ARM_JOINT`、`ARM_GRIPPER`、`ARM_DIM`
- 底座 160×160×90 mm，6 段圆柱连杆 + 球关节
- 末端带夹爪（Box + Cone 组合）

### `humanoid_hand_3d.scr`
- 简化 5 指仿人手模型
- 掌部：80×90×25 mm 方块 + 圆柱腕部
- 4 指（食指–小指）各 3 段指节 + 拇指 2 段（带倾斜角）
- 14 个关节球体
- 自动标注尺寸

---

## 十、后续可扩展方向

- [ ] 接入 ROS2 + MoveIt2 实现外部运动规划
- [ ] 替换 ikpy 为 UR5e 解析逆解以提升性能
- [ ] 集成视觉识别（基于摄像头的物体检测与定位）
- [ ] 将 GaiaHand 集成为 UR5e 末端执行器
- [ ] 添加力/力矩控制（柔顺抓取）
- [ ] 多机器人协作场景
- [ ] 基于 ROS2 的远程监控 Dashboard

## 相关笔记

- [[SOLIDWORKS AI Overview - Obsidian]]
- [[SOLIDWORKS 2025 安装步骤（Obsidian）]]

## 参考资源

- [Webots R2025a 文档](https://cyberbotics.com/doc/guide/index)
- [UR5e 技术参数](https://www.universal-robots.com/products/ur5-robot/)
- [ikpy 文档](https://github.com/Phylliade/ikpy)
- [Robotiq 3-Finger Gripper](https://robotiq.com/products/3-finger-adaptive-robot-gripper)
- [GaiaHand GitHub](https://github.com/GaiaHand)
- [webots_ros2 GitHub](https://github.com/cyberbotics/webots_ros2)
