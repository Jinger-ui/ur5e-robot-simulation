# UR5e 机器人仿真 & GaiaHand 灵巧手 | Webots + AutoCAD DXF 参数化建模

> 基于 Webots R2025a 的 UR5e 工业机器人臂 Pick-and-Place 仿真系统，集成 GaiaHand 开源灵巧手仿真与 AutoCAD 参数化 3D 建模脚本。

---

## 项目简介

本项目是一个综合性的机器人仿真与建模平台，包含：

- **UR5e 工业机器人臂**的完整 Pick-and-Place（抓取-放置）仿真，含 18 状态有限状态机、逆运动学求解、梯形速度轨迹规划
- **GaiaHand 15-DOF 五指灵巧手**在 Webots 中的仿真建模（基于开源 STL 模型转换）
- **AutoCAD 参数化建模脚本**：Python 自动生成 6-DOF 机械臂和仿人五指手的 DXF/SCR 文件

---

## 功能列表

- [x] 2D/3D 机械臂参数化建模（Python → DXF + AutoCAD SCR）
- [x] 仿人五指灵巧手参数化建模与仿真
- [x] UR5e 工业机械臂 Pick-and-Place 仿真（5 个目标物体）
- [x] 完整工厂场景（传送带、货架、信号灯、安全围栏等）
- [x] 基于 ikpy 的逆运动学求解（多种子策略 + FK 验证）
- [x] 梯形速度轨迹规划 + 笛卡尔线性插值 + 贝塞尔弧线路径
- [x] GPS 自动校准 + Connector 可靠抓取方案
- [x] 力反馈 + 触觉传感器融合的抓取验证
- [x] 抓取失败自动重试机制（最多 3 次）
- [x] 多目标物体顺序抓放循环
- [x] JSON 外部配置文件（无需改代码即可调参）
- [x] 实时状态显示（FSM 状态、关节角、传感器读数）

---

## 应用的 GitHub 开源项目

| 项目 | 说明 | 用途 |
|------|------|------|
| [cyberbotics/webots](https://github.com/cyberbotics/webots) | Webots 开源机器人仿真平台 | 核心仿真引擎 |
| [cyberbotics/webots_ros2](https://github.com/cyberbotics/webots_ros2) | Webots ROS2 接口包 | 参考架构与 UR5e MoveIt2 配置 |
| [Stella-robot/GaiaHand](https://github.com/Stella-robot/GaiaHand) | 15-DOF 开源五指灵巧手 | STL 模型来源 & URDF 转换 |
| [mozman/ezdxf](https://github.com/mozman/ezdxf) | Python DXF 读写库 | DXF 文件生成 |
| [phylliade/ikpy](https://github.com/phylliade/ikpy) | Python 逆运动学库 | UR5e 关节角求解 |
| [cyberbotics/urdf2webots](https://github.com/cyberbotics/urdf2webots) | URDF → Webots PROTO 转换 | GaiaHand URDF 转 Webots 模型 |

---

## 技术栈

### 仿真与控制

| 技术 | 版本/说明 |
|------|-----------|
| **Webots** | R2025a — 机器人仿真引擎（物理引擎 + 3D 渲染） |
| **Python** | 3.11 — 控制器语言 & 脚本 |
| **ikpy** | 逆运动学数值求解（DH 参数链） |
| **NumPy** | 矩阵运算、轨迹插值 |
| **Universal Robots UR5e** | 6-DOF 工业协作机器人模型 |
| **Robotiq 3F Gripper** | 三指自适应工业夹爪 |
| **Connector 节点** | Webots 磁吸式抓取方案 |
| **GPS 传感器** | 末端执行器姿态校准 |

### CAD 建模

| 技术 | 版本/说明 |
|------|-----------|
| **AutoCAD** | 2027 — CAD 查看与运行 SCR 脚本 |
| **ezdxf** | Python DXF 文件程序化生成 |
| **trimesh** | 3D 网格处理（STL 分析） |
| **numpy-stl** | STL 文件读写 |

### 灵巧手

| 技术 | 说明 |
|------|------|
| **GaiaHand** | 开源 15-DOF 灵巧手设计（Stella-robot） |
| **urdf2webots** | URDF 模型转 Webots PROTO |

### 参考架构（未在 Windows 部署）

| 技术 | 说明 |
|------|------|
| **ROS2 Humble** | 机器人操作系统（参考架构） |
| **MoveIt2** | 运动规划框架（参考配置） |

### 工具链

| 工具 | 用途 |
|------|------|
| **Git / GitHub** | 版本控制与代码托管 |
| **VS Code / Cursor** | 代码编辑器 |

---

## 项目结构

```
solidworks/
├── README.md                          # 本文件
├── .gitignore                         # Git 忽略规则
│
├── webots_ur5e_project/               # ★ 核心：UR5e 仿真项目
│   ├── worlds/
│   │   ├── ur5e_complete.wbt          # 完整工厂场景（主场景）
│   │   ├── ur5e_demo.wbt             # 简化演示场景
│   │   └── ur5e_pick.wbt            # 抓取测试场景
│   └── controllers/
│       ├── ur5e_complete_controller/
│       │   ├── ur5e_complete_controller.py  # 18 状态 FSM 控制器
│       │   └── config.json                  # 外部配置文件
│       ├── ur5e_controller/
│       │   └── ur5e_controller.py           # 基础控制器
│       ├── ur5e_pick/
│       │   └── ur5e_pick.py                 # 抓取专用控制器
│       └── ur5e_test/
│           └── ur5e_test.py                 # 测试控制器
│
├── webots_gaiahand_project/           # ★ GaiaHand 灵巧手仿真
│   ├── worlds/
│   │   ├── gaiahand.wbt              # 完整灵巧手场景
│   │   └── gaiahand_simple.wbt       # 简化场景
│   ├── controllers/
│   │   └── gaiahand_controller/
│   │       └── gaiahand_controller.py
│   ├── protos/                        # Webots PROTO 模型 + STL
│   ├── meshes/                        # 左/右手 STL 网格文件
│   └── analyze_stl.py                 # STL 分析脚本
│
├── robot_arm_generator.py             # 2D 机械臂 DXF 生成器
├── robot_arm_3d_generator.py          # 3D 机械臂 DXF 生成器
├── generate_humanoid_hand.py          # 仿人手 DXF 生成器
├── robot_arm_v1.dxf                   # 生成的 2D 机械臂模型
├── robot_arm_3d_v1.dxf                # 生成的 3D 机械臂模型
├── robot_arm_3d_v1.scr                # AutoCAD 脚本：机械臂
├── humanoid_hand_3d.dxf               # 生成的仿人手模型
└── humanoid_hand_3d.scr               # AutoCAD 脚本：仿人手
```

---

## 快速开始

### 环境要求

- **Webots R2025a**（[下载](https://cyberbotics.com/doc/guide/installation-procedure)）
- **Python 3.11+**（Webots 内置或系统安装）
- **AutoCAD 2024+**（可选，用于查看 .scr/.dxf 文件）

### 安装依赖

```bash
pip install ikpy numpy ezdxf trimesh numpy-stl
```

### 运行 UR5e 仿真

1. 打开 **Webots R2025a**
2. 菜单 `File → Open World...`
3. 选择 `webots_ur5e_project/worlds/ur5e_complete.wbt`
4. 点击播放按钮，控制器自动运行
5. 观察控制台输出（FSM 状态、传感器数据、抓取结果）

### 运行 GaiaHand 仿真

1. 在 Webots 中打开 `webots_gaiahand_project/worlds/gaiahand.wbt`
2. 观察灵巧手关节运动

### 生成 DXF 模型

```bash
# 生成 3D 机械臂 DXF + AutoCAD 脚本
python robot_arm_3d_generator.py

# 生成仿人手 DXF + AutoCAD 脚本
python generate_humanoid_hand.py
```

生成的 `.scr` 文件可在 AutoCAD 中通过 `SCRIPT` 命令运行，自动创建 3D 模型。

---

## 控制器架构

### 核心控制器 — 18 状态有限状态机

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

### 关键技术特性

- **多种子 IK 求解**：最多 5 次尝试，种子来源包括用户提供、零位、当前关节位置、随机采样
- **梯形速度轨迹**：v_max = 1.5 rad/s，a_max = 2.0 rad/s²
- **笛卡尔线性插值**：均匀采样 → 逐点 IK → 梯形拼接
- **传感器融合**：TouchSensor + Force Feedback 的 OR 逻辑抓取验证
- **自动 Fallback**：ikpy 不可用时回退到预定义关节角

---

## 许可证

本项目代码采用 **MIT License** 开源。

第三方资源的许可证请参照各自仓库：
- Webots: Apache 2.0
- GaiaHand: MIT
- ikpy: MIT
- ezdxf: MIT

---

## 致谢

感谢以下开源项目为本仿真系统提供了基础支持：

- **[Cyberbotics / Webots](https://github.com/cyberbotics/webots)** — 强大的开源机器人仿真平台
- **[Cyberbotics / webots_ros2](https://github.com/cyberbotics/webots_ros2)** — Webots 与 ROS2 的桥接接口，提供了 UR5e + MoveIt2 配置参考
- **[Stella-robot / GaiaHand](https://github.com/Stella-robot/GaiaHand)** — 开源 15-DOF 模块化五指灵巧手，提供了完整的 STL/URDF 模型
- **[mozman / ezdxf](https://github.com/mozman/ezdxf)** — Python DXF 文件创建与操作库
- **[Phylliade / ikpy](https://github.com/phylliade/ikpy)** — Python 逆运动学求解库
- **[Cyberbotics / urdf2webots](https://github.com/cyberbotics/urdf2webots)** — URDF 到 Webots PROTO 格式转换工具
- **[Universal Robots](https://www.universal-robots.com/)** — UR5e 协作机器人设计与技术文档
- **[Robotiq](https://robotiq.com/)** — 3-Finger Adaptive Gripper 设计

---

## 联系方式

如有问题或建议，欢迎通过 GitHub Issues 提交。
