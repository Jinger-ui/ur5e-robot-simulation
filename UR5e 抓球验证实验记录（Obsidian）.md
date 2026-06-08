---
title: UR5e 抓球验证实验记录
aliases:
  - UR5e 抓球实验
  - Ball Grasp Experiment Log
tags:
  - experiment
  - ur5e
  - ball-grasp
  - deep-learning
  - reinforcement-learning
  - iteration
  - webots
created: 2026-06-08
status: 进行中
---

# UR5e 抓球验证实验记录

## 实验目标

在 Webots R2025a 仿真环境中，使 UR5e 机械臂搭配 Robotiq 3F 夹爪 **成功抓起红色小球**（sphere r=0.03m），并放置到容器中。

**关键约束**：最终方案必须包含 **深度学习** 组件。若迭代结束时未使用深度学习且抓取失败，自动触发下一轮迭代升级算法。

## 实验环境

| 参数 | 值 |
|---|---|
| 仿真平台 | Webots R2025a |
| 机器人 | UR5e 6-DOF |
| 末端执行器 | Robotiq 3-Finger Gripper + Connector |
| 目标物体 | 红色球体 (r=0.03m, 0.12kg) |
| 物理引擎 | 内置 (摩擦系数=10, bounce=0.01) |
| 仿真步长 | 32ms |
| 操作系统 | Windows 10 |

## 迭代策略总览

```
迭代 1 (Baseline)    → GPS校准 + 预计算关节角 + Connector磁吸
迭代 2 (Optimized)   → 多种子IK求解 + 自适应力反馈 + 梯形速度轨迹
迭代 3 (Visual)      → 摄像头视觉伺服 + PID位置修正 + 闭环控制
迭代 4 (DL-Grasp)    → SAC深度强化学习策略网络 + 视觉特征提取
迭代 5 (DL-Enhanced) → CNN目标检测 + RL精细操控 + 课程学习
```

**自动迭代规则**：
- 每轮结束后检测 `ball_z > table_height + 0.05` 判定抓取成功
- 成功 + 使用了深度学习 → 实验完成
- 成功 + 未使用深度学习 → 记录结果，继续升级到包含DL的方案
- 失败 → 自动进入下一迭代

---

## 迭代 1：Baseline — GPS校准 + Connector磁吸

### 算法描述

| 组件 | 算法/方法 | 说明 |
|---|---|---|
| 定位 | GPS校准扫描 | 遍历 ~800 个候选关节角，GPS测距找最近 |
| 运动学 | 预计算关节角 | 固定 DH 参数计算，无实时 IK |
| 轨迹 | 分段关节插值 | 两阶段运动（先腕关节，后全身） |
| 抓取 | Connector磁吸 | Webots Connector 物理锁定 |
| 验证 | ball_z 高度检测 | Supervisor 读取球体世界坐标 |

### 核心代码逻辑

```python
# 校准：扫描 ~800 个姿态找最佳抓取角
for pose in generate_calibration_poses():
    set_joints(pose)
    gps = gps_pos()
    d = dist3(gps, BALL_TARGET["pos"])
    if d < best["dist"]:
        best = {"pose": pose, "dist": d, "gps": gps}

# 抓取：Connector 锁定
connector.lock()
presence = connector.getPresence()
```

### 深度学习使用：❌ 无

### 实验结果

> **状态**：⏳ 待运行
> 
> | 指标 | 值 |
> |---|---|
> | 校准距离 | — |
> | Connector 锁定 | — |
> | 球体抬升 | — |
> | 放入容器 | — |
> | 总体结果 | — |

### 迭代决策

- [ ] 若成功 → 记录但需升级到DL方案
- [ ] 若失败 → 进入迭代 2

---

## 迭代 2：Optimized — 多种子IK + 自适应力反馈

### 算法描述

| 组件 | 算法/方法 | 说明 |
|---|---|---|
| 定位 | GPS校准 + 精细微调 | 在最佳解附近做 ±0.05rad 网格搜索 |
| 运动学 | ikpy 数值 IK | L-BFGS-B 优化，5 种子策略 |
| 轨迹 | 梯形速度曲线 | v_max=1.2 rad/s, a_max=2.0 rad/s² |
| 抓取 | Connector + 力反馈 | 力阈值 > 5N 确认接触 |
| 验证 | 触觉 OR 力反馈融合 | 双传感器 OR 逻辑 |

### 关键改进

1. **多种子 IK 求解**：用户seed → 零位 → 当前位置 → 随机采样
2. **FK 验证**：反算误差 < 0.05m 才接受
3. **精细网格搜索**：在校准最佳解周围 ±0.05 rad 步长 0.02 rad

### 深度学习使用：❌ 无

### 实验结果

> **状态**：⏳ 待运行

### 迭代决策

- [ ] 若成功 → 记录但需升级到DL方案
- [ ] 若失败 → 进入迭代 3

---

## 迭代 3：Visual — 摄像头视觉伺服

### 算法描述

| 组件 | 算法/方法 | 说明 |
|---|---|---|
| 感知 | HSV 颜色分割 | 检测红色球体中心像素坐标 |
| 控制 | PID 视觉伺服 | 图像误差 → 关节增量修正 |
| 运动学 | 雅可比矩阵 | 将笛卡尔速度映射到关节速度 |
| 轨迹 | 笛卡尔空间线性插值 | 起点→终点均匀采样 |
| 抓取 | Connector + 视觉确认 | 图像中球体面积判断接近度 |

### 关键改进

1. **闭环视觉控制**：不再依赖开环 GPS 定位
2. **PID 伺服**：实时修正末端位置误差
3. **颜色分割**：OpenCV HSV 空间检测红色球体

### 深度学习使用：❌ 无（传统 CV）

### 实验结果

> **状态**：⏳ 待运行

### 迭代决策

- [ ] 若成功 → 记录但需升级到DL方案
- [ ] 若失败 → 进入迭代 4

---

## 迭代 4：DL-Grasp — SAC 深度强化学习

### 算法描述

| 组件 | 算法/方法 | 说明 |
|---|---|---|
| 算法 | SAC (Soft Actor-Critic) | Off-policy, 连续动作空间 |
| 策略网络 | MLP [256, 256] | 全连接，ReLU 激活 |
| 观测空间 | 21-D 向量 | 关节角(6)+角速度(6)+指位(3)+球位(3)+末端位(3) |
| 动作空间 | 7-D 连续 | 6关节增量 + 1夹爪开合 |
| 奖励函数 | 距离塑形 + 接触奖励 + 提升奖励 | 多层次引导信号 |
| 训练 | 50K~200K timesteps | 自动检查点保存 |

### 奖励函数设计

```python
r = 0.0
r -= dist * 2.0                    # 距离惩罚
r += (prev_dist - dist) * 30.0     # 接近奖励
if dist < 0.15: r += 2.0           # 近距离奖励
if dist < 0.08: r += 5.0           # 极近奖励
if dist < 0.04: r += 10.0          # 接触奖励
if connector.getPresence(): r += 20.0  # 物理连接奖励
if height_above > 0.03: r += height * 200.0  # 提升奖励
if height_above > 0.20: r += 1000.0  # 成功大奖励
r -= 0.05                           # 时间惩罚
r -= 0.01 * action_magnitude        # 动作平滑惩罚
```

### 超参数

| 参数 | 值 |
|---|---|
| learning_rate | 3e-4 |
| batch_size | 256 |
| buffer_size | 100,000 |
| gamma | 0.99 |
| tau | 0.005 |
| learning_starts | 1,000 |

### 深度学习使用：✅ SAC (深度强化学习)

### 实验结果

> **状态**：⏳ 待运行

### 迭代决策

- [ ] 若成功 → ✅ 实验完成！
- [ ] 若失败 → 进入迭代 5 (增强训练)

---

## 迭代 5：DL-Enhanced — CNN检测 + RL精细操控

### 算法描述

| 组件 | 算法/方法 | 说明 |
|---|---|---|
| 视觉 | CNN 目标检测 | 从摄像头图像定位球体 |
| 策略 | SAC + 视觉嵌入 | 图像特征拼接到观测向量 |
| 训练策略 | 课程学习 | 逐步增大随机化范围 |
| 奖励 | HER (Hindsight) | 事后经验回放提升样本效率 |

### 深度学习使用：✅ CNN + SAC (深度学习)

### 实验结果

> **状态**：⏳ 待运行

---

## 实验总结

| 迭代 | 方法 | 深度学习 | 抓取成功 | 用时 |
|---|---|---|---|---|
| 1 | GPS校准+Connector | ❌ | — | — |
| 2 | 多种子IK+力反馈 | ❌ | — | — |
| 3 | 视觉伺服+PID | ❌ | — | — |
| 4 | SAC深度RL | ✅ | — | — |
| 5 | CNN+SAC+课程学习 | ✅ | — | — |

## AutoCAD 图纸变更总览

每次实验迭代同步更新 AutoCAD 图纸（`.scr` 脚本），反映机械结构和传感器配置的改动。

| 迭代 | 图纸文件 | 主要变更 |
|---|---|---|
| 1 | `robot_arm_iter1.scr` | 基线：6-DOF臂 + 平行夹爪 + Connector + GPS |
| 2 | `robot_arm_iter2.scr` | 夹爪加宽(36mm) + Connector增大 + 距离/力传感器 |
| 3 | `robot_arm_iter3.scr` | Robotiq 3F三指夹爪 + 摄像头安装座 + TouchSensor |
| 4 | `robot_arm_iter4.scr` | RL观测标注层 + 编码器标记 + 全传感器套件 |
| 5 | `robot_arm_iter5.scr` | 高分摄像头 + CNN视野锥 + 课程学习区域标注 |

### 图纸图层说明

| 图层 | 颜色 | 用途 |
|---|---|---|
| ARM_BASE | 绿色(3) | 机械臂底座 |
| ARM_LINK | 红色(1) | 连杆 |
| ARM_JOINT | 蓝色(5) | 关节 |
| ARM_GRIPPER | 品红(6) | 夹爪 |
| ARM_SENSOR | 黄色(4) | 传感器安装位 |
| ARM_CONNECTOR | 灰色(8) | Connector 磁吸接口 |
| ARM_TARGET | 红色(1) | 目标球体 |
| ARM_CAMERA | 橙色(30) | 摄像头安装座 |
| ARM_RL | 青色(14) | RL 观测空间标注 |
| ARM_CNN | 紫色(40) | CNN 视觉处理标注 |
| ARM_ANNOT | 黄色(2) | 注释文字 |

### 逐迭代详细变更记录

> *以下内容由实验运行器自动填充*

---

## 关键发现

> *实验完成后总结*

## 相关笔记

- [[UR5e Webots 仿真项目技术文档（Obsidian）]]
- [[SOLIDWORKS AI Overview - Obsidian]]

## 参考文献

- Haarnoja et al., "Soft Actor-Critic: Off-Policy Maximum Entropy Deep RL" (2018)
- Levine et al., "Learning Hand-Eye Coordination for Robotic Grasping" (2018)
- Pinto & Gupta, "Supersizing Self-supervision: Learning Grasp from 50K Tries" (2016)
- Kalashnikov et al., "QT-Opt: Scalable Deep RL for Vision-Based Robotic Manipulation" (2018)
