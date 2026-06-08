"""
AutoCAD Drawing Sync — Generates updated .scr scripts per experiment iteration.
=================================================================================
Each iteration modifies the robot arm / gripper geometry to reflect the algorithm
changes. Produces:
  - robot_arm_iter{N}.scr  — Updated 3D robot arm drawing
  - changelog entry         — Written back to Obsidian log

Usage:
    from autocad_sync import generate_iteration_drawing
    changes = generate_iteration_drawing(iteration=1, params={...})
"""

import datetime
import math
import os

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))

# ── Baseline UR5e DH dimensions (mm, matching robot_arm_3d_v1.scr) ──
BASE_WIDTH = 160
BASE_DEPTH = 160
BASE_HEIGHT = 90
JOINT_RADIUS = 12
LINK_RADIUS = 10

# UR5e link lengths (scaled to mm for drawing)
LINK_LENGTHS_BASE = [162.5, 425.0, 392.2, 133.3, 99.7, 99.6]


def _scr_header(layers):
    """Generate layer setup commands."""
    lines = ["_FILEDIA 0", "-UNITS 2 4 1 4 0 N", ""]
    for name, color in layers.items():
        lines.append(f"-LAYER M {name} C {color} {name} ")
    lines.append("")
    return lines


def _scr_footer(title_text="", annotation_lines=None):
    """Generate view setup and annotations."""
    lines = []
    if annotation_lines:
        lines.append("-LAYER S ARM_ANNOT ")
        for i, text in enumerate(annotation_lines):
            y = -20 - i * 12
            lines.append(f'_TEXT 0,{y},0 3.5 0 {text}')
        lines.append("")
    lines.append("_-VIEW _SWISO")
    lines.append("_ZOOM _E")
    lines.append("_VSCURRENT _REALISTIC")
    lines.append("")
    lines.append("_FILEDIA 1")
    lines.append("")
    return lines


def _draw_base(lines, width, depth, height):
    hw, hd = width / 2, depth / 2
    lines.append("-LAYER S ARM_BASE ")
    lines.append(f"_BOX {-hw},{-hd},0 {hw},{hd},{height}")
    lines.append("")


def _draw_joint(lines, x, y, z, r):
    lines.append("-LAYER S ARM_JOINT ")
    lines.append(f"_SPHERE {x},{y},{z} {r}")


def _draw_link(lines, x, y, z, r, length):
    lines.append("-LAYER S ARM_LINK ")
    lines.append(f"_CYLINDER {x},{y},{z} {r} {length}")


def _draw_gripper_parallel(lines, z_start, gap, width, thickness, length, tip_h=15):
    """Standard parallel-jaw gripper."""
    lines.append("-LAYER S ARM_GRIPPER ")
    hg = gap / 2
    hw = width / 2
    ht = thickness / 2
    pw = hg + hw
    lines.append(f"_BOX {-pw},{-ht},{z_start} {pw},{ht},{z_start + thickness}")
    lines.append(f"_BOX {-hg - hw},{-ht},{z_start + thickness} {-hg + hw},{ht},{z_start + thickness + length}")
    lines.append(f"_BOX {hg - hw},{-ht},{z_start + thickness} {hg + hw},{ht},{z_start + thickness + length}")
    tip_z = z_start + thickness + length
    lines.append(f"_CONE {-hg},0,{tip_z} {hw} _T {hw * 0.3:.1f} {tip_h}")
    lines.append(f"_CONE {hg},0,{tip_z} {hw} _T {hw * 0.3:.1f} {tip_h}")
    lines.append("")


def _draw_gripper_3finger(lines, z_start, radius, finger_len, tip_r):
    """Robotiq-style 3-finger adaptive gripper."""
    lines.append("-LAYER S ARM_GRIPPER ")
    lines.append(f"; Robotiq 3F Gripper (3-finger adaptive)")
    lines.append(f"_CYLINDER 0,0,{z_start} {radius + 5} 12")
    for angle_deg in [0, 120, 240]:
        rad = math.radians(angle_deg)
        fx = radius * math.cos(rad)
        fy = radius * math.sin(rad)
        fz = z_start + 12
        lines.append(f"-LAYER S ARM_GRIPPER ")
        lines.append(f"_CYLINDER {fx:.1f},{fy:.1f},{fz} 4 {finger_len}")
        lines.append(f"_SPHERE {fx:.1f},{fy:.1f},{fz + finger_len} {tip_r}")
    lines.append("")


def _draw_sensor(lines, sensor_type, x, y, z, size, label):
    """Draw a sensor marker and label."""
    lines.append("-LAYER S ARM_SENSOR ")
    if sensor_type == "camera":
        lines.append(f"_BOX {x - size},{y - size},{z} {x + size},{y + size},{z + size * 1.5}")
        lines.append(f"; Camera: {label}")
    elif sensor_type == "distance":
        lines.append(f"_CONE {x},{y},{z} {size} _T {size * 0.2:.1f} {size * 2}")
        lines.append(f"; DistanceSensor: {label}")
    elif sensor_type == "touch":
        lines.append(f"_SPHERE {x},{y},{z} {size * 0.8:.1f}")
        lines.append(f"; TouchSensor: {label}")
    elif sensor_type == "gps":
        lines.append(f"_SPHERE {x},{y},{z} {size * 0.6:.1f}")
        lines.append(f"; GPS: {label}")
    elif sensor_type == "force":
        hw = size * 0.5
        lines.append(f"_BOX {x - hw},{y - hw},{z} {x + hw},{y + hw},{z + hw}")
        lines.append(f"; ForceSensor: {label}")
    lines.append("")


def _draw_target_ball(lines, x, y, z, r, color_code=1):
    """Draw the target ball object."""
    lines.append(f"-LAYER S ARM_TARGET C {color_code} ARM_TARGET ")
    lines.append(f"_SPHERE {x},{y},{z} {r}")
    lines.append(f"; Target ball (r={r}mm)")
    lines.append("")


def _draw_connector(lines, x, y, z, r):
    """Draw connector mount on end-effector."""
    lines.append("-LAYER S ARM_CONNECTOR ")
    lines.append(f"_CYLINDER {x},{y},{z} {r} 5")
    lines.append(f"; Connector (magnetic lock)")
    lines.append("")


# =========================================================================
#  Per-iteration drawing generators
# =========================================================================

def _generate_iter1(output_path):
    """Iteration 1: Baseline — standard arm + parallel gripper + connector."""
    layers = {
        "ARM_BASE": 3, "ARM_LINK": 1, "ARM_JOINT": 5,
        "ARM_GRIPPER": 6, "ARM_SENSOR": 4, "ARM_CONNECTOR": 8,
        "ARM_TARGET": 1, "ARM_ANNOT": 2,
    }
    lines = _scr_header(layers)
    _draw_base(lines, BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)

    z = float(BASE_HEIGHT)
    links_mm = [140, 120, 100, 80, 60, 50]
    for length in links_mm:
        _draw_joint(lines, 0, 0, z, JOINT_RADIUS)
        _draw_link(lines, 0, 0, z, LINK_RADIUS, length)
        z += length

    _draw_gripper_parallel(lines, z, gap=30, width=8, thickness=8, length=40, tip_h=15)
    _draw_connector(lines, 0, 0, z + 8 + 40, 6)
    _draw_sensor(lines, "gps", 5, 0, z + 8 + 40 + 5, 4, "tool_gps")
    _draw_target_ball(lines, 200, 0, BASE_HEIGHT + 30, 30, color_code=1)

    annotations = [
        f"Iter 1: Baseline - GPS Calibration + Connector",
        f"Gripper: Parallel jaw (gap=30mm)",
        f"Sensors: GPS only",
        f"Date: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}",
    ]
    lines += _scr_footer(annotation_lines=annotations)

    with open(output_path, "w", encoding="ascii", errors="replace") as f:
        f.write("\n".join(lines))

    return {
        "file": output_path,
        "changes": [
            "基线图纸：6-DOF 机械臂 + 平行夹爪",
            "添加 Connector 磁吸接口 (末端 r=6mm 圆柱)",
            "添加 GPS 传感器标记 (末端)",
            "添加目标球体 (r=30mm, 红色) 在工作台位置",
        ],
    }


def _generate_iter2(output_path):
    """Iteration 2: Optimized — wider gripper + force sensor + distance sensor."""
    layers = {
        "ARM_BASE": 3, "ARM_LINK": 1, "ARM_JOINT": 5,
        "ARM_GRIPPER": 6, "ARM_SENSOR": 4, "ARM_CONNECTOR": 8,
        "ARM_TARGET": 1, "ARM_ANNOT": 2,
    }
    lines = _scr_header(layers)
    _draw_base(lines, BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)

    z = float(BASE_HEIGHT)
    links_mm = [140, 120, 100, 80, 60, 50]
    for length in links_mm:
        _draw_joint(lines, 0, 0, z, JOINT_RADIUS)
        _draw_link(lines, 0, 0, z, LINK_RADIUS, length)
        z += length

    _draw_gripper_parallel(lines, z, gap=36, width=10, thickness=8, length=45, tip_h=18)
    _draw_connector(lines, 0, 0, z + 8 + 45, 8)

    _draw_sensor(lines, "gps", 5, 0, z + 8 + 45 + 5, 4, "tool_gps")
    _draw_sensor(lines, "distance", 0, 12, z + 8 + 30, 5, "gripper_distance")
    _draw_sensor(lines, "force", 0, -12, z + 8 + 45, 5, "gripper_force")

    _draw_target_ball(lines, 200, 0, BASE_HEIGHT + 30, 30, color_code=1)

    annotations = [
        f"Iter 2: Optimized - Fine-grid IK + Force Feedback",
        f"Gripper: Wider parallel jaw (gap=36mm, len=45mm)",
        f"Sensors: GPS + DistanceSensor + ForceSensor",
        f"Connector: Enlarged (r=8mm) for better contact",
        f"Date: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}",
    ]
    lines += _scr_footer(annotation_lines=annotations)

    with open(output_path, "w", encoding="ascii", errors="replace") as f:
        f.write("\n".join(lines))

    return {
        "file": output_path,
        "changes": [
            "夹爪间距加宽 30mm → 36mm (适配球体)",
            "夹爪指长增加 40mm → 45mm",
            "Connector 半径增大 6mm → 8mm (提高接触率)",
            "新增 DistanceSensor 安装位 (夹爪前端 y=12mm)",
            "新增 ForceSensor 安装位 (夹爪底部 y=-12mm)",
            "指尖锥高增加 15mm → 18mm (更深入包裹)",
        ],
    }


def _generate_iter3(output_path):
    """Iteration 3: Visual Servo — camera mount + 3-finger gripper."""
    layers = {
        "ARM_BASE": 3, "ARM_LINK": 1, "ARM_JOINT": 5,
        "ARM_GRIPPER": 6, "ARM_SENSOR": 4, "ARM_CONNECTOR": 8,
        "ARM_TARGET": 1, "ARM_CAMERA": 30, "ARM_ANNOT": 2,
    }
    lines = _scr_header(layers)
    _draw_base(lines, BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)

    z = float(BASE_HEIGHT)
    links_mm = [140, 120, 100, 80, 60, 50]
    for length in links_mm:
        _draw_joint(lines, 0, 0, z, JOINT_RADIUS)
        _draw_link(lines, 0, 0, z, LINK_RADIUS, length)
        z += length

    _draw_gripper_3finger(lines, z, radius=18, finger_len=35, tip_r=5)
    _draw_connector(lines, 0, 0, z + 12 + 35, 8)

    _draw_sensor(lines, "gps", 5, 0, z + 12 + 35 + 5, 4, "tool_gps")
    _draw_sensor(lines, "camera", -20, 0, z - 30, 8, "arm_camera (320x240)")
    _draw_sensor(lines, "distance", 0, 20, z + 12 + 20, 5, "gripper_distance")
    _draw_sensor(lines, "touch", 18, 0, z + 12 + 35, 4, "gripper_touch")

    _draw_target_ball(lines, 200, 0, BASE_HEIGHT + 30, 30, color_code=1)

    annotations = [
        f"Iter 3: Visual Servo - Camera + PID Correction",
        f"Gripper: Robotiq 3F (3-finger adaptive, r=18mm)",
        f"Camera: 320x240 mounted on link5 (-20mm offset)",
        f"Sensors: GPS + Camera + Distance + Touch",
        f"Date: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}",
    ]
    lines += _scr_footer(annotation_lines=annotations)

    with open(output_path, "w", encoding="ascii", errors="replace") as f:
        f.write("\n".join(lines))

    return {
        "file": output_path,
        "changes": [
            "夹爪替换: 平行夹爪 → Robotiq 3F 三指自适应夹爪",
            "三指布局: 120° 均匀分布, 指长 35mm",
            "新增摄像头安装座 (link5 偏移 -20mm, 8x8x12mm)",
            "摄像头参数: 320x240, FOV 0.785 rad",
            "新增 TouchSensor 安装位 (finger_1 末端)",
            "夹爪底座改为圆柱形 (r=23mm, h=12mm)",
        ],
    }


def _generate_iter4(output_path):
    """Iteration 4: DL-Grasp — full sensor suite + RL observation annotations."""
    layers = {
        "ARM_BASE": 3, "ARM_LINK": 1, "ARM_JOINT": 5,
        "ARM_GRIPPER": 6, "ARM_SENSOR": 4, "ARM_CONNECTOR": 8,
        "ARM_TARGET": 1, "ARM_CAMERA": 30, "ARM_RL": 14, "ARM_ANNOT": 2,
    }
    lines = _scr_header(layers)
    _draw_base(lines, BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)

    z = float(BASE_HEIGHT)
    links_mm = [140, 120, 100, 80, 60, 50]
    for li, length in enumerate(links_mm):
        _draw_joint(lines, 0, 0, z, JOINT_RADIUS)
        _draw_link(lines, 0, 0, z, LINK_RADIUS, length)
        # RL observation markers on each joint
        lines.append("-LAYER S ARM_RL ")
        lines.append(f"; Joint {li+1} encoder (obs[{li}]: angle, obs[{li+6}]: velocity)")
        lines.append(f"_CIRCLE 0,0,{z} {JOINT_RADIUS + 3}")
        lines.append("")
        z += length

    _draw_gripper_3finger(lines, z, radius=18, finger_len=35, tip_r=5)
    _draw_connector(lines, 0, 0, z + 12 + 35, 8)

    _draw_sensor(lines, "gps", 5, 0, z + 12 + 35 + 5, 4, "tool_gps (obs[18:21])")
    _draw_sensor(lines, "camera", -20, 0, z - 30, 8, "arm_camera")
    _draw_sensor(lines, "distance", 0, 20, z + 12 + 20, 5, "gripper_distance")
    _draw_sensor(lines, "touch", 18, 0, z + 12 + 35, 4, "gripper_touch")
    _draw_sensor(lines, "force", -18, 0, z + 12 + 35, 4, "force_feedback")

    for i, angle_deg in enumerate([0, 120, 240]):
        rad = math.radians(angle_deg)
        fx = 18 * math.cos(rad)
        fy = 18 * math.sin(rad)
        lines.append("-LAYER S ARM_RL ")
        lines.append(f"; Finger {i+1} position sensor (obs[{12+i}])")
        lines.append(f"_CIRCLE {fx:.1f},{fy:.1f},{z + 12 + 17} 6")
        lines.append("")

    _draw_target_ball(lines, 200, 0, BASE_HEIGHT + 30, 30, color_code=1)

    lines.append("-LAYER S ARM_RL ")
    lines.append(f"; Ball position from Supervisor (obs[15:18])")
    lines.append(f"_CIRCLE 200,0,{BASE_HEIGHT + 30} 35")
    lines.append("")

    annotations = [
        f"Iter 4: SAC Deep RL - Full Sensor Suite",
        f"Algorithm: SAC (Soft Actor-Critic), MLP[256,256]",
        f"Obs: 21-D (joints6+vel6+fingers3+ball3+ee3)",
        f"Act: 7-D (6 joint deltas + 1 gripper)",
        f"Sensors: GPS+Camera+Dist+Touch+Force+Encoders",
        f"RL obs markers shown as cyan circles",
        f"Date: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}",
    ]
    lines += _scr_footer(annotation_lines=annotations)

    with open(output_path, "w", encoding="ascii", errors="replace") as f:
        f.write("\n".join(lines))

    return {
        "file": output_path,
        "changes": [
            "新增 RL 图层 (ARM_RL, cyan) 标注观测空间映射",
            "6 个关节编码器标记 (obs[0:6] 角度, obs[6:12] 角速度)",
            "3 个指位传感器标记 (obs[12:15])",
            "球体位置观测标记 (obs[15:18], Supervisor读取)",
            "末端位置观测标记 (obs[18:21], GPS)",
            "新增 ForceSensor 安装位 (finger_1 对侧)",
            "标注 SAC 动作空间: 7-D (6关节增量+1夹爪)",
        ],
    }


def _generate_iter5(output_path):
    """Iteration 5: DL-Enhanced — CNN viewport + curriculum annotations."""
    layers = {
        "ARM_BASE": 3, "ARM_LINK": 1, "ARM_JOINT": 5,
        "ARM_GRIPPER": 6, "ARM_SENSOR": 4, "ARM_CONNECTOR": 8,
        "ARM_TARGET": 1, "ARM_CAMERA": 30, "ARM_RL": 14,
        "ARM_CNN": 40, "ARM_ANNOT": 2,
    }
    lines = _scr_header(layers)
    _draw_base(lines, BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)

    z = float(BASE_HEIGHT)
    links_mm = [140, 120, 100, 80, 60, 50]
    for li, length in enumerate(links_mm):
        _draw_joint(lines, 0, 0, z, JOINT_RADIUS)
        _draw_link(lines, 0, 0, z, LINK_RADIUS, length)
        lines.append("-LAYER S ARM_RL ")
        lines.append(f"_CIRCLE 0,0,{z} {JOINT_RADIUS + 3}")
        lines.append("")
        z += length

    _draw_gripper_3finger(lines, z, radius=20, finger_len=38, tip_r=5.5)
    _draw_connector(lines, 0, 0, z + 12 + 38, 10)

    _draw_sensor(lines, "gps", 5, 0, z + 12 + 38 + 5, 4, "tool_gps")
    _draw_sensor(lines, "camera", -22, 0, z - 25, 10, "arm_camera_hires (640x480)")
    _draw_sensor(lines, "distance", 0, 22, z + 12 + 22, 5, "gripper_distance")
    _draw_sensor(lines, "touch", 20, 0, z + 12 + 38, 4, "gripper_touch")
    _draw_sensor(lines, "force", -20, 0, z + 12 + 38, 4, "force_feedback")

    lines.append("-LAYER S ARM_CNN ")
    lines.append(f"; CNN Feature Extraction Viewport")
    cam_z = z - 25
    fov_half = 22.5
    lines.append(f"_LINE -22,0,{cam_z} {-22 + 80 * math.cos(math.radians(fov_half)):.1f},0,{cam_z - 80 * math.sin(math.radians(fov_half)):.1f}")
    lines.append(f"_LINE -22,0,{cam_z} {-22 + 80 * math.cos(math.radians(-fov_half)):.1f},0,{cam_z + 80 * math.sin(math.radians(-fov_half)):.1f}")
    lines.append(f"; CNN input: 640x480 RGB → Conv layers → 64-D embedding")
    lines.append("")

    lines.append("-LAYER S ARM_CNN ")
    lines.append(f"; Curriculum learning: ball randomization zones")
    for r_mm, label in [(20, "Phase1 r=2cm"), (50, "Phase2 r=5cm"), (80, "Phase3 r=8cm")]:
        lines.append(f"_CIRCLE 200,0,{BASE_HEIGHT + 30} {r_mm}")
        lines.append(f"; {label}")
    lines.append("")

    _draw_target_ball(lines, 200, 0, BASE_HEIGHT + 30, 30, color_code=1)

    annotations = [
        f"Iter 5: CNN + SAC + Curriculum Learning",
        f"Camera: Upgraded 640x480 (10x10mm mount)",
        f"CNN: Conv2D feature extraction → 64-D embedding",
        f"FOV visualization: camera field of view cone",
        f"Curriculum: 3 phases (r=2/5/8 cm randomization)",
        f"Gripper: Enlarged 3F (r=20mm, finger=38mm)",
        f"Connector: Max size (r=10mm)",
        f"Date: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}",
    ]
    lines += _scr_footer(annotation_lines=annotations)

    with open(output_path, "w", encoding="ascii", errors="replace") as f:
        f.write("\n".join(lines))

    return {
        "file": output_path,
        "changes": [
            "摄像头升级: 320x240 → 640x480 高分辨率",
            "摄像头安装座增大: 8x8 → 10x10mm",
            "新增 CNN 图层 (ARM_CNN) 标注视觉处理流水线",
            "绘制摄像头 FOV 锥形视野范围",
            "标注 CNN 特征提取流程: RGB→Conv→64-D嵌入",
            "三指夹爪增大: r=18→20mm, 指长35→38mm",
            "Connector 增大: r=8→10mm (最大尺寸)",
            "新增课程学习随机化区域标注 (3个同心圆)",
        ],
    }


# =========================================================================
#  Public API
# =========================================================================

_GENERATORS = {
    1: _generate_iter1,
    2: _generate_iter2,
    3: _generate_iter3,
    4: _generate_iter4,
    5: _generate_iter5,
}


def generate_iteration_drawing(iteration, output_dir=None):
    """
    Generate the AutoCAD .scr drawing for a given iteration.

    Returns dict with keys:
        "file"    — absolute path to the generated .scr
        "changes" — list[str] of human-readable change descriptions
    """
    if output_dir is None:
        output_dir = OUTPUT_DIR

    gen_fn = _GENERATORS.get(iteration)
    if gen_fn is None:
        return {"file": None, "changes": [f"迭代 {iteration} 无对应图纸生成器"]}

    filename = f"robot_arm_iter{iteration}.scr"
    output_path = os.path.join(output_dir, filename)
    result = gen_fn(output_path)

    size = os.path.getsize(output_path)
    print(f"  [AutoCAD] Generated {filename} ({size:,} bytes)")
    return result


def get_all_changes_summary():
    """Return a dict mapping iteration → change list (without generating files)."""
    return {
        1: [
            "基线图纸：6-DOF 机械臂 + 平行夹爪",
            "Connector 磁吸接口, GPS 传感器标记",
        ],
        2: [
            "夹爪加宽 (30→36mm), Connector 增大 (r=6→8mm)",
            "新增 DistanceSensor, ForceSensor 安装位",
        ],
        3: [
            "夹爪替换为 Robotiq 3F 三指自适应",
            "新增摄像头安装座, TouchSensor 安装位",
        ],
        4: [
            "新增 RL 观测空间标注 (ARM_RL 图层)",
            "关节编码器、指位传感器、力反馈标记",
        ],
        5: [
            "摄像头升级 640x480, CNN 视野锥标注",
            "课程学习随机化区域 (3同心圆), 夹爪增大",
        ],
    }


if __name__ == "__main__":
    print("=" * 60)
    print("  AutoCAD Drawing Sync — Batch Generate All Iterations")
    print("=" * 60)
    for i in range(1, 6):
        result = generate_iteration_drawing(i)
        print(f"\n  Iteration {i}: {result['file']}")
        for c in result["changes"]:
            print(f"    - {c}")
    print("\n" + "=" * 60)
    print("  Done.")
