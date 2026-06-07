"""
3D Robot Arm Generator
Generates a 3D robot arm model using two approaches:
  1. DXF file with 3DFACE mesh entities (via ezdxf)
  2. AutoCAD Script (.scr) file with native 3D solid commands
"""

import ezdxf
from ezdxf.render import forms
import os
import math

# ─── Parameters (same as 2D version) ───
JOINT_COUNT = 5
LINK_LENGTHS = [140, 120, 100, 80, 60]
BASE_WIDTH = 160
BASE_DEPTH = 160
BASE_HEIGHT = 90
JOINT_RADIUS = 12
LINK_RADIUS = 10  # ~20mm diameter

GRIPPER_LENGTH = 40
GRIPPER_WIDTH = 8
GRIPPER_THICKNESS = 8
GRIPPER_GAP = 30

MESH_SEGMENTS = 24
SPHERE_STACKS = 12

OUTPUT_DIR = r"c:\Users\Cleveland\Desktop\solidworks"
DXF_FILE = os.path.join(OUTPUT_DIR, "robot_arm_3d_v1.dxf")
SCR_FILE = os.path.join(OUTPUT_DIR, "robot_arm_3d_v1.scr")

LAYERS = {
    "ARM_BASE":    3,   # Green
    "ARM_LINK":    1,   # Red
    "ARM_JOINT":   5,   # Blue
    "ARM_GRIPPER": 6,   # Magenta
    "ARM_DIM":     2,   # Yellow
}


def generate_dxf():
    """Generate DXF with 3DFACE mesh entities via ezdxf."""
    doc = ezdxf.new("R2010")
    msp = doc.modelspace()

    for name, color in LAYERS.items():
        doc.layers.add(name, color=color)

    # ── Base: scaled unit cube ──
    base = forms.cube(center=True)
    base.scale(BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)
    base.translate(0, 0, BASE_HEIGHT / 2)
    base.render_3dfaces(msp, dxfattribs={"layer": "ARM_BASE"})

    # ── Arm segments ──
    z = float(BASE_HEIGHT)
    for i in range(JOINT_COUNT):
        length = LINK_LENGTHS[i]

        joint = forms.sphere(count=MESH_SEGMENTS, stacks=SPHERE_STACKS,
                             radius=JOINT_RADIUS)
        joint.translate(0, 0, z)
        joint.render_3dfaces(msp, dxfattribs={"layer": "ARM_JOINT"})

        link = forms.cylinder(count=MESH_SEGMENTS, radius=LINK_RADIUS,
                              top_radius=LINK_RADIUS,
                              top_center=(0, 0, length), caps=True)
        link.translate(0, 0, z)
        link.render_3dfaces(msp, dxfattribs={"layer": "ARM_LINK"})

        z += length

    # ── End-effector: simple gripper ──
    half_gap = GRIPPER_GAP / 2
    hw = GRIPPER_WIDTH / 2
    ht = GRIPPER_THICKNESS / 2

    # Base plate
    plate = forms.cube(center=True)
    plate.scale(GRIPPER_GAP + GRIPPER_WIDTH * 2, GRIPPER_THICKNESS, GRIPPER_THICKNESS)
    plate.translate(0, 0, z + ht)
    plate.render_3dfaces(msp, dxfattribs={"layer": "ARM_GRIPPER"})

    # Left prong
    lp = forms.cube(center=True)
    lp.scale(GRIPPER_WIDTH, GRIPPER_THICKNESS, GRIPPER_LENGTH)
    lp.translate(-half_gap, 0, z + GRIPPER_THICKNESS + GRIPPER_LENGTH / 2)
    lp.render_3dfaces(msp, dxfattribs={"layer": "ARM_GRIPPER"})

    # Right prong
    rp = forms.cube(center=True)
    rp.scale(GRIPPER_WIDTH, GRIPPER_THICKNESS, GRIPPER_LENGTH)
    rp.translate(half_gap, 0, z + GRIPPER_THICKNESS + GRIPPER_LENGTH / 2)
    rp.render_3dfaces(msp, dxfattribs={"layer": "ARM_GRIPPER"})

    # Finger tips (tapered)
    tip_h = 15
    for sign in (-1, 1):
        tip = forms.cylinder(count=MESH_SEGMENTS, radius=hw,
                             top_radius=hw * 0.3,
                             top_center=(0, 0, tip_h), caps=True)
        tip.translate(sign * half_gap, 0, z + GRIPPER_THICKNESS + GRIPPER_LENGTH)
        tip.render_3dfaces(msp, dxfattribs={"layer": "ARM_GRIPPER"})

    # Set isometric view via VPORT table if available
    try:
        vport = doc.viewports.get("*Active")
        if vport:
            vport[0].dxf.view_direction_vector = (1, -1, 1)
    except Exception:
        pass

    doc.saveas(DXF_FILE)
    size = os.path.getsize(DXF_FILE)
    print(f"  DXF saved : {DXF_FILE}")
    print(f"  File size : {size:,} bytes ({size/1024:.1f} KB)")
    return True


def generate_scr():
    """Generate AutoCAD Script (.scr) with native 3D solid commands."""
    lines = []

    def cmd(*args):
        for a in args:
            lines.append(str(a))

    def blank():
        lines.append("")

    # ── Layer setup (all in one invocation) ──
    cmd("_.-LAYER")
    cmd("_N", "ARM_BASE,ARM_LINK,ARM_JOINT,ARM_GRIPPER,ARM_DIM")
    cmd("_C", "3", "ARM_BASE")
    cmd("_C", "1", "ARM_LINK")
    cmd("_C", "5", "ARM_JOINT")
    cmd("_C", "6", "ARM_GRIPPER")
    cmd("_C", "2", "ARM_DIM")
    cmd("_S", "ARM_BASE")
    blank()

    # ── Base box ──
    hw = BASE_WIDTH / 2
    hd = BASE_DEPTH / 2
    cmd("_BOX")
    cmd(f"{-hw},{-hd},0")
    cmd(f"{hw},{hd},{BASE_HEIGHT}")

    # ── Arm segments ──
    z = float(BASE_HEIGHT)
    for i in range(JOINT_COUNT):
        length = LINK_LENGTHS[i]

        # Set layer to ARM_JOINT
        cmd("_.-LAYER", "_S", "ARM_JOINT")
        blank()
        cmd("_SPHERE")
        cmd(f"0,0,{z}")
        cmd(f"{JOINT_RADIUS}")

        # Set layer to ARM_LINK
        cmd("_.-LAYER", "_S", "ARM_LINK")
        blank()
        cmd("_CYLINDER")
        cmd(f"0,0,{z}")
        cmd(f"{LINK_RADIUS}")
        cmd(f"{length}")

        z += length

    # ── Gripper ──
    cmd("_.-LAYER", "_S", "ARM_GRIPPER")
    blank()

    half_gap = GRIPPER_GAP / 2
    hw_g = GRIPPER_WIDTH / 2
    ht = GRIPPER_THICKNESS / 2

    # Base plate
    pw = half_gap + hw_g
    cmd("_BOX")
    cmd(f"{-pw},{-ht},{z}")
    cmd(f"{pw},{ht},{z + GRIPPER_THICKNESS}")

    # Left prong
    cmd("_BOX")
    cmd(f"{-half_gap - hw_g},{-ht},{z + GRIPPER_THICKNESS}")
    cmd(f"{-half_gap + hw_g},{ht},{z + GRIPPER_THICKNESS + GRIPPER_LENGTH}")

    # Right prong
    cmd("_BOX")
    cmd(f"{half_gap - hw_g},{-ht},{z + GRIPPER_THICKNESS}")
    cmd(f"{half_gap + hw_g},{ht},{z + GRIPPER_THICKNESS + GRIPPER_LENGTH}")

    # Left finger tip (cone)
    tip_z = z + GRIPPER_THICKNESS + GRIPPER_LENGTH
    cmd("_CONE")
    cmd(f"{-half_gap},0,{tip_z}")
    cmd(f"{hw_g}")          # base radius
    cmd("_T")               # top radius option
    cmd(f"{hw_g * 0.3}")    # tapered top
    cmd("15")               # height

    # Right finger tip (cone)
    cmd("_CONE")
    cmd(f"{half_gap},0,{tip_z}")
    cmd(f"{hw_g}")
    cmd("_T")
    cmd(f"{hw_g * 0.3}")
    cmd("15")

    # ── View setup ──
    cmd("_VPOINT", "1,-1,1")
    cmd("_ZOOM", "_E")
    cmd("_-VISUALSTYLES", "_S", "Realistic")
    blank()

    with open(SCR_FILE, "w", encoding="ascii", errors="replace") as f:
        f.write("\n".join(lines) + "\n")

    size = os.path.getsize(SCR_FILE)
    print(f"  SCR saved : {SCR_FILE}")
    print(f"  File size : {size:,} bytes")
    return True


if __name__ == "__main__":
    total_h = BASE_HEIGHT + sum(LINK_LENGTHS) + GRIPPER_THICKNESS + GRIPPER_LENGTH + 15
    print("=" * 55)
    print("  3D Robot Arm Generator")
    print("=" * 55)
    print(f"  Joints      : {JOINT_COUNT}")
    print(f"  Link lengths : {LINK_LENGTHS}")
    print(f"  Base         : {BASE_WIDTH} x {BASE_DEPTH} x {BASE_HEIGHT} mm")
    print(f"  Joint radius : {JOINT_RADIUS} mm")
    print(f"  Link diameter: {LINK_RADIUS * 2} mm")
    print(f"  Total height : ~{total_h} mm")
    print("-" * 55)

    print("\n[1/2] Generating DXF (ezdxf 3DFACE mesh)...")
    try:
        generate_dxf()
        print("  Status: SUCCESS")
    except Exception as e:
        print(f"  Status: FAILED - {e}")

    print(f"\n[2/2] Generating SCR (AutoCAD native 3D solids)...")
    try:
        generate_scr()
        print("  Status: SUCCESS")
    except Exception as e:
        print(f"  Status: FAILED - {e}")

    print("\n" + "=" * 55)
    print("  Done. Two files generated:")
    print(f"    DXF: {DXF_FILE}")
    print(f"    SCR: {SCR_FILE}")
    print("=" * 55)
