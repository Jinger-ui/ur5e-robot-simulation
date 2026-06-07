"""
Parametric 2D Robot Arm DXF Generator
Generates a multi-joint robot arm drawing with dimensions using ezdxf.
"""

import ezdxf
from ezdxf.enums import TextEntityAlignment

# === Parameters ===
JOINT_COUNT = 5
LINK_LENGTHS = [140, 120, 100, 80, 60]  # mm
BASE_WIDTH = 160  # mm
BASE_HEIGHT = 90  # mm
JOINT_RADIUS = 12  # mm
OUTPUT_FILE = "robot_arm_v1.dxf"

# === Layer definitions ===
LAYERS = {
    "ARM_BASE":  {"color": 3},   # green
    "ARM_LINK":  {"color": 1},   # red
    "ARM_JOINT": {"color": 5},   # blue
    "ARM_DIM":   {"color": 2},   # yellow
}


def create_robot_arm():
    doc = ezdxf.new("R2010")
    msp = doc.modelspace()

    for name, props in LAYERS.items():
        doc.layers.add(name, color=props["color"])

    # --- Base rectangle ---
    base_left = -BASE_WIDTH / 2
    base_bottom = 0.0
    base_right = BASE_WIDTH / 2
    base_top = BASE_HEIGHT

    msp.add_lwpolyline(
        [
            (base_left, base_bottom),
            (base_right, base_bottom),
            (base_right, base_top),
            (base_left, base_top),
            (base_left, base_bottom),
        ],
        dxfattribs={"layer": "ARM_BASE"},
    )

    # --- Base dimension annotations ---
    dim_offset = 20

    # Base width dimension (horizontal, below base)
    msp.add_linear_dim(
        base=(0, base_bottom - dim_offset),
        p1=(base_left, base_bottom),
        p2=(base_right, base_bottom),
        dimstyle="EZDXF",
        override={"dimtxt": 5, "dimclrd": 2, "dimclre": 2, "dimclrt": 2},
        dxfattribs={"layer": "ARM_DIM"},
    ).render()

    # Base height dimension (vertical, to the right of base)
    msp.add_linear_dim(
        base=(base_right + dim_offset, BASE_HEIGHT / 2),
        p1=(base_right, base_bottom),
        p2=(base_right, base_top),
        angle=90,
        dimstyle="EZDXF",
        override={"dimtxt": 5, "dimclrd": 2, "dimclre": 2, "dimclrt": 2},
        dxfattribs={"layer": "ARM_DIM"},
    ).render()

    # --- Links and joints ---
    start_x = 0.0
    start_y = base_top  # top-center of base

    current_x = start_x
    current_y = start_y

    # Draw first joint at base top
    msp.add_circle(
        center=(current_x, current_y),
        radius=JOINT_RADIUS,
        dxfattribs={"layer": "ARM_JOINT"},
    )

    for i in range(JOINT_COUNT):
        link_len = LINK_LENGTHS[i]
        next_x = current_x
        next_y = current_y + link_len

        # Draw link line
        msp.add_line(
            start=(current_x, current_y),
            end=(next_x, next_y),
            dxfattribs={"layer": "ARM_LINK"},
        )

        # Draw joint circle at end of link
        msp.add_circle(
            center=(next_x, next_y),
            radius=JOINT_RADIUS,
            dxfattribs={"layer": "ARM_JOINT"},
        )

        # Dimension for this link (offset to the left)
        link_dim_offset = 30 + i * 15
        msp.add_linear_dim(
            base=(-link_dim_offset, current_y + link_len / 2),
            p1=(current_x, current_y),
            p2=(next_x, next_y),
            angle=90,
            dimstyle="EZDXF",
            override={"dimtxt": 4, "dimclrd": 2, "dimclre": 2, "dimclrt": 2},
            dxfattribs={"layer": "ARM_DIM"},
        ).render()

        current_x = next_x
        current_y = next_y

    # --- End effector (gripper hint) ---
    gripper_len = 25
    gripper_spread = 15
    msp.add_line(
        start=(current_x, current_y),
        end=(current_x - gripper_spread, current_y + gripper_len),
        dxfattribs={"layer": "ARM_LINK"},
    )
    msp.add_line(
        start=(current_x, current_y),
        end=(current_x + gripper_spread, current_y + gripper_len),
        dxfattribs={"layer": "ARM_LINK"},
    )

    # --- Save ---
    doc.saveas(OUTPUT_FILE)
    print(f"DXF file saved: {OUTPUT_FILE}")


if __name__ == "__main__":
    create_robot_arm()
