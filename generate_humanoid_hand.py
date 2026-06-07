"""
Parametric Humanoid Robot Hand 3D Model Generator
Generates a detailed five-finger anthropomorphic hand model in DXF format
with accompanying AutoCAD SCR script.

Structure:
- Palm: flat rectangular box (~90x80x25mm)
- 5 fingers (Thumb, Index, Middle, Ring, Pinky)
- Each finger: 3 phalanges (thumb: 2), connected by spherical joints
- Phalanges approximated as cylinders
- Joints approximated as spheres

Layers:
- HAND_PALM (green, color 3)
- HAND_FINGER (red, color 1)
- HAND_JOINT (blue, color 5)
- HAND_DIM (yellow, color 2)
"""

import ezdxf
import math
from ezdxf.math import Vec3

def create_cylinder_3d(msp, center, radius, height, axis='z', segments=16, layer='HAND_FINGER'):
    """Create a 3D cylinder using 3DFACE entities along given axis."""
    cx, cy, cz = center
    faces = []
    
    for i in range(segments):
        angle1 = 2 * math.pi * i / segments
        angle2 = 2 * math.pi * (i + 1) / segments
        
        if axis == 'z':
            x1 = cx + radius * math.cos(angle1)
            y1 = cy + radius * math.sin(angle1)
            x2 = cx + radius * math.cos(angle2)
            y2 = cy + radius * math.sin(angle2)
            p1 = (x1, y1, cz)
            p2 = (x2, y2, cz)
            p3 = (x2, y2, cz + height)
            p4 = (x1, y1, cz + height)
        elif axis == 'y':
            x1 = cx + radius * math.cos(angle1)
            z1 = cz + radius * math.sin(angle1)
            x2 = cx + radius * math.cos(angle2)
            z2 = cz + radius * math.sin(angle2)
            p1 = (x1, cy, z1)
            p2 = (x2, cy, z2)
            p3 = (x2, cy + height, z2)
            p4 = (x1, cy + height, z1)
        else:  # axis == 'x'
            y1 = cy + radius * math.cos(angle1)
            z1 = cz + radius * math.sin(angle1)
            y2 = cy + radius * math.cos(angle2)
            z2 = cz + radius * math.sin(angle2)
            p1 = (cx, y1, z1)
            p2 = (cx, y2, z2)
            p3 = (cx + height, y2, z2)
            p4 = (cx + height, y1, z1)
        
        msp.add_3dface([p1, p2, p3, p4], dxfattribs={'layer': layer})
        
    # Top and bottom caps
    for i in range(segments):
        angle1 = 2 * math.pi * i / segments
        angle2 = 2 * math.pi * (i + 1) / segments
        
        if axis == 'z':
            x1 = cx + radius * math.cos(angle1)
            y1 = cy + radius * math.sin(angle1)
            x2 = cx + radius * math.cos(angle2)
            y2 = cy + radius * math.sin(angle2)
            # Bottom cap
            msp.add_3dface([(cx, cy, cz), (x1, y1, cz), (x2, y2, cz), (cx, cy, cz)],
                          dxfattribs={'layer': layer})
            # Top cap
            msp.add_3dface([(cx, cy, cz+height), (x1, y1, cz+height), (x2, y2, cz+height), (cx, cy, cz+height)],
                          dxfattribs={'layer': layer})
        elif axis == 'y':
            x1 = cx + radius * math.cos(angle1)
            z1 = cz + radius * math.sin(angle1)
            x2 = cx + radius * math.cos(angle2)
            z2 = cz + radius * math.sin(angle2)
            msp.add_3dface([(cx, cy, cz), (x1, cy, z1), (x2, cy, z2), (cx, cy, cz)],
                          dxfattribs={'layer': layer})
            msp.add_3dface([(cx, cy+height, cz), (x1, cy+height, z1), (x2, cy+height, z2), (cx, cy+height, cz)],
                          dxfattribs={'layer': layer})


def create_sphere_3d(msp, center, radius, u_segments=12, v_segments=8, layer='HAND_JOINT'):
    """Create a 3D sphere using 3DFACE entities."""
    cx, cy, cz = center
    
    for i in range(u_segments):
        for j in range(v_segments):
            phi1 = math.pi * j / v_segments - math.pi / 2
            phi2 = math.pi * (j + 1) / v_segments - math.pi / 2
            theta1 = 2 * math.pi * i / u_segments
            theta2 = 2 * math.pi * (i + 1) / u_segments
            
            p1 = (cx + radius * math.cos(phi1) * math.cos(theta1),
                   cy + radius * math.cos(phi1) * math.sin(theta1),
                   cz + radius * math.sin(phi1))
            p2 = (cx + radius * math.cos(phi1) * math.cos(theta2),
                   cy + radius * math.cos(phi1) * math.sin(theta2),
                   cz + radius * math.sin(phi1))
            p3 = (cx + radius * math.cos(phi2) * math.cos(theta2),
                   cy + radius * math.cos(phi2) * math.sin(theta2),
                   cz + radius * math.sin(phi2))
            p4 = (cx + radius * math.cos(phi2) * math.cos(theta1),
                   cy + radius * math.cos(phi2) * math.sin(theta1),
                   cz + radius * math.sin(phi2))
            
            msp.add_3dface([p1, p2, p3, p4], dxfattribs={'layer': layer})


def create_box_3d(msp, origin, width, depth, height, layer='HAND_PALM'):
    """Create a 3D box using 3DFACE entities."""
    x, y, z = origin
    w, d, h = width, depth, height
    
    vertices = [
        (x, y, z), (x+w, y, z), (x+w, y+d, z), (x, y+d, z),
        (x, y, z+h), (x+w, y, z+h), (x+w, y+d, z+h), (x, y+d, z+h)
    ]
    
    faces = [
        [0, 1, 2, 3],  # bottom
        [4, 5, 6, 7],  # top
        [0, 1, 5, 4],  # front
        [2, 3, 7, 6],  # back
        [0, 3, 7, 4],  # left
        [1, 2, 6, 5],  # right
    ]
    
    for face in faces:
        pts = [vertices[i] for i in face]
        msp.add_3dface(pts, dxfattribs={'layer': layer})


def create_finger(msp, base_pos, phalanx_lengths, phalanx_radius, joint_radius, 
                  direction=(0, 1, 0), angle_offset=0):
    """
    Create a finger with multiple phalanges and joints.
    direction: primary direction the finger extends
    """
    dx, dy, dz = direction
    current_pos = list(base_pos)
    
    for i, length in enumerate(phalanx_lengths):
        # Joint sphere at the base of each phalanx
        create_sphere_3d(msp, tuple(current_pos), joint_radius, 
                        u_segments=10, v_segments=6, layer='HAND_JOINT')
        
        # Phalanx cylinder
        if dy != 0:
            create_cylinder_3d(msp, tuple(current_pos), phalanx_radius, 
                             length * dy, axis='y', segments=12, layer='HAND_FINGER')
        elif dx != 0:
            create_cylinder_3d(msp, tuple(current_pos), phalanx_radius, 
                             length * dx, axis='x', segments=12, layer='HAND_FINGER')
        else:
            create_cylinder_3d(msp, tuple(current_pos), phalanx_radius, 
                             length * dz, axis='z', segments=12, layer='HAND_FINGER')
        
        # Move to next joint position
        current_pos[0] += dx * length
        current_pos[1] += dy * length
        current_pos[2] += dz * length
    
    # Fingertip sphere
    create_sphere_3d(msp, tuple(current_pos), phalanx_radius * 0.8, 
                    u_segments=8, v_segments=6, layer='HAND_FINGER')


def add_dimensions(msp, palm_origin, palm_width, palm_depth, palm_height):
    """Add dimension annotations."""
    x, y, z = palm_origin
    
    # Palm width dimension text
    msp.add_text(f"Palm: {palm_width}mm x {palm_depth}mm x {palm_height}mm",
                dxfattribs={
                    'layer': 'HAND_DIM',
                    'height': 3,
                    'insert': (x, y - 15, 0)
                })
    
    # Overall label
    msp.add_text("Humanoid Robot Hand - 5 Fingers, 14 Joints",
                dxfattribs={
                    'layer': 'HAND_DIM',
                    'height': 4,
                    'insert': (x, y - 25, 0)
                })
    
    # Finger labels
    finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
    finger_x_positions = [x - 10, x + 10, x + 27, x + 44, x + 61]
    for name, fx in zip(finger_names, finger_x_positions):
        msp.add_text(name, dxfattribs={
            'layer': 'HAND_DIM',
            'height': 2.5,
            'insert': (fx, y + palm_depth + 85, 0)
        })


def generate_humanoid_hand():
    """Main function to generate the humanoid hand DXF model."""
    doc = ezdxf.new('R2010')
    msp = doc.modelspace()
    
    # Setup layers
    doc.layers.add('HAND_PALM', color=3)    # Green
    doc.layers.add('HAND_FINGER', color=1)  # Red
    doc.layers.add('HAND_JOINT', color=5)   # Blue
    doc.layers.add('HAND_DIM', color=2)     # Yellow
    
    # Palm parameters (mm)
    palm_width = 80    # X direction
    palm_depth = 90    # Y direction  
    palm_height = 25   # Z direction
    palm_origin = (0, 0, 0)
    
    # Create palm
    create_box_3d(msp, palm_origin, palm_width, palm_depth, palm_height, layer='HAND_PALM')
    
    # Add a slight palm arch (curved top surface detail)
    for i in range(8):
        angle1 = math.pi * i / 8
        angle2 = math.pi * (i + 1) / 8
        x1 = palm_width/2 + (palm_width/2 - 5) * math.cos(angle1)
        x2 = palm_width/2 + (palm_width/2 - 5) * math.cos(angle2)
        z1 = palm_height + 3 * math.sin(angle1)
        z2 = palm_height + 3 * math.sin(angle2)
        msp.add_3dface(
            [(x1, 10, z1), (x2, 10, z2), (x2, palm_depth-10, z2), (x1, palm_depth-10, z1)],
            dxfattribs={'layer': 'HAND_PALM'}
        )
    
    # Finger parameters
    # Format: (base_x, base_y, base_z, phalanx_lengths, radius, direction)
    finger_spacing = 17  # spacing between fingers
    finger_base_y = palm_depth  # fingers start at top of palm
    finger_base_z = palm_height / 2  # centered vertically on palm
    
    fingers = {
        'thumb': {
            'base': (-5, palm_depth * 0.4, palm_height / 2),
            'phalanx_lengths': [30, 25],  # 2 phalanges for thumb
            'radius': 8,
            'joint_radius': 6,
            'direction': (-0.5, 0.866, 0),  # angled outward ~30 degrees
        },
        'index': {
            'base': (10, finger_base_y, finger_base_z),
            'phalanx_lengths': [28, 22, 18],
            'radius': 6.5,
            'joint_radius': 5,
            'direction': (0, 1, 0),
        },
        'middle': {
            'base': (10 + finger_spacing, finger_base_y, finger_base_z),
            'phalanx_lengths': [32, 24, 19],
            'radius': 6.5,
            'joint_radius': 5,
            'direction': (0, 1, 0),
        },
        'ring': {
            'base': (10 + 2*finger_spacing, finger_base_y, finger_base_z),
            'phalanx_lengths': [29, 22, 17],
            'radius': 6,
            'joint_radius': 4.5,
            'direction': (0, 1, 0),
        },
        'pinky': {
            'base': (10 + 3*finger_spacing, finger_base_y, finger_base_z),
            'phalanx_lengths': [22, 17, 14],
            'radius': 5.5,
            'joint_radius': 4,
            'direction': (0, 1, 0),
        },
    }
    
    # Generate each finger
    for name, params in fingers.items():
        create_finger(
            msp,
            base_pos=params['base'],
            phalanx_lengths=params['phalanx_lengths'],
            phalanx_radius=params['radius'],
            joint_radius=params['joint_radius'],
            direction=params['direction']
        )
    
    # Add wrist connector (cylinder at bottom of palm)
    wrist_center = (palm_width/2, -5, palm_height/2)
    create_cylinder_3d(msp, (palm_width/2, -20, palm_height/2 - 12), 
                      18, 20, axis='y', segments=16, layer='HAND_PALM')
    
    # Add knuckle bumps on top of palm
    for i in range(4):
        knuckle_x = 10 + i * finger_spacing
        knuckle_y = finger_base_y - 3
        knuckle_z = palm_height + 2
        create_sphere_3d(msp, (knuckle_x, knuckle_y, knuckle_z), 5,
                        u_segments=8, v_segments=6, layer='HAND_PALM')
    
    # Add dimension annotations
    add_dimensions(msp, palm_origin, palm_width, palm_depth, palm_height)
    
    # Add coordinate axes for reference
    msp.add_line((0, 0, 0), (20, 0, 0), dxfattribs={'layer': 'HAND_DIM'})  # X - red
    msp.add_line((0, 0, 0), (0, 20, 0), dxfattribs={'layer': 'HAND_DIM'})  # Y - green
    msp.add_line((0, 0, 0), (0, 0, 20), dxfattribs={'layer': 'HAND_DIM'})  # Z - blue
    msp.add_text("X", dxfattribs={'layer': 'HAND_DIM', 'height': 3, 'insert': (22, 0, 0)})
    msp.add_text("Y", dxfattribs={'layer': 'HAND_DIM', 'height': 3, 'insert': (0, 22, 0)})
    msp.add_text("Z", dxfattribs={'layer': 'HAND_DIM', 'height': 3, 'insert': (0, 0, 22)})
    
    # Save DXF
    output_path = r'c:\Users\Cleveland\Desktop\solidworks\humanoid_hand_3d.dxf'
    doc.saveas(output_path)
    print(f"DXF file saved: {output_path}")
    
    return output_path


def generate_scr_script():
    """Generate AutoCAD SCR script for the humanoid hand."""
    scr_path = r'c:\Users\Cleveland\Desktop\solidworks\humanoid_hand_3d.scr'
    
    lines = []
    lines.append("_FILEDIA 0")
    lines.append("-UNITS 2 4 1 4 0 N")
    lines.append("")
    
    # Set up layers
    lines.append("-LAYER M HAND_PALM C 3 HAND_PALM ")
    lines.append("-LAYER M HAND_FINGER C 1 HAND_FINGER ")
    lines.append("-LAYER M HAND_JOINT C 5 HAND_JOINT ")
    lines.append("-LAYER M HAND_DIM C 2 HAND_DIM ")
    lines.append("")
    
    # Set current layer to HAND_PALM and create palm box
    lines.append("-LAYER S HAND_PALM ")
    lines.append("_BOX 0,0,0 80,90,25")
    lines.append("")
    
    # Wrist cylinder
    lines.append("_CYLINDER 40,-20,12.5 18 H 20")
    lines.append("")
    
    # Create knuckle spheres
    lines.append("-LAYER S HAND_JOINT ")
    for i in range(4):
        x = 10 + i * 17
        lines.append(f"_SPHERE {x},87,27 5")
    lines.append("")
    
    # Create fingers
    lines.append("-LAYER S HAND_FINGER ")
    
    # Finger definitions: name, base_x, base_y, phalanx_lengths, radius
    finger_defs = [
        ("Index", 10, 90, [28, 22, 18], 6.5, 5),
        ("Middle", 27, 90, [32, 24, 19], 6.5, 5),
        ("Ring", 44, 90, [29, 22, 17], 6, 4.5),
        ("Pinky", 61, 90, [22, 17, 14], 5.5, 4),
    ]
    
    base_z = 12.5
    
    for name, bx, by, lengths, radius, jradius in finger_defs:
        current_y = by
        for i, length in enumerate(lengths):
            # Joint
            lines.append(f"-LAYER S HAND_JOINT ")
            lines.append(f"_SPHERE {bx},{current_y},{base_z} {jradius}")
            # Phalanx
            lines.append(f"-LAYER S HAND_FINGER ")
            lines.append(f"_CYLINDER {bx},{current_y},{base_z} {radius} H {length}")
            current_y += length
        # Fingertip
        lines.append(f"_SPHERE {bx},{current_y},{base_z} {radius*0.8:.1f}")
        lines.append("")
    
    # Thumb (angled)
    lines.append("; Thumb - angled outward")
    lines.append(f"-LAYER S HAND_JOINT ")
    lines.append(f"_SPHERE -5,36,12.5 6")
    lines.append(f"-LAYER S HAND_FINGER ")
    lines.append(f"_CYLINDER -5,36,12.5 8 H 30")
    lines.append(f"-LAYER S HAND_JOINT ")
    lines.append(f"_SPHERE -20,62,12.5 6")
    lines.append(f"-LAYER S HAND_FINGER ")
    lines.append(f"_CYLINDER -20,62,12.5 8 H 25")
    lines.append(f"_SPHERE -32.5,83.6,12.5 6.4")
    lines.append("")
    
    # Dimension text
    lines.append("-LAYER S HAND_DIM ")
    lines.append('_TEXT 0,-15,0 3 0 Palm: 80mm x 90mm x 25mm')
    lines.append('_TEXT 0,-25,0 4 0 Humanoid Robot Hand - 5 Fingers, 14 Joints')
    lines.append("")
    
    # Set 3D view
    lines.append("_-VIEW _SWISO")
    lines.append("_ZOOM _E")
    lines.append("_VSCURRENT _REALISTIC")
    lines.append("")
    lines.append("_FILEDIA 1")
    lines.append("")
    
    with open(scr_path, 'w') as f:
        f.write('\n'.join(lines))
    
    print(f"SCR script saved: {scr_path}")
    return scr_path


if __name__ == '__main__':
    dxf_path = generate_humanoid_hand()
    scr_path = generate_scr_script()
    
    import os
    dxf_size = os.path.getsize(dxf_path)
    scr_size = os.path.getsize(scr_path)
    print(f"\nGenerated files:")
    print(f"  {os.path.basename(dxf_path)}: {dxf_size:,} bytes ({dxf_size/1024:.1f} KB)")
    print(f"  {os.path.basename(scr_path)}: {scr_size:,} bytes ({scr_size/1024:.1f} KB)")
