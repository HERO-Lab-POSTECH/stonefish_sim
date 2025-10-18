#!/usr/bin/env python3
"""
각 부품의 bounding box 크기를 측정하고 physical용 box mesh 생성
"""

import os

# 부품 파일들
parts = {
    'duct': 'bluerov2_duct.obj',
    'buoy_cover': 'bluerov2_buoy_cover.obj',
    'plate': 'bluerov2_plate.obj',
    'hull': 'bluerov2_hull.obj'
}

mesh_dir = '/workspace/catkin_ws/src/stonefish_description/robots/bluerov2/meshes'

def get_bounding_box(obj_file):
    """OBJ 파일의 bounding box 계산"""
    vertices = []
    with open(obj_file, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.strip().split()
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                vertices.append((x, y, z))

    min_x = min(v[0] for v in vertices)
    max_x = max(v[0] for v in vertices)
    min_y = min(v[1] for v in vertices)
    max_y = max(v[1] for v in vertices)
    min_z = min(v[2] for v in vertices)
    max_z = max(v[2] for v in vertices)

    width = max_x - min_x
    depth = max_y - min_y
    height = max_z - min_z
    center_x = (max_x + min_x) / 2
    center_y = (max_y + min_y) / 2
    center_z = (max_z + min_z) / 2

    return (width, depth, height, center_x, center_y, center_z)

def create_box_mesh(name, width, depth, height, center_x, center_y, center_z, output_file):
    """Box mesh OBJ 생성 (중심 위치 고려)"""
    # Box의 8개 꼭지점 (중심 기준)
    hw = width / 2
    hd = depth / 2
    hh = height / 2

    vertices = [
        (center_x - hw, center_y - hd, center_z - hh),  # 1
        (center_x + hw, center_y - hd, center_z - hh),  # 2
        (center_x + hw, center_y + hd, center_z - hh),  # 3
        (center_x - hw, center_y + hd, center_z - hh),  # 4
        (center_x - hw, center_y - hd, center_z + hh),  # 5
        (center_x + hw, center_y - hd, center_z + hh),  # 6
        (center_x + hw, center_y + hd, center_z + hh),  # 7
        (center_x - hw, center_y + hd, center_z + hh),  # 8
    ]

    # Box의 12개 면 (삼각형 2개씩)
    faces = [
        # Bottom (-Z)
        (1, 2, 3), (1, 3, 4),
        # Top (+Z)
        (5, 7, 6), (5, 8, 7),
        # Front (-Y)
        (1, 5, 6), (1, 6, 2),
        # Back (+Y)
        (4, 3, 7), (4, 7, 8),
        # Left (-X)
        (1, 4, 8), (1, 8, 5),
        # Right (+X)
        (2, 6, 7), (2, 7, 3),
    ]

    with open(output_file, 'w') as f:
        f.write(f"# Physical box for {name}\n")
        f.write(f"# Size: {width:.4f}m x {depth:.4f}m x {height:.4f}m\n")
        f.write(f"# Center: ({center_x:.4f}, {center_y:.4f}, {center_z:.4f})\n\n")
        f.write(f"o {name}_box\n")
        f.write(f"g {name}_box_group\n\n")

        # Vertices
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")

        f.write("\n")

        # Normals (6개 방향)
        normals = [
            (0, 0, -1),  # Bottom
            (0, 0, 1),   # Top
            (0, -1, 0),  # Front
            (0, 1, 0),   # Back
            (-1, 0, 0),  # Left
            (1, 0, 0),   # Right
        ]
        for n in normals:
            f.write(f"vn {n[0]} {n[1]} {n[2]}\n")

        f.write("\n")

        # Faces with normals
        for i, face in enumerate(faces):
            normal_idx = i // 2 + 1  # 각 면마다 같은 normal
            f.write(f"f {face[0]}//{normal_idx} {face[1]}//{normal_idx} {face[2]}//{normal_idx}\n")

print("="*70)
print("BlueROV2 부품별 Physical Box 생성")
print("="*70 + "\n")

for part_name, mesh_file in parts.items():
    obj_path = os.path.join(mesh_dir, mesh_file)

    # Bounding box 계산
    width, depth, height, cx, cy, cz = get_bounding_box(obj_path)

    print(f"{part_name.upper()}:")
    print(f"  크기: {width:.4f}m × {depth:.4f}m × {height:.4f}m")
    print(f"  중심: ({cx:.4f}, {cy:.4f}, {cz:.4f})")

    # Physical box mesh 생성
    box_file = os.path.join(mesh_dir, f"bluerov2_{part_name}_phy.obj")
    create_box_mesh(part_name, width, depth, height, cx, cy, cz, box_file)

    print(f"  생성: {box_file}")
    print()

print("="*70)
print("✓ 완료!")
print("="*70)
