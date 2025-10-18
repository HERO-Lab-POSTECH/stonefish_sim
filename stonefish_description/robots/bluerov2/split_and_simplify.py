#!/usr/bin/env python3
"""
BlueROV2 메시를 material별로 분리하고 각각 50% 단순화
"""

import os
import subprocess
import sys
from collections import defaultdict

# 설정
SIMPLIFY_BIN = "/workspace/Fast-Quadric-Mesh-Simplification/simplify"
INPUT_OBJ = "/workspace/bluerov2/bluerov2_description/meshes/bluerov2_noprop.obj"
OUTPUT_DIR = "/workspace/catkin_ws/src/stonefish_description/robots/bluerov2/meshes"
SIMPLIFY_RATIO = 0.5  # 50% 유지
AGGRESSIVENESS = 7.0

def parse_face_indices(face_str):
    """Face 문자열에서 vertex/texcoord/normal 인덱스 추출"""
    parts = face_str.strip().split()[1:]
    indices = []
    for part in parts:
        v_idx = None
        vt_idx = None
        vn_idx = None
        vals = part.split('/')
        if len(vals) >= 1 and vals[0]:
            v_idx = int(vals[0])
        if len(vals) >= 2 and vals[1]:
            vt_idx = int(vals[1])
        if len(vals) >= 3 and vals[2]:
            vn_idx = int(vals[2])
        indices.append((v_idx, vt_idx, vn_idx))
    return indices

def split_obj_by_material(input_obj, output_dir):
    """OBJ 파일을 material별로 분리"""
    print(f"{'='*70}")
    print(f"1단계: Material별 OBJ 분리")
    print(f"{'='*70}\n")

    # 파일 읽기
    with open(input_obj, 'r') as f:
        lines = f.readlines()

    # MTL 파일명
    mtl_file = None
    for line in lines:
        if line.startswith('mtllib '):
            mtl_file = line.strip().split()[1]
            break

    # 전역 데이터
    vertices = []
    normals = []
    texcoords = []
    material_faces = defaultdict(list)
    current_material = None

    # 파싱
    for line in lines:
        line = line.strip()
        if line.startswith('v '):
            vertices.append(line)
        elif line.startswith('vn '):
            normals.append(line)
        elif line.startswith('vt '):
            texcoords.append(line)
        elif line.startswith('usemtl '):
            current_material = line.split()[1]
        elif line.startswith('f '):
            if current_material:
                material_faces[current_material].append(line)

    print(f"총 vertices: {len(vertices):,}")
    print(f"총 normals: {len(normals):,}")
    print(f"총 faces: {sum(len(faces) for faces in material_faces.values()):,}")
    print(f"Material 개수: {len(material_faces)}\n")

    # Material별로 분리
    for material, faces in sorted(material_faces.items()):
        # 사용된 인덱스 수집
        used_v = set()
        used_vn = set()
        used_vt = set()

        for face in faces:
            indices = parse_face_indices(face)
            for v_idx, vt_idx, vn_idx in indices:
                if v_idx: used_v.add(v_idx)
                if vt_idx: used_vt.add(vt_idx)
                if vn_idx: used_vn.add(vn_idx)

        # 인덱스 매핑
        v_map = {old_idx: new_idx for new_idx, old_idx in enumerate(sorted(used_v), 1)}
        vn_map = {old_idx: new_idx for new_idx, old_idx in enumerate(sorted(used_vn), 1)}
        vt_map = {old_idx: new_idx for new_idx, old_idx in enumerate(sorted(used_vt), 1)}

        # 출력 파일
        output_file = os.path.join(output_dir, f"bluerov2_{material}.obj")

        with open(output_file, 'w') as f:
            if mtl_file:
                f.write(f"mtllib {mtl_file}\n")
            f.write(f"o bluerov2_{material}\n")
            f.write(f"g {material}_group\n")

            # 사용된 vertices만
            for old_idx in sorted(used_v):
                f.write(vertices[old_idx - 1] + '\n')
            for old_idx in sorted(used_vn):
                f.write(normals[old_idx - 1] + '\n')
            for old_idx in sorted(used_vt):
                f.write(texcoords[old_idx - 1] + '\n')

            f.write(f"usemtl {material}\n")

            # Faces (인덱스 재매핑)
            for face in faces:
                indices = parse_face_indices(face)
                new_face = "f"
                for v_idx, vt_idx, vn_idx in indices:
                    new_v = v_map.get(v_idx, v_idx) if v_idx else ""
                    new_vt = vt_map.get(vt_idx, vt_idx) if vt_idx else ""
                    new_vn = vn_map.get(vn_idx, vn_idx) if vn_idx else ""

                    if new_vt and new_vn:
                        new_face += f" {new_v}/{new_vt}/{new_vn}"
                    elif new_vt:
                        new_face += f" {new_v}/{new_vt}"
                    elif new_vn:
                        new_face += f" {new_v}//{new_vn}"
                    else:
                        new_face += f" {new_v}"
                f.write(new_face + '\n')

        print(f"{material}: {len(faces):,} faces → {output_file}")

def simplify_meshes(mesh_dir, ratio, aggressiveness):
    """분리된 메시들을 단순화"""
    print(f"\n{'='*70}")
    print(f"2단계: 각 부위별 메시 단순화 ({ratio*100}% 유지)")
    print(f"{'='*70}\n")

    import glob
    material_files = sorted(glob.glob(f"{mesh_dir}/bluerov2_material_*.obj"))

    total_before = 0
    total_after = 0

    for input_file in material_files:
        basename = os.path.basename(input_file)
        output_file = input_file.replace(".obj", "_simplified.obj")

        # simplify 실행
        cmd = [SIMPLIFY_BIN, input_file, output_file, str(ratio), str(aggressiveness)]
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode == 0:
            for line in result.stdout.split('\n'):
                if 'Input:' in line:
                    parts = line.split()
                    for i, part in enumerate(parts):
                        if part == 'triangles':
                            before = int(parts[i-1])
                            total_before += before
                elif 'Output:' in line:
                    parts = line.split()
                    for i, part in enumerate(parts):
                        if part == 'triangles':
                            after = int(parts[i-1])
                            total_after += after
                            print(f"{basename}: {before:,} → {after:,} faces")

    print(f"\n{'='*70}")
    print(f"전체 통계:")
    print(f"  이전 총 faces: {total_before:,}")
    print(f"  이후 총 faces: {total_after:,}")
    print(f"  감소율: {(1-total_after/total_before)*100:.1f}%")
    print(f"{'='*70}")

if __name__ == '__main__':
    # 1단계: Material별 분리
    split_obj_by_material(INPUT_OBJ, OUTPUT_DIR)

    # 2단계: 각 부위 단순화
    simplify_meshes(OUTPUT_DIR, SIMPLIFY_RATIO, AGGRESSIVENESS)

    print("\n✓ 완료!")
