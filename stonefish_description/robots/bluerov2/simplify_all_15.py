#!/usr/bin/env python3
"""
모든 material OBJ 파일을 15%로 단순화
"""

import os
import subprocess
import glob

# 설정
SIMPLIFY_BIN = "/workspace/Fast-Quadric-Mesh-Simplification/simplify"
MESH_DIR = "/workspace/catkin_ws/src/stonefish_description/robots/bluerov2/meshes"
RATIO = 0.15  # 15% 유지
AGGRESSIVENESS = 7.0

# 분리된 material OBJ 파일들 (원본, _simplified 제외)
material_files = sorted(glob.glob(f"{MESH_DIR}/bluerov2_material_[0-9]*.obj"))
material_files = [f for f in material_files if '_simplified' not in f]

print(f"{'='*70}")
print(f"BlueROV2 메시 단순화 (비율: {RATIO*100}%)")
print(f"{'='*70}\n")

total_before = 0
total_after = 0

for input_file in material_files:
    basename = os.path.basename(input_file)
    output_file = input_file.replace(".obj", "_simplified.obj")

    print(f"처리 중: {basename}")

    # simplify 실행
    cmd = [SIMPLIFY_BIN, input_file, output_file, str(RATIO), str(AGGRESSIVENESS)]
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode == 0:
        # 출력에서 faces 정보 추출
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
                        print(f"  {before:,} → {after:,} faces")
    else:
        print(f"  오류 발생: {result.stderr}")

    print()

print(f"{'='*70}")
print(f"전체 통계:")
print(f"  이전 총 faces: {total_before:,}")
print(f"  이후 총 faces: {total_after:,}")
print(f"  감소율: {(1-total_after/total_before)*100:.1f}%")
print(f"  원본 대비: {total_after/307785*100:.1f}%")
print(f"{'='*70}")
