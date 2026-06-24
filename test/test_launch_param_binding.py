"""launch 노드명 ↔ yaml 파라미터 키 정합 게이트 (T1.2/H4, P4 Phase 1, golden-blind [T1']).

ROS2는 yaml 파라미터 블록을 노드의 *런타임* 이름(launch의 name= 가 있으면 그 값,
없으면 생성자 super().__init__('X') 문자열)으로 매칭한다. yaml 최상위 키가 런타임명과
다르고 wildcard('/**')도 아니면, 파라미터가 조용히 declare_parameter 기본값으로
fallback한다(게인 미로딩).

T1.2: hybrid_controller_node 생성자명은 'hybrid_controller_4dof'이나 controller.launch.py
와 path_following.launch.py가 name='hybrid_controller'로 덮어쓰고, hybrid_controller.yaml
키는 'hybrid_controller_4dof:'(non-wildcard) → 매칭 실패, position_mode.max_force가
800→200(4×)로 silent 저하.

이 게이트(architect 권고)는 각 launch Node가 로드하는 yaml이 **wildcard('/**') 이거나
런타임 실효명과 일치하는 키**를 가지는지 검사한다 — 단순 3자 string-equal이 아니다
(그러면 정당하게 wildcard된 path_following_node를 잘못 fail시킨다).

rclpy registry 의미라 fake-node 재현 불가(golden-blind) — 정적 RED + 런타임 sign-off.
"""
import ast
import re
from pathlib import Path

import pytest

try:
    import yaml
except ImportError:  # PyYAML 부재 시 skip(컨테이너엔 있음)
    yaml = None

REPO_ROOT = Path(__file__).resolve().parent.parent

# (launch 파일, 패키지 config 디렉토리) — launch가 PathJoinSubstitution으로 yaml 경로를
# 합성하므로, yaml 파일명만 추출해 이 디렉토리에서 찾는다.
_LAUNCH_FILES = [
    REPO_ROOT / "stonefish_control/stonefish_control/launch/controller.launch.py",
    REPO_ROOT / "stonefish_control/stonefish_trajectory_manager/launch/path_following.launch.py",
    REPO_ROOT / "stonefish_control/stonefish_trajectory_manager/launch/path_generator.launch.py",
]

# config yaml 탐색 루트(패키지별)
_CONFIG_ROOTS = [
    REPO_ROOT / "stonefish_control/stonefish_control/config",
    REPO_ROOT / "stonefish_control/stonefish_trajectory_manager/config",
]


def _yaml_top_keys(yaml_path):
    """yaml 최상위 키 집합. wildcard('/**') 포함 여부 판정용."""
    if yaml is None:
        pytest.skip("PyYAML not available")
    try:
        data = yaml.safe_load(yaml_path.read_text())
    except Exception:
        return set()
    return set(data.keys()) if isinstance(data, dict) else set()


def _find_yaml(filename):
    """config 루트들에서 filename(.yaml stem 또는 풀네임) 탐색."""
    for root in _CONFIG_ROOTS:
        for cand in root.rglob(filename):
            return cand
    return None


def _extract_controller_nodes(launch_file):
    """launch 파일에서 Node(...) 중 controller류를 (name=값, 로드 yaml stem들)로 추출.

    name= 값과 parameters=[...]에 등장하는 *_controller*.yaml 파일명을 AST로 뽑는다.
    yaml 경로는 PathJoinSubstitution이라 마지막 문자열 리터럴(파일명)만 본다.
    """
    src = launch_file.read_text()
    tree = ast.parse(src)
    results = []
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and getattr(node.func, "id", None) == "Node"):
            continue
        name_val = None
        for kw in node.keywords:
            if kw.arg == "name" and isinstance(kw.value, ast.Constant):
                name_val = kw.value.value
        if name_val is None:
            continue
        # 이 Node가 controller인지: name에 controller 포함, 또는 executable이 controller
        results.append(name_val)
    return results


def _yaml_files_referenced(launch_file):
    """launch 파일이 참조하는 *.yaml 파일명(controller류) 집합."""
    src = launch_file.read_text()
    return set(re.findall(r"'([a-z_]+_controller\.yaml)'", src)) | \
        set(re.findall(r'"([a-z_]+_controller\.yaml)"', src))


def test_hybrid_controller_yaml_binds_to_runtime_name():
    """hybrid_controller.yaml이 wildcard이거나 런타임명 'hybrid_controller'와 매칭되는가.

    RED: 현재 키 'hybrid_controller_4dof'(non-wildcard) ≠ 런타임명 'hybrid_controller'.
    GREEN: 키를 '/**'로 (또는 'hybrid_controller'로) 변경 후 통과.
    """
    yaml_path = _find_yaml("hybrid_controller.yaml")
    assert yaml_path is not None, "hybrid_controller.yaml not found"
    keys = _yaml_top_keys(yaml_path)
    # 런타임 실효명: 두 launch 모두 name='hybrid_controller'로 덮어씀
    runtime_name = "hybrid_controller"
    ok = ("/**" in keys) or (runtime_name in keys)
    assert ok, (
        f"hybrid_controller.yaml top-level keys {keys} bind to neither '/**' "
        f"(wildcard) nor the launch runtime name '{runtime_name}' → params silently "
        f"fall back to declare_parameter defaults (e.g. position_mode.max_force "
        f"800->200). Use '/**:' per repo convention."
    )


def test_all_launched_controllers_bind_params():
    """모든 launch Node의 로드 yaml이 wildcard이거나 런타임명과 매칭(일반화 게이트)."""
    failures = []
    for lf in _LAUNCH_FILES:
        if not lf.exists():
            continue
        runtime_names = _extract_controller_nodes(lf)
        yaml_names = _yaml_files_referenced(lf)
        for yname in yaml_names:
            yaml_path = _find_yaml(yname)
            if yaml_path is None:
                continue
            keys = _yaml_top_keys(yaml_path)
            if "/**" in keys:
                continue  # wildcard absorbs any runtime name
            # non-wildcard면 런타임명 중 하나와 키가 일치해야 함
            if not (keys & set(runtime_names)):
                failures.append(
                    f"{lf.name}: {yname} keys {keys} match no runtime name "
                    f"{runtime_names} and is not wildcard"
                )
    assert failures == [], "H4 param-binding failures:\n" + "\n".join(failures)
