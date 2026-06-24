"""정적 import/entry 검증 도구 (P3 안전망, rclpy 부재 환경용).

이 환경엔 rclpy가 없어 노드 모듈을 실제 import(exec_module)하면 `import rclpy`에서
즉시 죽는다. 따라서 "모듈 실제 로드" import-smoke가 불가능하다. 대신 **정적 분석**
(ast.parse + py_compile + 파일경로 해석)으로 동작보존 불변식을 검증한다.

architect 적대 검증(2026-06-23) 권고를 반영한 5개 게이트:
  G1  console_scripts entry-chain 정합성 (엔트리 우변 → 파일 존재 → top-level main 심볼)
  G2  velocity dead-state pin (resolution-based: pid_4dof 타겟이 디스크에 부재)
  G3  모듈 top-level 부작용 0 (topic/path/__file__/__name__ 동적 참조 실행문 없음)
  G4  __init__.py eager node-import 금지
  G5  py_compile + resolved import target-set (T3 이동/변환 전후 비교용)

⚠️ 한계(정직하게 명시): 이 게이트들은 경로/엔트리/타겟집합 불변을 **정적으로** 증명할
뿐, 런타임 심볼 binding(동명이클래스, __init__ re-export 섀도잉)은 증명하지 못한다 —
그건 colcon build + ros2 launch가 필요하며 P4로 미룬다.
"""
import ast
import py_compile
from pathlib import Path

# repo root = test/ 의 부모
REPO_ROOT = Path(__file__).resolve().parent.parent

# ament_python 패키지: (패키지명, src 루트 디렉토리) — 모듈경로 ↔ 파일경로 해석에 사용
_PKG_SRC_ROOTS = {
    'stonefish_control':
        REPO_ROOT / 'stonefish_control/stonefish_control',
    'stonefish_thruster_manager':
        REPO_ROOT / 'stonefish_control/stonefish_thruster_manager',
    'stonefish_trajectory_manager':
        REPO_ROOT / 'stonefish_control/stonefish_trajectory_manager',
}

# 서드파티/외부 — 절대 import여도 repo 내부 파일로 해석하지 않음
_EXTERNAL_TOP = {
    'rclpy', 'numpy', 'np', 'scipy', 'transforms3d', 'std_msgs', 'nav_msgs',
    'geometry_msgs', 'sensor_msgs', 'visualization_msgs', 'builtin_interfaces',
    'rclpy', 'tf2_ros', 'ament_index_python', 'casadi', 'math', 'sys', 'os',
    'traceback', 'collections', 'typing', 'dataclasses', 'enum', 'abc',
    'stonefish_control_msgs', 'stonefish_msgs',
}


def module_to_file(dotted):
    """`pkg.sub.module` 점표기 → 실제 .py 파일 Path. 해석 불가면 None."""
    parts = dotted.split('.')
    top = parts[0]
    if top not in _PKG_SRC_ROOTS:
        return None
    base = _PKG_SRC_ROOTS[top]
    # src 루트 아래는 다시 패키지명 디렉토리로 시작 (ament_python 이중 명명)
    candidate = base.joinpath(*parts).with_suffix('.py')
    if candidate.exists():
        return candidate
    pkg_init = base.joinpath(*parts) / '__init__.py'
    if pkg_init.exists():
        return pkg_init
    return None


def top_level_main_symbol(pyfile):
    """파일의 **모듈 top-level**에 `def main` 또는 `main = ...`이 있으면 True (실행 안 함)."""
    tree = ast.parse(Path(pyfile).read_text())
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == 'main':
            return True
        if isinstance(node, ast.AsyncFunctionDef) and node.name == 'main':
            return True
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == 'main':
                    return True
    return False


def resolve_entry(rhs):
    """console_scripts 우변 `pkg.path.module:func` → (파일 Path|None, func명)."""
    module_path, _, func = rhs.partition(':')
    return module_to_file(module_path.strip()), func.strip()


def _import_targets_for_file(pyfile):
    """파일 내 ImportFrom 노드를 (level, module, names) 튜플 집합으로.

    상대 import의 level을 파일의 패키지 위치로 풀어 **완전수식 점표기**로 정규화한다.
    절대/상대 어느 형태든 같은 모듈을 가리키면 동일 튜플이 나오도록 — T3 이동/변환
    전후 target-set diff의 핵심.
    """
    pyfile = Path(pyfile).resolve()
    # 파일이 속한 패키지 점표기 prefix 계산
    pkg_prefix = None
    for top, root in _PKG_SRC_ROOTS.items():
        root = root.resolve()
        try:
            rel = pyfile.relative_to(root)
        except ValueError:
            continue
        # rel.parts[:-1] = 패키지 내부 경로(파일명 제외)
        pkg_prefix = list(rel.parts[:-1])
        break
    targets = set()
    tree = ast.parse(pyfile.read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom):
            continue
        names = tuple(sorted(a.name for a in node.names))
        if node.level == 0:
            # 절대 import: 외부 top이면 외부 마커로 기록(파일 비교 제외)
            mod = node.module or ''
            top = mod.split('.')[0]
            if top in _EXTERNAL_TOP or top not in _PKG_SRC_ROOTS:
                targets.add(('EXTERNAL', mod, names))
            else:
                targets.add(('REPO', mod, names))
        else:
            # 상대 import: pkg_prefix에서 (level-1)단계 올라간 뒤 module 부착
            if pkg_prefix is None:
                targets.add(('UNRESOLVED', f'.{node.level}:{node.module}', names))
                continue
            up = node.level - 1
            base = pkg_prefix[:len(pkg_prefix) - up] if up <= len(pkg_prefix) else None
            if base is None:
                targets.add(('UNRESOLVED', f'.{node.level}:{node.module}', names))
                continue
            mod_parts = base + ([node.module] if node.module else [])
            dotted = '.'.join(mod_parts)
            targets.add(('REPO', dotted, names))
    return targets


def repo_import_targets(pyfile):
    """파일의 repo-내부 import 타겟 집합(외부 import 제외). 이동/변환 전후 비교용."""
    return {t for t in _import_targets_for_file(pyfile) if t[0] == 'REPO'}


def py_compiles(pyfile):
    """py_compile로 바이트컴파일(실행 안 함). 성공 True. rclpy-safe."""
    try:
        py_compile.compile(str(pyfile), doraise=True)
        return True
    except py_compile.PyCompileError:
        return False


def init_eager_imports_node(init_file):
    """__init__.py가 `from .X_node import ...`로 노드를 eager import하면 True."""
    tree = ast.parse(Path(init_file).read_text())
    for node in tree.body:
        if isinstance(node, ast.ImportFrom) and node.module:
            if node.module.endswith('_node') or '_node' in node.module:
                return True
        if isinstance(node, ast.ImportFrom) and node.level > 0:
            for a in node.names:
                # from .path_generator_node import main 형태
                if node.module and 'node' in node.module:
                    return True
    return False


def module_toplevel_dynamic_refs(pyfile):
    """모듈 top-level(함수/클래스 body 제외)에 동적 topic/path 참조 실행문이 있으면
    그 노드 설명 리스트 반환. 빈 리스트 = 깨끗(이동이 토픽그래프에 무영향).

    import / def / class / docstring(Expr-str) / __all__ 할당 / if __name__ 가드는 허용.
    """
    tree = ast.parse(Path(pyfile).read_text())
    offenders = []
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom,
                             ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            continue
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant):
            continue  # 모듈 docstring
        if isinstance(node, ast.Assign):
            # __all__ / 단순 상수 할당은 허용. __file__/__name__ 참조 잡기
            src = ast.dump(node)
            if '__file__' in src or "id='topic'" in src:
                offenders.append(f'L{node.lineno}: dynamic assign')
            continue
        if isinstance(node, ast.If):
            # if __name__ == '__main__': 가드 허용
            test_src = ast.dump(node.test)
            if '__name__' in test_src:
                continue
            offenders.append(f'L{node.lineno}: top-level if (non-__name__ guard)')
            continue
        # 그 외 모든 top-level 실행문 = 의심
        offenders.append(f'L{node.lineno}: {type(node).__name__}')
    return offenders
