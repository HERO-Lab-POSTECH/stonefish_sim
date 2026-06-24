# stonefish_control liveness 감사 (T2.0, P4 Phase 0.5)

> **목적**: "무엇이 살아있나(LIVE)" 전수판정. dead 운명결정(O1·O14)·trajectory liveness(O13)·LIVE 모듈 추출(T2.1) 대상의 **단일 근거**. P4 Phase 0.5 산출물.
> **측정 방식**: 3축 병렬 측정(정적 import 그래프 · launch/config 도달성 · 동적참조/out-of-tree/동명이클래스) 후 교차검증. **eager(pre-T0.7) 그래프에서 측정**해 baseline 동결(T0.7 lazy 전환이 측정을 오염하지 않게).
> **liveness 술어**: `LIVE = console_scripts 엔트리 → launch 파일 참조 → 동작하는 config 도달 가능`. launch에서 참조되지 않으면(엔트리만 존재) `catalogue-only`. import 자체가 깨진 경우 `dead(broken)`.
> 경로는 `/workspace/src/stonefish_sim/` 기준. 소스 패키지: `stonefish_control/stonefish_control/stonefish_control/`(control_interfaces·controllers·nodes), `stonefish_control/stonefish_trajectory_manager/stonefish_trajectory_manager/`(path_generator·path_following·nodes·common).

## Verdict 테이블

| module | static-importers | dynamic/out-of-tree | launch 도달? | verdict | O1-fate |
|---|---|---|---|---|---|
| `control_interfaces/dynamics_loader.py` | 1 (`__init__.py:19`); 심볼 `DynamicsLoader`는 live 노드 2개 + dead 노드 1개가 소비 | 0 | YES (hybrid+position 노드의 `from ..control_interfaces import DynamicsLoader`) | **LIVE** | KEEP — 유일 live control_interfaces 서브모듈 |
| `control_interfaces/data_types.py` | 3 (`position_controller.py:31`·`unified_controller.py:26`·`unified_controller_node.py:33`); `__init__` 미재export | 0 (테스트 2개 path-load) | YES (position_controller ← position_controller_node; hybrid→position) | **LIVE** | KEEP — angle_wrap SSOT(line 214) |
| `control_interfaces/vehicle.py` | 2 (`__init__.py:22` eager + dead `dp_controller_base.py:27`) | 0 | NO | **dead** | DELETE |
| `control_interfaces/dp_controller_base.py` | 2 (`__init__.py:23` eager + dead `dp_pid_controller_base.py:20`) | 0 | NO | **dead** | DELETE |
| `control_interfaces/dp_pid_controller_base.py` | 1 (`__init__.py:24` eager만) | 0 | NO | **dead** | DELETE — pure leaf |
| `controllers/position_controller.py` | 3 (`controllers/__init__.py:1`·`hybrid_controller.py:9`·`position_controller_node.py:36`) | 0 | YES (hybrid 체인) | **LIVE** | KEEP |
| `controllers/hybrid_controller.py` | 2 (`controllers/__init__.py:2`·`hybrid_controller_node.py:11`) | 0 | YES (`controller.launch.py:92-99` + `path_following.launch.py:109-116`) | **LIVE** | KEEP |
| `controllers/unified_controller.py` | 1 (dead `unified_controller_node.py:32`만) | 0 | NO | **dead** | DELETE — orphan 서브트리 |
| `controllers/unified_controller_node.py` | 0 | 0 | NO (console_scripts·launch 모두 없음) | **dead** | DELETE — orphan 노드 |
| `controllers/velocity_controller_node.py` | 0 | 0 | NO | **dead(broken)** — 존재하지 않는 `pid_4dof` import(L36) | DELETE — char test G2/G3/G4가 dead 동결 |
| `nodes/position_controller_node.py` | 0 (console_scripts 엔트리, `setup.py:26`) | 0 | **NO** (엔트리·yaml 존재하나 launch 참조 0건) | **catalogue-only** | KEEP (엔트리는 의도적 catalogue) — ★T1.1 크래시는 고치되 "LIVE 차단" 아님 |
| `nodes/hybrid_controller_node.py` | 0 (console_scripts 엔트리, `setup.py:25`) | 0 | YES (`controller.launch.py` + `path_following.launch.py`) | **LIVE** | KEEP — 도달성 루트 |
| `trajectory/nodes/path_following_node.py` | 1 (`nodes/__init__.py:2`) + 엔트리 | 0 | YES (`path_following.launch.py:99-106`) | **LIVE** | KEEP — 도달성 루트 |
| `trajectory/nodes/path_generator_node.py` | 1 (`nodes/__init__.py:1`) + 엔트리 | 0 | YES (`path_following.launch.py:84-96` + `path_generator.launch.py:112-125`) | **LIVE** | KEEP — 도달성 루트 |
| `trajectory/path_following/ilos_guidance.py` | 3 (`__init__.py:1`·`alos_guidance.py:44`·`path_following_node.py:42`) | 0 | YES (인스턴스화 `path_following_node.py:140`) | **LIVE** | KEEP |
| `trajectory/path_following/alos_guidance.py` | 2 (`__init__.py:3`·`path_following_node.py:42`) | 0 | YES (인스턴스화 `path_following_node.py:136`, 기본 guidance) | **LIVE** | KEEP |
| `trajectory/path_following/los_guidance.py` | 1 (`__init__.py:2`만) | 0 | NO (path_following_node는 ILOS/ALOS만) | **dead** | DELETE — ★계획에 없던 발견 |
| `trajectory/path_generator/path_generator.py` | 6 (`trajectory_manager/__init__.py:36`·`path_generator/__init__.py:35`·`cs_interpolator.py:27`·`linear_interpolator.py:25`·`lipb_interpolator.py:26`·`common/trajectory_generator.py:26`) | 0 | YES (`WPTrajectoryGenerator` ← path_generator_node) | **LIVE** | KEEP — T1.4 argmin in-scope |
| `trajectory/path_generator/cs_interpolator.py` | `path_generator/__init__.py` | `VelocityProfiler` 미해결 자유이름(L159), `use_velocity_profiler` config 키 미설정으로 guard | config 키 절대 안 켜짐 | **LIVE**(모듈) / **latent-dead**(VelocityProfiler 분기) | KEEP 모듈, VelocityProfiler 분기 DELETE |
| `trajectory/path_generator/lipb_interpolator.py` | `path_generator/__init__.py`(lipb=기본 interp) | `VelocityProfiler` 미해결(L257), 동일 guard | 동일 | **LIVE**(모듈) / **latent-dead**(VelocityProfiler 분기) | 동일 |
| **VelocityProfiler 클래스(심볼)** | `path_generator/__init__.py:49` `__all__`에 등재되나 **import/bind 안 됨**; repo 전체에 `class VelocityProfiler` 없음 | 0 (dangling `__all__` 이름) | NO (`create_trajectory_generator` utils.py:86 import 1회·호출 0회) | **dead(non-existent)** | DELETE dangling `__all__` + dead guard 분기 |

## 결론 (O1/O13/O14 입력)

### LIVE-extract set (control_interfaces 삭제 전 추출 필수)
`control_interfaces/dynamics_loader.py` + `control_interfaces/data_types.py`. 이 둘만 live control_interfaces 서브모듈. 패키지 `__init__.py` eager 블록을 lazy화(T0.7)해야 `vehicle.py`/`dp_controller_base.py`/`dp_pid_controller_base.py` 삭제가 `from ..control_interfaces import DynamicsLoader`(live 노드 2개 의존)를 깨지 않음. `data_types.py`는 테스트가 pin한 `angle_wrap` SSOT(L214) 보유.

### O13 verdict — path_following/path_generator = **LIVE** → T1.4 argmin **in-scope (frozen 아님)**
근거: 둘 다 실제 console_scripts 엔트리 + 동작하는 launch+config 연결. `path_following_node`=`path_following.launch.py:99-106` + `path_following.yaml`(`/**` wildcard라 항상 로드). `path_generator_node`=`path_following.launch.py:84-96`(+`path_generator.yaml`) AND `path_generator.launch.py:112-125`(inline params + `config/examples/krit_lawnmower.yaml`). 3축 일치.

### hybrid path = LIVE certify (Phase 1 전제 충족)
`hybrid_controller_node` = **유일하게 launch되는 컨트롤러**. `controller.launch.py:92-99` + `path_following.launch.py:109-116`. 전체 체인 `hybrid_controller_node → hybrid_controller.py → position_controller.py → data_types.py + control_interfaces.__init__(dynamics_loader)`. ⚠️ **H4 caveat**(liveness 실패 아님): launch가 런타임 노드명을 `hybrid_controller`로 덮어써 non-wildcard yaml 키 `hybrid_controller_4dof`와 불일치 → PID 게인이 조용히 `declare_parameter` 기본값으로 fallback. LIVE-but-config-broken(T1.2 수리 대상, LIVE verdict는 불변).

### ★position_controller_node = **catalogue-only/latent-dead (NOT LIVE)** — 계획 가정 반증
`grep position_controller *.launch.py` = EXIT 1 → **어떤 launch에서도 참조 안 됨**. 엔트리·소스·`position_controller.yaml`은 존재하나 never launched. → **T1.1 생성자 크래시는 진짜이고 고쳐야 하나**(setup.py 등록 엔트리가 깨진 채면 안 됨), 분류는 "유일 LIVE 경로 차단"이 아니라 **"catalogue 엔트리의 기동불가"**. 긴급도 LIVE→catalogue-only 정정.

### delete set (O1=삭제, owner 확정 — LIVE-extract 후 안전)
- `control_interfaces/vehicle.py` · `dp_controller_base.py` · `dp_pid_controller_base.py` (orphan legacy 체인)
- `controllers/unified_controller.py` · `unified_controller_node.py` (orphan unified 서브트리)
- `controllers/velocity_controller_node.py` (broken — `pid_4dof` 부재)
- `trajectory/path_following/los_guidance.py` (★계획에 없던 발견 — path_following_node는 ILOS/ALOS만)
- **VelocityProfiler dead 코드**(독립 파일 없음): `path_generator/__init__.py:49` dangling `__all__` 엔트리 + `cs_interpolator.py:158-159`·`lipb_interpolator.py:256-257` dead `if self._use_velocity_profiler:` 분기 제거.

**삭제 동반 편집**(삭제 아니나 동반 필수): `control_interfaces/__init__.py` lazy화(T0.7, L22-24 제거), `path_following/__init__.py:2`에서 `los_guidance` 제거. `position_controller_node.py`는 delete set 아님(console_scripts 엔트리는 의도적 catalogue) — KEEP.

### 3축 충돌
없음 — 모든 모듈 verdict에서 3축 일치. 헤드카운트 정정(`path_generator.py` distinct importer=6), `linear_interpolator.py`·`common/trajectory_generator.py`·`common/__init__`·`nodes/utils.py`는 엣지로만 측정됨(LIVE-by-inference, 명시적 단독 측정 아님 — 부재≠dead).
