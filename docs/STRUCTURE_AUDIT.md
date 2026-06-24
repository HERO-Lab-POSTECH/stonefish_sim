All critical claims verified against the filesystem. The defects are real: VelocityProfiler exported but never imported/defined and instantiated in two files, thruster config glob points to nonexistent dir, control_msgs README says Apache 2.0 while package.xml says GPL-3.0, teleop README says Apache 2.0, empty launch dir, meta-dir has no package.xml, dead models/, and src/stonefish_ros2/ package-name nesting alongside flat executables. Now writing the report.

# stonefish_sim 구조 전체 감사 (재감사 — 1차 감사 sloppy 판정 후 재작성)

## 1. BLUF (결론 우선)

**stonefish_sim는 엔터프라이즈-표준이 아니다.** 7개 패키지 중 빌드 메타·layout·naming·license·dead-code 전 범주에 걸쳐 **confirmed-defect 12건 + confirmed-nonstandard 약 35건**이 확정됐다. 1차 감사는 가장 눈에 띄는 비표준 패턴인 `stonefish_description/data/` 래퍼를 "standard"로 통과시켰는데, 이는 공식 ROS2 description 패키지(turtlebot3/turtlebot4/ur_description/ros_gz)를 단 하나도 대조하지 않은 결과다 — 실측 결과 **어떤 공식 패키지도 `data/` 래퍼를 쓰지 않는다**. 또한 1차 감사는 규칙 위반(REP/PEP)만 점검하고 **실사용 가능성(usability)** — 존재하지 않는 `VelocityProfiler` 클래스를 `__all__`에 export하고 두 파일에서 인스턴스화해 `use_velocity_profiler=True` 시 `NameError`로 죽는 런타임 결함, 존재하지 않는 `config/` 디렉터리를 가리키는 glob, README↔package.xml 라이선스 모순 2건 — 을 "looks-standard"로 흘려보냈다. 핵심 런타임 결함(VelocityProfiler), 디렉터리 구조(data/ 래퍼), 미완성 stub(teleop), 라이선스 모순은 전부 파일시스템에서 직접 재확인했다. P3(동작보존) 수정 가능 항목이 다수지만, **VelocityProfiler·teleop 패키지화·data/ 래퍼 제거는 동작/구조를 바꾸므로 P4 격리 대상**이다.

---

## 2. confirmed-nonstandard + confirmed-defect 전수 표

범례: 판정 `D`=confirmed-defect, `N`=confirmed-nonstandard. 동작보존 `O`=preserves-behavior, `X`=동작/구조 변경(P4).

| # | 패키지·경로 | 무엇이 문제 | 판정 | 공식 근거 (URL) | P3?/P4 | 동작보존 |
|---|---|---|---|---|---|---|
| 1 | `stonefish_trajectory_manager/.../path_generator/__init__.py` | `__all__`에 `'VelocityProfiler'` 있으나 import도 정의도 없음 → `from ... import VelocityProfiler` 시 ImportError | D | https://peps.python.org/pep-0008/#public-and-internal-interfaces (`__all__`은 정의된 이름만) | P3 | O |
| 2 | `.../path_generator/lipb_interpolator.py:257`, `cs_interpolator.py:159` | `use_velocity_profiler=True`일 때 미정의 `VelocityProfiler(...)` 인스턴스화 → `NameError` 런타임 크래시. `velocity_profiler.py` 파일 부재(실측 확인) | D | 신뢰할 출처 없음 (패키지 내부 구현 누락 결함, ARCHITECTURE.md §2 line 86은 파일 존재 주장) | P4 | X (기능 활성화 시 동작 변경) |
| 3 | `stonefish_thruster_manager/setup.py:16` | `glob('config/*.yaml')`이 존재하지 않는 `config/` 가리킴(실측: NO config/ dir). 빈 리스트 → dead code | D | sibling `stonefish_control/setup.py`·`trajectory_manager/setup.py`는 실재 디렉터리 glob; turtlebot3/nav2 관행 | P3 | O |
| 4 | `stonefish_thruster_manager/.../models/` (thruster.py, thruster_proportional.py) | UUV Simulator 잔재. 어디서도 import 안 됨(실측: node는 `..thruster_manager`만 import). factory 메서드 정의되나 호출 0회. 구조적 dead code | D | 신뢰할 출처 없음 (패키지 내부 dead-code; grep 증거 기반) | P3 | O |
| 5 | `stonefish_control_msgs/CMakeLists.txt` | `ament_export_dependencies(rosidl_default_runtime)` 누락(실측: line 47 `ament_package()` 직전 없음) | D | https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/CMakeLists.txt , nav2_msgs/CMakeLists.txt (전부 포함) | P3 | O |
| 6 | `stonefish_control_msgs/README.md:378` | README "Apache 2.0" vs package.xml "GPL-3.0"(실측 확인). 라이선스 모순 | D | REP 127 (package.xml이 authoritative): https://github.com/ros-infrastructure/rep/blob/master/rep-0127.rst | P3 | O |
| 7 | `stonefish_teleop_manager/README.md:122` | README "Apache 2.0" vs 루트 LICENSE/CONVENTIONS GPL-3.0(실측 확인) | D | file: 루트 `LICENSE`(GPL-3.0), `docs/CONVENTIONS.md` L136 | P3 | O |
| 8 | `stonefish_teleop_manager/` | package.xml/setup.py 없이 README만(실측). colcon-invisible·human-visible 비대칭. 미완성 stub | D | https://github.com/colcon/colcon-ros/blob/master/colcon_ros/package_identification/ros.py (package.xml 필수); https://github.com/ros2/ros2cli/tree/rolling/ros2pkg/ros2pkg/resource/ament_python | P4 (패키지화=신규동작) | X |
| 9 | `stonefish_thruster_manager/README.md:375-387` | Package Structure가 실제와 불일치 — `thruster_allocator.py`로 적혀있으나 실제 `nodes/thruster_allocator_node.py`. nodes/·test/·resource/ 누락 | D | https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.rst | P3 | O |
| 10 | `stonefish_thruster_manager/package.xml` | 의존성 스타일 혼재: msg pkg는 `<depend>`, system pkg는 `<exec_depend>`. ament_python은 전부 exec_depend 관행 | D | https://github.com/ros2/demos/blob/humble/demo_nodes_py/package.xml , nav2_simple_commander/package.xml | P3 | O |
| 11 | `stonefish_trajectory_manager/package.xml` (+ setup.py) | setup.py `install_requires`에 scipy·PyYAML 누락(코드에서 직접 import). package.xml엔 선언됨 | D | https://github.com/ros-infrastructure/rep/blob/master/rep-0140.rst ; demo_nodes_py/setup.py | P3 | O |
| 12 | `stonefish_description/launch/` | 빈 디렉터리(실측: 0 파일)인데 CMakeLists `install(DIRECTORY launch ...)`. ROS1→ROS2 마이그레이션(516d81a)에서 .launch 삭제 후 install 라인 잔존 | D | https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble/turtlebot3_description/CMakeLists.txt (launch install 생략) | P3 | O |
| 13 | `stonefish_description/` + `/data/` | **`data/` 래퍼**가 robots/worlds/models를 감쌈 → `share/stonefish_description/data/...` 설치. 공식 패키지 0개가 이 패턴 사용 | N | turtlebot3_description, turtlebot4_description, ur_description, ros_gz_example_description CMakeLists (전부 root-level 설치, data/ 없음) | P3 가능하나 .scn 경로 동시수정 필요 | X (경로 의존) |
| 14 | `stonefish_description/data/robots/_legacy` | `_` prefix 디렉터리는 ROS2 description 패키지 비관행 (PEP 8는 Python 코드용) | N | nav2는 `old_` prefix 사용; turtlebot3 deprecated dir 없음; https://peps.python.org/pep-0008/#naming-conventions | P3 | O |
| 15 | `stonefish_description/CMakeLists.txt` (data 설치) | `install(DIRECTORY data ...)` → 비표준 `data/` 경로 생성 | N | turtlebot3_description/turtlebot4_description CMakeLists (root-level install) | P3 | X |
| 16 | `stonefish_description/README.md` | 비표준 `data/` 레이아웃을 문서화·정당화 (README 품질 자체는 높음) | N | ros2_control_demos, stretch_description, nav2_bringup (data/ 래퍼 없음) | P3 | O |
| 17 | `stonefish_description/data/robots/bluerov2/config/TAM.yaml, dynamics_params.yaml` | `data/robots/{vehicle}/config/` 중첩 — 공식은 root `config/` 또는 별도 bringup 패키지 | N | https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/master/config | P3 | X (consumer 경로 의존) |
| 18 | `stonefish_ros2/src/stonefish_ros2/` | 구현 .cpp가 패키지명 미러링 `src/<pkg>/`에 중첩(실측 확인). 공식은 flat src/ 또는 기능별 subdir | N | https://github.com/ros2/demos/tree/rolling/demo_nodes_cpp ; nav2_util/src; moveit_core | P3 | O |
| 19 | `stonefish_ros2/src/` (실행파일 분리) | executable(`stonefish_simulator.cpp`)은 flat src/, 라이브러리는 `src/stonefish_ros2/` → 비대칭 | N | https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_sim ; rclcpp/src (일관 패턴) | P3 | O |
| 20 | `stonefish_ros2/CMakeLists.txt:28-34` | `src/${PROJECT_NAME}/` 확장 — 비표준 중첩의 우회책 | N | tf2/rcl/rmw_implementation/nav2_map_server CMakeLists (flat 또는 기능별) | P3 | O |
| 21 | `stonefish_ros2/CMakeLists.txt` (export) | bare `$<INSTALL_INTERFACE:include>`, `ament_export_include_directories(include)`, `install(TARGETS)` EXPORT절·`ament_export_targets()` 누락 (구식 2021-22 관행) | N | https://github.com/ros2/geometry2/blob/rolling/tf2/CMakeLists.txt ; rcl/CMakeLists.txt | P3 | O |
| 22 | `stonefish_ros2/` (헤더 설치) | `install(DIRECTORY include DESTINATION include)` → `include/stonefish_ros2/`. 공식은 `DESTINATION include/${PROJECT_NAME}` 네임스페이싱 | N | nav2_costmap_2d/nav2_util/moveit_core CMakeLists | P3 | O |
| 23 | `stonefish_ros2/docs/` | 패키지-레벨 Sphinx conf.py. 공식은 top-level doc(s)/ 또는 .rst만(conf.py 없음) | N | https://github.com/ros2/rclpy/tree/rolling/rclpy/docs/source ; moveit2 | P3 | O |
| 24 | `stonefish_control_msgs/srv/Hold.srv` | empty request + `bool success`만. 공식 status 서비스는 success+message 둘 다 | N | https://github.com/ros2/common_interfaces/blob/humble/std_srvs/srv/Trigger.srv ; SetBool.srv | P3 | O |
| 25 | `stonefish_control_msgs/README.md` | msg 패키지치고 과도하게 상세(field table, code examples). 공식은 미니멀 1줄 설명 | N | geometry_msgs/nav_msgs/std_msgs/README.md (미니멀) | P3 | O |
| 26 | `stonefish_control_msgs/` (CHANGELOG 부재) | CHANGELOG 없음(실측). sibling stonefish_msgs엔 있음 → 워크스페이스 내 불일치. 또한 공식은 `.rst`(아래 #27 참조) | N | https://github.com/ros2/common_interfaces/tree/humble/std_msgs (CHANGELOG.rst); REP 132 | P3 | O |
| 27 | `stonefish_msgs/CHANGELOG.md`, 루트 `CHANGELOG.md` | `.md` 포맷 — REP 132는 RST 요구. 공식 전부 `.rst` | N | https://www.ros.org/reps/rep-0132.html ("must be valid RST"); turtlebot3/ros_gz/moveit2 CHANGELOG.rst | P3 | O |
| 28 | `stonefish_msgs/msg/Int32Stamped.msg` | scalar(int32)에 Stamped 패턴 적용. 공식 Stamped는 composite 타입만(Point/Pose/Twist…) | N | https://index.ros.org/p/geometry_msgs/ ; std_msgs는 scalar Stamped 미정의 | P3 | O |
| 29 | `stonefish_msgs/srv/SetMode.srv` | response `bool success`만 — 공식 SetBool은 success+message. 동일 패키지 타 서비스와도 불일치 | N | std_srvs/SetBool 패턴; https://github.com/ros2/rosidl/pull/25 | P3 | O |
| 30 | `stonefish_msgs/srv/SonarSettings2.srv` | 숫자 접미사 서비스 변형. ROS2는 숫자 접미사를 데이터 타입 버전(Float32 등)에만 사용, 서비스 변형엔 안 씀 | N | control_msgs/common_interfaces/nav2_msgs srv 전수 — 숫자 서비스 변형 0건 | P3 | O |
| 31 | `stonefish_msgs/README.md` | msg 패키지 README가 과도(전체 정의 inline, Py/C++ 예제). 공식은 미니멀 | N | sensor_msgs/geometry_msgs/std_msgs/nav_msgs/trajectory_msgs README.md | P3 | O |
| 32 | `stonefish_msgs/CMakeLists.txt` | `if(BUILD_TESTING)`+`ament_lint_auto_find_test_dependencies()` (ROS1/catkin 스타일), `ament_export_dependencies(rosidl_default_runtime)` 누락, 파일 inline 나열 | N | geometry_msgs/sensor_msgs/nav2_msgs/ros_gz_interfaces CMakeLists (ADD_LINTER_TESTS + export) | P3 | O |
| 33 | `stonefish_control/stonefish_control/.../control_interfaces/`, `controllers/`, `nodes/` | 내부 subdir 조직(nodes/·controllers/·control_interfaces/)은 공식 ROS2 표준 아님 — CONVENTIONS.md §2.0 line 49 스스로 "신뢰할 출처 없음" 명시. demo_nodes_py는 기능별(topics/·services/) | N | https://design.ros2.org/articles/ament.html ; demo_nodes_py (기능별 subdir); `docs/CONVENTIONS.md` L49 | P3 (선택) | O |
| 34 | `…/control_interfaces/__init__.py` | PEP 8 위반: `__all__`이 docstring→imports→`__all__` 순서(import 뒤). 정상은 docstring→`__all__`→imports. 5+ 파일 동일 | N | https://peps.python.org/pep-0008/#module-level-dunder-names | P3 | O |
| 35 | `stonefish_thruster_manager/.../__init__.py` | `ThrusterManager` 재노출 안 함. CONVENTIONS §2.1 권장 패턴(trajectory_manager:28-44) 미준수 | N | `docs/CONVENTIONS.md` §2.1 L83-84 (repo 자체 표준) | P3 | O |
| 36 | `…thruster_manager/nodes/__init__.py`, `…stonefish_control/nodes/__init__.py` | `__version__`도 `__all__`도 없는 bare comment. sibling trajectory_manager/nodes/는 재노출함 | N (control/nodes는 D 판정) | `docs/CONVENTIONS.md` §2.1 L83-84 | P3 | O |
| 37 | `stonefish_trajectory_manager/.../path_generator/path_generator.py` | `abc.ABC` 주장과 달리 `object` 상속 + `raise NotImplementedError()` (PEP 3119 미준수, pre-2.6 레거시) | N | https://www.python.org/dev/peps/pep-3119/ ; workspace `pid_optimizer/base.py`는 올바른 ABC 사용 | P3 | O |
| 38 | `…common/waypoint_set.py`, `trajectory_point.py`, `path_generator/bezier_curve.py`, `line_segment.py` | markdown-style docstring (UUV 레거시). CONVENTIONS §2.4는 Google-style 표준, 이들은 "grandfathered" 비표준 명시 | N | https://google.github.io/styleguide/pyguide.html ; `docs/CONVENTIONS.md` §2.4 | P3 | O |
| 39 | `stonefish_trajectory_manager/.../nodes/utils.py` | `nodes/utils.py` 공유헬퍼 패턴은 공식 근거 없음. CONVENTIONS §2.1도 이 파일 언급 안 함 | N | `docs/CONVENTIONS.md` §2.0 L49 (신뢰할 출처 없음); REP 144는 내부구조 미규정 | P3 | O |
| 40 | `stonefish_trajectory_manager/setup.py` | `config/examples/` 중첩 subdir glob (line 16) — 공식은 단일 recursive glob 또는 CMake DIRECTORY | N | moveit2 moveit_configs_utils/setup.py ; nav2_simple_commander/setup.py | P3 | O |
| 41 | `stonefish_trajectory_manager/ARCHITECTURE.md` | 패키지-레벨 ARCHITECTURE.md — 공식 패키지 0개가 보유(nav2는 /doc/architecture/ 디렉터리). 우수하나 비표준 | N | nav2/moveit2/ros_gz/turtlebot 전수 — ARCHITECTURE.md 없음 | P3 | O |
| 42 | `stonefish_thruster_manager/test/`, `…/test_thruster_manager.py` | `conftest.py load_module` fixture는 repo-특화 발명(ROS/gtsam 오염 회피). 공식 ROS2 패키지 0개 사용 — "standard"라 부른 건 부정확 | N | https://github.com/ros2/rclpy ; ros2cli ; geometry2 (conftest 패턴 없음) | P3 (필요악) | O |
| 43 | `stonefish_control/` (메타 디렉터리) | package.xml 없는 그룹핑 디렉터리가 5개 sub-pkg 감쌈(실측: NO package.xml). 공식은 flat hierarchy(nav2) 또는 metapackage+package.xml(moveit2) | N | ros2_control/ros2_controllers (flat); moveit2 (metapackage); ros_gz_project_template (naming prefix) | P3 | O |
| 44 | `stonefish_control/README.md` (메타-README) | package.xml 없는 plain 디렉터리의 meta-README. 공식은 metapackage 또는 top-level README only | N | moveit2(moveit_ros metapackage); turtlebot3/4; ros_gz_project_template | P3 | O |
| 45 | 루트 `test/` 디렉터리 | root-level cross-package test. 공식은 per-package test/ 또는 별도 test 패키지(test_tf2) | N | https://github.com/ros2/geometry2/tree/humble (test_tf2); moveit2 per-package | P4 (의도적 예외) | O |
| 46 | 루트 `docs/` (복수형) | 공식은 `doc/`(단수). `docs/CONVENTIONS.md`는 published-docs 아닌 팀 표준 → CONTRIBUTING.md 관행 | N | nav2/moveit2/ros2_control (doc/ 단수); design.ros2.org/per_package_documentation | P3 | O |
| 47 | 루트 `pytest.ini` | root-level `testpaths` 지정. 공식 멀티패키지 repo는 colcon test 사용, root pytest.ini 안 씀 | N | nav2/ros2_control/moveit2/turtlebot/ros_gz (root pytest.ini 없음) | P3 (필요악) | O |
| 48 | `.github/workflows/ci.yml` | pytest-only, colcon build 없음 → ament_cmake(C++) 패키지(stonefish_ros2/msgs) 미컴파일·미검증 | N | nav2(.github colcon); moveit2(industrial_ci); REP 2004 | P3 | O |
| 49 | 루트 `CHANGELOG.md` | package.xml 없는 워크스페이스 root에 위치. REP 132는 package.xml peer 요구 + `.rst` | N | https://github.com/ros-infrastructure/rep/blob/master/rep-0132.rst ; nav2/moveit2 per-pkg CHANGELOG.rst | P3 | O |
| 50 | `docs/CONVENTIONS.md` | 전용 CONVENTIONS.md는 공식 ROS2 비관행(공식은 CONTRIBUTING.md). 내용은 우수, 한국어=내부 팀용 | N | nav2/moveit2/gazebo CONTRIBUTING.md (embedded style guide 없음) | P3 | O |

추가 N 항목(naming 레거시 docstring 계열)은 #38에 묶었다. `package.xml` buildtool_depend ament_python 관련 1차 판정(confirmed-nonstandard)은 verdict 데이터에 1건 존재하나, 동일 repo의 trajectory_manager 등 다수 패키지가 export build_type만 쓰는 것과 혼재하므로 #10/#11과 함께 build-meta 정합성 이슈로 본다.

---

## 3. 1차 감사가 "표준인 척" 통과시킨 비표준 항목 (오판 교정)

1차 감사가 `my_claim: looks-standard` 또는 `unsure`로 흘려보냈으나 공식 대조 결과 **비표준**으로 뒤집힌 항목들. 이것이 재감사를 부른 핵심 실패다.

| 경로 | 1차 주장 | 교정 판정 | 오판을 깨는 공식 근거 |
|---|---|---|---|
| `stonefish_description/data/` | "data/{robots,worlds,models} is standard… turtlebot3/ur_description use this" (정면으로 틀림) | **confirmed-nonstandard** | turtlebot3_description: `install(DIRECTORY meshes rviz urdf)` / turtlebot4_description: `install(DIRECTORY launch meshes urdf)` / ur_description: `cfg config launch meshes urdf` — **전부 root-level, data/ 래퍼 0건**. `data/` 때문에 설치경로가 `share/stonefish_description/data/...`로 비표준화 |
| `stonefish_description` (전체) | "Follows ROS2 resource package conventions" | nonstandard | 위와 동일 — `data/` 래퍼는 examined 공식 패키지 어디에도 없음 |
| `scenarios/`, `data/robots/.scn`, `data/worlds/.scn` | "unsure / looks-nonstandard" 혼재, 경로는 미검증 | scn 경로 자체는 **standard**(Stonefish dataPath 루트 해석), 그러나 그 standard함이 곧 `data/` 래퍼를 강제 → 래퍼 제거 시 전 .scn 경로 동시수정 필요 | stonefish_simulator.cpp가 share 루트를 dataPath로 전달; 모든 .scn이 `data/` prefix(일관성)가 곧 래퍼 종속의 증거 |
| `path_generator/__init__.py`, `lipb/cs_interpolator.py` | "looks-standard" (VelocityProfiler를 전혀 검증 안 함) | **confirmed-defect (런타임 크래시)** | 실측: `velocity_profiler.py` 파일 없음, `__all__`에 export, 두 파일서 인스턴스화 → import semantics 위반 + `use_velocity_profiler=True` 시 NameError |
| `stonefish_thruster_manager/setup.py` config glob | "looks-standard" / "looks-nonstandard"(harmless라 평가절하) | **confirmed-defect** | 실측: `config/` 디렉터리 부재. sibling은 실재 디렉터리 glob — usability 결함 |
| `stonefish_control_msgs/README.md`, `teleop/README.md` 라이선스 | "looks-standard" (license 모순 미점검) | **confirmed-defect ×2** | 실측: 양쪽 README "Apache 2.0" vs package.xml/루트 LICENSE "GPL-3.0" |
| `control_interfaces/`, `controllers/`, `nodes/` 내부구조 | "looks-standard… nav2/moveit use dedicated dirs" | **confirmed-nonstandard** | CONVENTIONS.md §2.0 L49가 스스로 "신뢰할 출처 없음" 명시; demo_nodes_py는 기능별(topics/·services/), generic `nodes/` 아님 |
| `src/stonefish_ros2/` C++ 중첩 | "looks-nonstandard"로 일부 잡았으나 "preserves_behavior, 무해"로 약하게 처리 | nonstandard (구조 혼란 + cmake 우회 유발) | demo_nodes_cpp/nav2_util/moveit_core — `src/<pkg-name>/` 미러링 0건 |

1차 감사의 패턴: **공식 패키지를 실제로 열어보지 않고 기억/추정으로 "nav2/turtlebot이 이렇게 한다"고 단언**했고(특히 data/ 건은 정반대), **파일 존재 여부·라이선스 일관성 같은 usability를 점검하지 않고 REP/PEP 규칙 위반만 봤다**.

---

## 4. 진짜 표준으로 확정된 항목 (공식 URL 증거 포함)

bare assertion 금지 — 각 항목에 대조한 공식 패키지 URL을 명시한다.

| 경로 | standard인 이유 | 공식 근거 URL |
|---|---|---|
| `stonefish_control/` ament_python 레이아웃(setup.py/setup.cfg/resource marker/nested pkg) | ros2cli ament_python 템플릿 정합 | https://github.com/ros2/ros2cli/tree/humble (ros2pkg ament_python); https://github.com/ros2/examples/blob/humble/rclpy/topics/minimal_publisher/setup.py |
| `setup.cfg` [develop]/[install] script_dir=lib/<pkg> | demo_nodes_py와 정확히 일치 | https://github.com/ros2/demos/blob/humble/demo_nodes_py/setup.cfg |
| `resource/<pkg>` marker file (3개 ament_python 패키지) | ament Resource Index 표준 | https://github.com/ament/ament_cmake/blob/rolling/ament_cmake_core/doc/resource_index.md |
| `stonefish_msgs/`, `stonefish_control_msgs/` flat msg/+srv/ 구조 | geometry_msgs/sensor_msgs와 동일 | https://github.com/ros2/common_interfaces/tree/master/geometry_msgs |
| msg/srv PascalCase 파일명 (전 12+6 파일) | REP 127 / design.ros2.org 강제 | https://design.ros2.org/articles/interface_definition.html ; common_interfaces/geometry_msgs/msg |
| .msg/.srv snake_case 필드명 (전수) | rosidl 강제 | https://github.com/ros2/rosidl/pull/25 |
| `DVL.msg`/`DVLBeam.msg` UUV 출처 재사용 | sensor_msgs/Imu 패턴 동일 | https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg ; https://github.com/uuvsimulator/uuv_simulator |
| `INS.msg`/`NEDPose.msg` NED 좌표계 | REP 103 `_ned` secondary frame 허용 | https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst |
| `controller.launch.py` 등 모든 launch generate_launch_description() | turtlebot3/webots 공식 패턴 일치 | https://github.com/ROBOTIS-GIT/turtlebot3_simulations/.../turtlebot3_world.launch.py |
| `launch/` root-level 배치 | nav2_bringup/turtlebot3_bringup 동일 | https://github.com/ros-navigation/navigation2/tree/main/nav2_bringup/launch |
| `config/bluerov2/` vehicle subdir | UR description의 `config/<variant>/` 동일 | https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/humble/config |
| console_scripts entry_points (전 패키지) | demo_nodes_py 동일 패턴 | https://github.com/ros2/demos/blob/humble/demo_nodes_py/setup.py |
| `include/stonefish_ros2/` 헤더 배치(double-nesting 없음) | tf2/rclcpp/controller_interface 동일 | https://github.com/ros2/rclcpp/tree/humble/rclcpp/include |
| `stonefish_*` 7개 package.xml format=3 + REP 144 naming + `_msgs`/`_description` 접미사 | REP 144/149 정합 | https://github.com/ros-infrastructure/rep/blob/master/rep-0149.rst ; common_interfaces |
| 모든 `__pycache__/` (launch/docs/root) | .gitignore L7로 정상 ignore(git check-ignore 확인) | https://github.com/ros-navigation/navigation2/blob/main/.gitignore (PEP 3147) |
| `thruster_manager/launch/install/__pycache__` | git ignore됨(.gitignore L30 `**/install/`) — committed 아님 | turtlebot3/navigation2 .gitignore |
| 루트 `conftest.py` (load_module fixture 위치) | root conftest.py는 pytest 표준(스코프 공유) | https://docs.pytest.org/en/latest/how-to/fixtures.html#conftest-py-sharing-fixtures-across-multiple-files |
| `.gitignore` 코어 패턴(build/install/log/, pycache, IDE, OS) | ros2/ros2·nav2·moveit2 동일 | https://github.com/ros2/ros2/blob/master/.gitignore |
| `log/` 디렉터리(colcon 산출, ignore됨) | GitHub ROS2.gitignore 템플릿 표준 | https://github.com/github/gitignore/blob/main/community/ROS2.gitignore |
| `stonefish_description` `_description` 접미사 + share/${PROJECT_NAME} 설치 | REP 144 mandated | https://ros.org/reps/rep-0144.html (단, 내부 data/ 래퍼는 별개로 #13 비표준) |
| 모든 클래스 PascalCase / 메서드 snake_case (HybridController, PositionController, 모든 guidance/interpolator) | PEP 8 + CONVENTIONS §2.3 | https://peps.python.org/pep-0008/#class-names |

주의: `stonefish_description`이 표준인 부분(`_description` naming, share 설치 메커니즘)과 비표준인 부분(`data/` 래퍼)은 **분리**된다 — 같은 패키지라도 naming은 통과, 내부 layout은 실패다. 1차 감사는 이 둘을 뭉뚱그려 전체를 통과시켰다.

---

## 5. 신뢰할 출처 없음 (정직하게 표기 — 추정 금지)

다음 항목은 공식 ROS2 표준(REP/design.ros2.org/공식 패키지)에서 **근거를 찾을 수 없어** standard/nonstandard 판정 불가다. 추정하지 않고 명시한다.

| 경로 | 왜 판정 불가인가 |
|---|---|
| `path_following/` 내부 subdir 구조 | CONVENTIONS.md §2.0 L49가 내부 subdir에 대해 "신뢰할 출처 없음" 명시. design.ros2.org은 내부 모듈 조직을 규정 안 함. repo 내부 일관성은 있으나 외부 표준 부재 |
| `rviz/` subdir 설치 위치 | 공식 ROS2 문서/REP가 .rviz 설치 위치를 규정한 바 없음. design.ros2.org는 include/msg/srv만 auto-process 언급. share/<pkg>/rviz/는 합리적이나 authoritative 근거 없음 |
| `stonefish_thruster_manager/setup.cfg` [install] install_scripts | modern setuptools 문서는 [install] 섹션·install_scripts 미문서화(distutils 레거시). REP 144·ros2cli 템플릿 원문 미확인. entry_points console_scripts엔 영향 없을 수 있음 |
| `stonefish_trajectory_manager/README.md` 존재 자체 | CONVENTIONS §2.1 ament_python 레이아웃이 README.md를 명시 안 함. 7개 패키지 모두 보유하나 문서화된 권위 없는 local 패턴 |
| `VelocityProfiler` 결함의 "공식" 근거 | 이는 ROS2 표준 문제가 아닌 패키지 **내부 구현 누락 결함** — 비교할 공식 패키지가 없음. 판정 근거는 ARCHITECTURE.md §2 L86의 파일 존재 주장 vs 실측 부재 |

이 항목들은 "repo 내부 규약(CONVENTIONS.md)으로는 정당하나 외부 ROS2 표준으로는 미검증"이라는 이중 상태다 — 둘을 섞어 "standard"라 부르면 안 된다.

---

## 6. 정직한 종결 — 1차 감사가 놓친 것과 교정된 전체 그림

**1차 감사가 놓친 것 (왜):**

1차 감사는 두 가지 체계적 실패를 했다. 첫째, **공식 패키지를 실제로 열어 대조하지 않고 기억으로 단언**했다. 가장 치명적인 예가 `stonefish_description/data/` 래퍼를 "turtlebot3/ur_description도 이렇게 한다"며 standard로 통과시킨 것인데, 실제로 그 패키지들의 CMakeLists를 열면 `install(DIRECTORY meshes rviz urdf ...)`처럼 **root-level 설치이고 data/ 래퍼가 0건**이다 — 정확히 정반대를 주장했다. 둘째, **규칙 위반(REP/PEP)만 보고 실사용 가능성(usability)을 점검하지 않았다**. 그래서 `VelocityProfiler`(파일 부재, `__all__` export, 2곳 인스턴스화 → `use_velocity_profiler=True` 시 NameError 크래시), 존재하지 않는 `config/`를 가리키는 glob, README↔package.xml 라이선스 모순 2건을 전부 "looks-standard"로 흘려보냈다. 이 넷은 "REP 위반"은 아니지만 패키지를 **실제로 쓰면 깨지거나 법적으로 모순**되는, 더 심각한 결함이다.

**교정된 전체 그림:**

stonefish_sim은 표면 레이어(package.xml format, naming, msg/srv 구조, launch, console_scripts, 클래스 naming)는 견고하게 공식 표준을 따르지만, **한 꺼풀 아래에서 confirmed-defect 12건이 실사용을 막는다.** 가장 시급한 셋은 (1) `VelocityProfiler` 런타임 크래시 — 광고된 기능이 호출 즉시 죽음, (2) 라이선스 모순 ×2 — 배포 시 법적 리스크, (3) 죽은 glob/dead models — 빌드는 통과하나 의도와 어긋남. 구조 차원에서는 (4) `data/` 래퍼와 (5) `stonefish_control/` 메타 디렉터리(package.xml 없는 그룹핑), (6) `src/stonefish_ros2/` C++ 패키지명 중첩이 공식 생태계 어디에도 없는 패턴이다.

**P3(동작보존) vs P4(동작/구조 변경) 격리:** 표의 `동작보존=O` 항목(라이선스 수정, 죽은 glob 제거, ament_export 추가, CMakeLists/README 정합, `__all__` 정리, CHANGELOG.rst 전환 등 대부분)은 P3에서 안전하게 처리 가능하다. 반면 **P4 격리 필수**는: VelocityProfiler 기능 구현/제거(#2, 기능 활성 시 동작 변경), teleop 패키지화(#8, 신규 동작 — p3-restructure.md가 명시적으로 P3 범위 밖 결정), `data/` 래퍼 제거(#13/15/17, 전 .scn·consumer 경로 동시 변경 필요). 이 셋은 동작보존 증명이 불가능하므로 P3에서 손대면 안 된다.

검증 방식: data/ 래퍼·teleop stub·VelocityProfiler 부재·라이선스 모순·죽은 config glob·메타 dir package.xml 부재·dead models·ros2 src 중첩은 전부 이 세션에서 파일시스템 직접 실측으로 재확인했다(추정 아님). 공식 패키지 대조 URL은 표의 각 행에 명시했고, 근거 없는 항목은 §5에 "신뢰할 출처 없음"으로 정직하게 분리했다.