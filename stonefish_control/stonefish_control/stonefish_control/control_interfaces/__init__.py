# Copyright (c) 2016-2019 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Control interfaces for trajectory tracking and dynamic positioning."""

import importlib

# 4DOF lightweight dynamics loader (for underactuated vehicles).
# LIVE: imported by hybrid_controller_node / position_controller_node.
# rclpy/nav_msgs 비의존 leaf라 eager import 안전.
from .dynamics_loader import DynamicsLoader

# 6DOF full dynamics and controller bases (LEGACY, dead — 상속자 0).
# 이들은 vehicle.py(nav_msgs hard-import)·dp_controller_base.py(rclpy hard-import)를
# 끌어와 import-time에 ROS 의존성을 강제한다. 그러나 LIVE 노드는 이 패키지에서
# DynamicsLoader 하나만 import하므로, dead 3클래스를 eager로 끌면 그 LIVE import가
# ROS 부재 환경에서 깨진다(T0.7). PEP 562 __getattr__로 lazy 노출해 import-time
# 부작용을 끊는다 — 심볼명(__all__)·동작은 보존, 실제 접근 시에만 로드.
#
# ⚠️ TEMPORARY BRIDGE (O1=삭제 확정, P4 T2.2서 제거): 이 lazy 스캐폴드는 dead 3클래스
# 본체와 함께 T2.2-삭제에서 통째로 제거될 임시 구조다(영구 구조 아님). dead-code slop이
# 아니라 의도된 단명 브릿지. 근거: docs/LIVENESS_AUDIT.md (delete set).
_LAZY_LEGACY = {
    'Vehicle': '.vehicle',
    'DPControllerBase': '.dp_controller_base',
    'DPPIDControllerBase': '.dp_pid_controller_base',
}


def __getattr__(name):
    """PEP 562 lazy attribute — dead legacy 클래스를 실제 접근 시에만 로드."""
    module_name = _LAZY_LEGACY.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = importlib.import_module(module_name, __name__)
    return getattr(module, name)


def __dir__():
    return sorted(list(globals().keys()) + list(_LAZY_LEGACY.keys()))


__all__ = [
    'DynamicsLoader',  # NEW - for 4DOF controllers
    'Vehicle',         # LEGACY - for 6DOF model-based controllers (lazy)
    'DPControllerBase',
    'DPPIDControllerBase',
]
