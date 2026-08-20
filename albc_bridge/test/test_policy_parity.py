"""Golden-parity gate: npforward (pure numpy) must match torch-produced goldens
to atol=1e-5. Mirrors marinelab constrained_albc/deploy/golden.py's contract:
- golden_tcn: StudentTCN.forward(input_window) == latent == forward_out
- golden_teacher: TeacherActor.normalize(obs) == obs_normalized;
  TeacherActor.act(obs_normalized, latent) == action

npforward.py's policy/ dir has no __init__.py (not a built subpackage), so it
is imported by adding policy/ to sys.path directly rather than via
`albc_bridge.policy.npforward` -- policy assets are read-only, not touched here.
"""
import os
import sys

import numpy as np

_POLICY_DIR = os.path.join(os.path.dirname(__file__), "..", "albc_bridge", "policy")
_GOLDEN_DIR = os.path.join(_POLICY_DIR, "golden")
sys.path.insert(0, os.path.abspath(_POLICY_DIR))

from npforward import StudentTCN, TeacherActor  # noqa: E402


def test_tcn_parity():
    golden = np.load(os.path.join(_GOLDEN_DIR, "golden_tcn.npz"))
    weights = np.load(os.path.join(_POLICY_DIR, "weights_tcn.npz"))
    model = StudentTCN(weights)
    latent = model.forward(golden["input_window"])
    assert np.allclose(latent, golden["latent"], atol=1e-5)
    assert np.allclose(latent, golden["forward_out"], atol=1e-5)


def test_teacher_parity():
    golden = np.load(os.path.join(_GOLDEN_DIR, "golden_teacher.npz"))
    weights = np.load(os.path.join(_POLICY_DIR, "weights_teacher.npz"))
    model = TeacherActor(weights)
    obs_norm = model.normalize(golden["obs"])
    assert np.allclose(obs_norm, golden["obs_normalized"], atol=1e-5)
    action = model.act(obs_norm, golden["latent"])
    assert np.allclose(action, golden["action"], atol=1e-5)
