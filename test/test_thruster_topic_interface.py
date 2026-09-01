"""A launch remapping must name a topic the node actually opens, and the docstring
must name the same ones.

Two failure modes this pins, both of which had already happened:

  P1-20 — thruster_manager.launch.py remapped '~/input_stamped', a PRIVATE name
  (/{ns}/thruster_allocator/input_stamped). The node subscribes to the RELATIVE
  name 'thruster_manager/input_stamped', so the remapping renamed a topic nobody
  opens: a no-op that reads like wiring.

  P1-16 — the class docstring advertised '~/input' (Wrench), '~/input_stamped' and
  '~/thruster_forces'. Only one subscription exists, it is not private, and the
  publisher emits PWM on an absolute /{vehicle_name}/setpoint/pwm.

rclpy is absent here, so both are read statically from the AST rather than from a
live node -- which is also what makes the check cheap enough to keep.
"""
import ast
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
NODE = REPO_ROOT / ("stonefish_control/stonefish_thruster_manager/"
                    "stonefish_thruster_manager/nodes/thruster_allocator_node.py")
LAUNCH = REPO_ROOT / "stonefish_control/stonefish_thruster_manager/launch/thruster_manager.launch.py"


def _tree(path):
    return ast.parse(path.read_text(), filename=str(path))


def _resolve_self_attr(tree, attr):
    """The string a `self.<attr> = ...` assignment binds, f-strings included."""
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if not (isinstance(target, ast.Attribute) and target.attr == attr):
                continue
            if isinstance(node.value, ast.Constant):
                return node.value.value
            if isinstance(node.value, ast.JoinedStr):
                return "".join(
                    p.value if isinstance(p, ast.Constant) else "{}"
                    for p in node.value.values
                )
    return None


def _opened_topics(tree):
    """Topic argument of every create_subscription/create_publisher call.

    Literals come back verbatim; an f-string yields its literal segments joined by
    '{}', so '/{vehicle_name}/setpoint/pwm' is recognisable without evaluating it.
    """
    names = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr not in ("create_subscription", "create_publisher"):
            continue
        # (msg_type, topic, ...) for both; the topic may be bound to an attribute.
        arg = node.args[1] if len(node.args) > 1 else None
        if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
            names.append(arg.value)
        elif isinstance(arg, ast.Attribute):          # self.output_topic
            names.append(_resolve_self_attr(tree, arg.attr))
    return [n for n in names if n]


def _class_docstring(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == name:
            return ast.get_docstring(node) or ""
    pytest.fail(f"class {name} not found")


def test_node_opens_exactly_the_topics_we_think_it_does():
    """Baseline for the two gates below -- if this changes, they must change with it."""
    assert sorted(_opened_topics(_tree(NODE))) == [
        "/{}/setpoint/pwm",
        "thruster_manager/input_stamped",
    ]


def test_launch_remaps_only_topics_the_node_opens():
    """P1-20: a remapping whose source no node opens is dead wiring, not a rename."""
    opened = set(_opened_topics(_tree(NODE)))
    for node in ast.walk(_tree(LAUNCH)):
        if not (isinstance(node, ast.keyword) and node.arg == "remappings"):
            continue
        for pair in getattr(node.value, "elts", []):
            src = pair.elts[0]
            assert isinstance(src, ast.Constant), "non-literal remapping source"
            assert src.value in opened, (
                f"launch remaps {src.value!r}, which the node never opens "
                f"(it opens {sorted(opened)}) -- a no-op that reads like wiring"
            )


def test_docstring_names_the_real_topics_and_no_invented_ones():
    """P1-16: the docstring is the only interface doc; a wrong one is worse than none."""
    doc = _class_docstring(_tree(NODE), "ThrusterAllocatorNode")

    assert "thruster_manager/input_stamped" in doc
    assert "setpoint/pwm" in doc

    for invented in ("~/input", "~/thruster_forces"):
        assert invented not in doc, f"docstring still advertises {invented!r}"
