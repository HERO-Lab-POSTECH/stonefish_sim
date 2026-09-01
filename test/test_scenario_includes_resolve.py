"""Every <include file="..."> in the description package must point at a file.

P0-2: scenarios/blueboat_sea.scn included data/worlds/sea.scn, a path that has
never existed anywhere in this repository's history. The scenario therefore
failed inside Stonefish's parser the moment anything tried to load it, and
nothing here noticed because a scenario is data, not code -- no import, no
build step, no test ever opened it. It was deleted along with the launch
wrapper whose only job was to name it.

This gate is the reason that class of defect cannot come back silently. It is
deliberately dumb: resolve the literal path, assert the file is there. Include
paths are resolved by Stonefish relative to the package root (the directory
holding data/ and scenarios/), which is what every resolving include in the
tree already assumes.

Out of scope on purpose: <include> children with a $(...) or @...@ substitution
in the path. None exist today; if one appears it is skipped rather than guessed
at.
"""
import re
from pathlib import Path

import pytest

DESCRIPTION_ROOT = Path(__file__).resolve().parent.parent / "stonefish_description"

_INCLUDE_FILE = re.compile(r'file="([^"]+)"')
_SUBSTITUTION = re.compile(r'\$\(|@')


def _scn_files():
    return sorted(DESCRIPTION_ROOT.rglob("*.scn"))


def test_the_tree_still_has_scenarios_to_check():
    """A rename of scenarios/ or data/ would otherwise make this file vacuously green."""
    files = _scn_files()
    assert len(files) >= 10, f"only {len(files)} .scn found under {DESCRIPTION_ROOT}"
    assert (DESCRIPTION_ROOT / "scenarios").is_dir()


@pytest.mark.parametrize("scn", _scn_files(), ids=lambda p: p.name)
def test_every_include_resolves(scn):
    unresolved = []
    for target in _INCLUDE_FILE.findall(scn.read_text()):
        if _SUBSTITUTION.search(target):
            continue
        if not (DESCRIPTION_ROOT / target).exists():
            unresolved.append(target)

    assert not unresolved, (
        f"{scn.relative_to(DESCRIPTION_ROOT)} includes files that do not exist: "
        f"{unresolved} -- Stonefish fails to parse the scenario, so it can never load"
    )


def test_the_deleted_blueboat_scenario_stays_deleted():
    """Restoring the .scn without its world would restore an unloadable scenario.

    Pinned by absence rather than by content so a rename does not slip past it:
    nothing in the tree may include data/worlds/sea.scn again.
    """
    assert not (DESCRIPTION_ROOT / "scenarios/blueboat_sea.scn").exists()
    assert not (DESCRIPTION_ROOT / "data/worlds/sea.scn").exists(), (
        "sea.scn now exists -- if the BlueBoat world was authored deliberately, "
        "restore the scenario and delete this assertion"
    )
