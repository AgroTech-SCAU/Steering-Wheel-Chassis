from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from atlas_nav_direct_backend.startup_localization import localization_candidates  # noqa: E402


def _navigation():
    return {
        "arenas": {
            "A": {"map": "maps/a.yaml", "pbstream": "maps/a.pbstream"},
            "B": {"map": "maps/b.yaml", "pbstream": "maps/b.pbstream"},
        }
    }


def test_localization_candidates_can_be_limited_to_selected_arena(tmp_path):
    candidates = localization_candidates(_navigation(), tmp_path / "competition.yaml", arena="B")
    assert [candidate.label for candidate in candidates] == ["B"]
    assert candidates[0].map_path.endswith("maps/b.yaml")
    assert candidates[0].pbstream_path.endswith("maps/b.pbstream")


def test_localization_candidates_keep_all_unique_arenas_when_arena_unspecified(tmp_path):
    candidates = localization_candidates(_navigation(), tmp_path / "competition.yaml")
    assert [candidate.label for candidate in candidates] == ["A", "B"]
