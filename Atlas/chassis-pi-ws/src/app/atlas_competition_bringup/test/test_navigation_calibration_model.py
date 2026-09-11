from pathlib import Path
import importlib.util

import yaml


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "navigation_calibration_model.py"


def _load_model():
    assert MODULE_PATH.is_file(), "navigation_calibration_model.py must exist"
    spec = importlib.util.spec_from_file_location("navigation_calibration_model", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _source(map_value: str, pbstream_value: str):
    return {
        "competition": {
            "navigation": {
                "coordinate_mode": "absolute_map",
                "arenas": {
                    "A": {
                        "map": map_value,
                        "pbstream": pbstream_value,
                        "waypoints": {
                            "pickup": {"x": 9.0, "y": 9.0, "yaw": 9.0, "configured": True},
                        },
                    },
                    "B": {
                        "map": "maps/b.yaml",
                        "pbstream": "maps/b.pbstream",
                        "waypoints": {
                            "pickup": {"x": 8.0, "y": 8.0, "yaw": 8.0, "configured": True},
                        },
                    },
                },
            }
        }
    }


def test_inspect_navigation_maps_requires_map_image_and_pbstream(tmp_path):
    model = _load_model()
    config_dir = tmp_path / "config"
    maps = config_dir / "maps"
    maps.mkdir(parents=True)
    config_path = config_dir / "competition.yaml"
    config_path.write_text("competition: {}\n", encoding="utf-8")

    (maps / "arena.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    (maps / "arena.pbstream").write_bytes(b"pbstream")
    (maps / "arena.yaml").write_text(
        "image: arena.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    source = _source("maps/arena.yaml", "maps/arena.pbstream")

    statuses = model.inspect_navigation_maps(source, config_path)
    assert statuses["A"].ready is True
    assert statuses["A"].map_path == maps / "arena.yaml"
    assert statuses["A"].image_path == maps / "arena.pgm"
    assert statuses["A"].pbstream_path == maps / "arena.pbstream"

    (maps / "arena.pgm").unlink()
    statuses = model.inspect_navigation_maps(source, config_path)
    assert statuses["A"].ready is False
    assert any("image" in reason for reason in statuses["A"].errors)


def test_merge_navigation_calibration_updates_only_selected_arena():
    model = _load_model()
    source = _source("maps/a.yaml", "maps/a.pbstream")
    waypoints = {
        "pickup": {"x": 1.1, "y": 0.2, "yaw": 0.3, "configured": True},
        "park_1": {"x": 2.1, "y": -0.2, "yaw": -1.0, "configured": True},
        "park_2": {"x": 2.2, "y": 0.4, "yaw": 1.0, "configured": True},
    }

    merged = model.merge_navigation_calibration(source, "A", waypoints)
    nav = merged["competition"]["navigation"]

    assert nav["coordinate_mode"] == "absolute_map"
    assert nav["arenas"]["A"]["waypoints"] == waypoints
    assert nav["arenas"]["B"]["waypoints"]["pickup"]["x"] == 8.0
    assert nav["arenas"]["A"]["map"] == "maps/a.yaml"
    assert nav["arenas"]["A"]["pbstream"] == "maps/a.pbstream"


def test_dump_clean_yaml_round_trips():
    model = _load_model()
    data = {"competition": {"navigation": {"coordinate_mode": "absolute_map"}}}
    text = model.dump_clean_yaml(data)
    assert "#" not in text
    assert yaml.safe_load(text) == data
