from types import SimpleNamespace

from atlas_nav_direct_backend.direct_nav_model import Pose2D
from atlas_nav_direct_backend.map_scan_match import MapScanMatch


def test_saved_map_rejects_a_start_pose_that_only_looks_like_origin(tmp_path):
    width = height = 60
    pixels = bytearray([254] * (width * height))
    pixels[height // 2 * width + width // 2 + 10] = 0
    (tmp_path / "map.pgm").write_bytes(b"P5\n60 60\n255\n" + pixels)
    (tmp_path / "map.yaml").write_text(
        "image: map.pgm\nresolution: 0.1\norigin: [-3.0, -3.0, 0.0]\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    matcher = MapScanMatch(str(tmp_path / "map.yaml"))
    scan = SimpleNamespace(
        ranges=[1.0], range_min=0.1, range_max=3.5,
        angle_min=0.0, angle_increment=0.0,
    )

    assert matcher.agreement(scan, Pose2D(0.0, 0.0, 0.0)) == (1.0, 1)
    assert matcher.agreement(scan, Pose2D(0.5, 0.0, 0.0)) == (0.0, 1)


def test_unobserved_cartographer_cells_are_not_counted_as_free(tmp_path):
    width = height = 60
    pixels = bytearray([205] * (width * height))
    (tmp_path / "map.pgm").write_bytes(b"P5\n60 60\n255\n" + pixels)
    (tmp_path / "map.yaml").write_text(
        "image: map.pgm\nresolution: 0.1\norigin: [-3.0, -3.0, 0.0]\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    matcher = MapScanMatch(str(tmp_path / "map.yaml"))
    scan = SimpleNamespace(
        ranges=[1.0], range_min=0.1, range_max=3.5,
        angle_min=0.0, angle_increment=0.0,
    )
    assert matcher.agreement(scan, Pose2D(0.0, 0.0, 0.0)) == (0.0, 0)
