#!/usr/bin/env python3
"""手眼标定数据质量与坐标约定回归测试。"""

import os
import unittest
from unittest import mock

import numpy as np
import cv2

from calib_utils import (corner_subpix_win, load_sample_validity,
                         make_chessboard_objp, prepare_inputs,
                         validate_chessboard_geometry)
from fk_utils import make_transform
from recompute_t2c import solve_pnp
from solve import (_intrinsics_binding_matches_file,
                   _load_intrinsics_binding)
import solve as solve_module


HERE = os.path.dirname(os.path.abspath(__file__))


class CalibrationQualityTests(unittest.TestCase):
    def test_subpixel_window_stays_inside_grid_spacing(self):
        corners = np.mgrid[0:11, 0:8].T.reshape(-1, 1, 2).astype(np.float32)
        corners *= 14.0
        win = corner_subpix_win(
            np.zeros((480, 640), dtype=np.uint8), (11, 8), corners)
        self.assertEqual(win, (3, 3))
        self.assertLess(2 * win[0] + 1, 14)

    def test_collapsed_grid_is_rejected(self):
        corners = np.mgrid[0:11, 0:8].T.reshape(-1, 1, 2).astype(np.float32)
        corners *= 18.0
        corners[12] = corners[11]
        ok, diag = validate_chessboard_geometry(corners, (11, 8))
        self.assertFalse(ok)
        self.assertEqual(diag["reason"], "collapsed_or_irregular_grid")

    def test_packaged_samples_reject_known_corrupt_frames(self):
        valid, rejected = load_sample_validity(
            os.path.join(HERE, "samples.yaml"))
        self.assertEqual(
            [item["index"] + 1 for item in rejected],
            [4, 5, 6, 11, 12, 13, 14, 15])
        self.assertEqual(len(valid), 25)

    def test_eye_to_hand_preparation_inverts_robot_pose(self):
        pose = make_transform(np.eye(3), [0.1, -0.2, 0.3])
        target = np.eye(4)
        _, translations, _, _ = prepare_inputs(
            [pose], [target], "eye_to_hand")
        np.testing.assert_allclose(
            translations[0].reshape(3), [-0.1, 0.2, -0.3])

    def test_planar_pnp_recovers_synthetic_pose(self):
        obj = make_chessboard_objp(11, 8, 0.015)
        camera = np.array([
            [532.0, 0.0, 342.0],
            [0.0, 531.0, 269.0],
            [0.0, 0.0, 1.0],
        ])
        distortion = np.zeros(5)
        expected_rvec = np.array([[0.15], [-0.10], [0.05]])
        expected_tvec = np.array([[0.02], [-0.03], [0.42]])
        image, _ = cv2.projectPoints(
            obj, expected_rvec, expected_tvec, camera, distortion)
        pose, reprojection = solve_pnp(
            obj, image.astype(np.float32), camera, distortion)
        self.assertIsNotNone(pose)
        self.assertLess(reprojection, 1e-3)
        np.testing.assert_allclose(
            pose[:3, 3], expected_tvec.reshape(3), atol=1e-5)

    def test_intrinsics_binding_detects_deployment_mismatch(self):
        samples = os.path.join(HERE, "samples.yaml")
        binding = _load_intrinsics_binding(samples)
        self.assertIsNotNone(binding)
        self.assertTrue(_intrinsics_binding_matches_file(
            binding, os.path.join(HERE, "camera_intrinsics.yaml")))
        self.assertFalse(_intrinsics_binding_matches_file(
            binding,
            os.path.join(HERE, "..", "handeye_bridge", "config",
                         "camera_intrinsics.yaml")))

    def test_simple_mode_bypasses_quality_filter(self):
        samples = os.path.join(HERE, "samples.yaml")
        with mock.patch.object(
                solve_module, "_solve_simple", return_value="simple") as call:
            result = solve_module.solve(samples, simple=True)
        self.assertEqual(result, "simple")
        args = call.call_args.args
        self.assertEqual(len(args[1]), 33)
        self.assertEqual(len(args[2]), 33)
        self.assertEqual(call.call_args.kwargs["pre_rejected"], [])


if __name__ == "__main__":
    unittest.main()
