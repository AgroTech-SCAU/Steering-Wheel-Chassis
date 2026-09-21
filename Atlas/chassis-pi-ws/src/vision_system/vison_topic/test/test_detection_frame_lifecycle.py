"""Exercise the real detector callbacks without camera, ONNX, or ROS hardware."""
import array
import importlib
import sys
import threading
import time
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np


def load_detector():
    stubs = {}
    try:
        import cv2  # noqa: F401
    except ImportError:
        stubs['cv2'] = ModuleType('cv2')
    try:
        import vison_topic_interfaces.srv  # noqa: F401
    except ImportError:
        srv = ModuleType('vison_topic_interfaces.srv')
        srv.VisionDetect = object
        msg = ModuleType('vison_topic_interfaces.msg')
        msg.DetectionCenter = object
        msg.DetectionCenterArray = object
        stubs.update({'vison_topic_interfaces.srv': srv,
                      'vison_topic_interfaces.msg': msg})
    with patch.dict(sys.modules, stubs):
        return importlib.import_module('vison_topic.detect_and_send')


detector = load_detector()


class DetectionFrameLifecycleTest(unittest.TestCase):
    def setUp(self):
        self.server = detector.VisionDetectServer.__new__(detector.VisionDetectServer)
        s = self.server
        s._lock = threading.Lock()
        s._node = Mock()
        s._node.get_clock().now().to_msg.return_value = SimpleNamespace(sec=1, nanosec=0)
        s._running = False
        s._detection_timer = None
        s._require_vision_pose = True
        s._vision_pose_ready = True
        s._vision_pose_received_at = time.monotonic()
        s._latest_detections = []
        s._latest_infer_ms = 0.0
        s._best_detections = []
        s._best_score = -1.0
        s._best_frame_stamp = None
        s._start_time = None
        s._cap = Mock()
        s._cap.read.return_value = (True, np.zeros((480, 640, 3), dtype=np.uint8))
        s._camera_fail_count = 0
        s._process_every_n = 1
        s._model = object()
        s._conf_threshold = 0.5
        s._class_names = ['gear']
        s._publish_rate_hz = 0
        s._pub = Mock()
        s._pub_msg = SimpleNamespace(data=array.array('f'))
        s._publish_centers = Mock()
        s._frame_count = 0
        s._last_fps_time = 0.0
        s._fps = 0.0
        s._show_preview = False
        self.inference = patch.object(detector, 'run_inference', return_value=(
            np.array([[100., 100., 200., 200., 0.9, 0.]]), 2.0)).start()
        self.addCleanup(patch.stopall)

    def start_and_capture(self):
        self.server._start_detection()
        self.server._detection_tick()  # flush the pre-start buffered camera frame
        self.server._detection_tick()
        self.assertEqual(len(self.server._best_detections), 1)

    def test_first_camera_frame_is_discarded_after_start(self):
        self.server._start_detection()
        self.server._detection_tick()
        self.assertEqual(self.server._best_detections, [])
        self.inference.assert_not_called()
        self.server._detection_tick()
        self.assertEqual(len(self.server._best_detections), 1)

    def test_moving_arm_frames_do_not_enter_best(self):
        self.server._start_detection()
        self.server._vision_pose_ready = False
        self.server._detection_tick()
        self.server._detection_tick()
        self.assertEqual(self.server._best_detections, [])
        self.inference.assert_not_called()

    def test_readiness_loss_clears_best_and_latest(self):
        self.start_and_capture()
        self.server._on_vision_pose_ready(SimpleNamespace(data=False))
        self.assertEqual(self.server._best_detections, [])
        self.assertEqual(self.server._latest_detections, [])
        self.assertIsNone(self.server._best_frame_stamp)
        self.assertFalse(self.server._running)

    def test_stale_ready_heartbeat_rejects_start(self):
        self.server._vision_pose_received_at = time.monotonic() - 2.0
        response = self.server._on_detect_request(SimpleNamespace(start=True), SimpleNamespace())
        self.assertFalse(response.success)
        self.assertFalse(self.server._running)

    def test_stale_ready_heartbeat_cannot_publish_final_best(self):
        self.start_and_capture()
        self.server._publish_centers.reset_mock()
        self.server._vision_pose_received_at = time.monotonic() - 2.0
        response = self.server._on_detect_request(SimpleNamespace(start=False), SimpleNamespace())
        self.assertFalse(response.success)
        self.assertEqual(response.count, 0)
        self.server._publish_centers.assert_not_called()
        self.assertEqual(self.server._best_detections, [])

    def test_readiness_expiring_during_inference_clears_previous_best(self):
        self.start_and_capture()
        self.server._publish_centers.reset_mock()
        def expire_readiness(*_args):
            self.server._vision_pose_received_at = time.monotonic() - 2.0
            return np.array([[100., 100., 200., 200., 0.99, 0.]]), 2.0
        self.inference.side_effect = expire_readiness
        self.server._detection_tick()
        self.assertEqual(self.server._best_detections, [])
        self.server._publish_centers.assert_not_called()

    def test_readiness_recovery_discards_buffered_frame(self):
        self.start_and_capture()
        self.server._vision_pose_received_at = time.monotonic() - 2.0
        self.server._detection_tick()
        self.assertEqual(self.server._best_detections, [])
        self.server._on_vision_pose_ready(SimpleNamespace(data=True))
        self.server._detection_tick()
        self.assertEqual(self.server._best_detections, [])
        self.server._detection_tick()
        self.assertEqual(len(self.server._best_detections), 1)

    def test_frame_cannot_survive_observation_change_during_inference(self):
        self.server._start_detection()
        self.server._detection_tick()
        def change_observation(*_args):
            self.server._on_vision_pose_ready(SimpleNamespace(data=False))
            self.server._on_vision_pose_ready(SimpleNamespace(data=True))
            self.server._start_detection()
            return np.array([[100., 100., 200., 200., 0.99, 0.]]), 2.0
        self.inference.side_effect = change_observation
        self.server._detection_tick()
        self.assertEqual(self.server._best_detections, [])
        self.server._publish_centers.assert_not_called()

    def test_stop_returns_original_best_once_then_clears_cache(self):
        self.start_and_capture()
        original_stamp = self.server._best_frame_stamp
        self.server._publish_centers.reset_mock()
        response = self.server._on_detect_request(SimpleNamespace(start=False), SimpleNamespace())
        self.assertTrue(response.success)
        self.assertEqual(response.count, 1)
        self.assertEqual(self.server._publish_centers.call_args.args[1], original_stamp)
        self.assertTrue(self.server._publish_centers.call_args.kwargs['is_final_best'])
        self.assertEqual(self.server._best_detections, [])
        self.assertIsNone(self.server._best_frame_stamp)
        self.server._publish_centers.reset_mock()
        again = self.server._on_detect_request(SimpleNamespace(start=False), SimpleNamespace())
        self.assertFalse(again.success)
        self.server._publish_centers.assert_not_called()

    def test_start_after_observation_change_cannot_return_old_best(self):
        self.start_and_capture()
        self.server._on_vision_pose_ready(SimpleNamespace(data=False))
        self.server._on_vision_pose_ready(SimpleNamespace(data=True))
        self.server._start_detection()
        response = self.server._on_detect_request(SimpleNamespace(start=False), SimpleNamespace())
        self.assertTrue(response.success)
        self.assertEqual(response.count, 0)


if __name__ == '__main__':
    unittest.main()
