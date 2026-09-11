from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = Path(__file__).resolve().parents[3]


def test_navigation_status_publisher_uses_declared_yaw_error_field():
    msg_text = (WORKSPACE_SRC / "app" / "atlas_mission_interfaces" / "msg" / "NavigationStatus.msg").read_text(encoding="utf-8")
    backend_text = (PACKAGE_ROOT / "atlas_nav_full_backend" / "full_nav_backend.py").read_text(encoding="utf-8")
    publish_status = backend_text[
        backend_text.index("    def publish_status(self)"):
    ]

    assert "float64 yaw_error_rad" in msg_text
    assert "msg.yaw_error_rad" in publish_status
    assert "msg.angle_error_rad" not in publish_status


def test_ctrl_c_is_treated_as_clean_shutdown():
    backend_text = (PACKAGE_ROOT / "atlas_nav_full_backend" / "full_nav_backend.py").read_text(encoding="utf-8")
    main_text = backend_text[backend_text.index("def main(args=None)"):]

    assert "except KeyboardInterrupt:" in main_text
    assert "if rclpy.ok():" in main_text


def test_backend_waits_for_bt_navigator_active_before_sending_goal():
    backend_text = (PACKAGE_ROOT / "atlas_nav_full_backend" / "full_nav_backend.py").read_text(encoding="utf-8")
    ensure_text = backend_text[
        backend_text.index("    def _ensure_nav_stack"):
        backend_text.index("    def on_cancel")
    ]

    assert "self._wait_for_nav2_active(self.stack_ready_timeout_s)" in ensure_text
    assert "State.PRIMARY_STATE_ACTIVE" in ensure_text
    assert "MultiThreadedExecutor(num_threads=4)" in backend_text
    assert "ReentrantCallbackGroup()" in backend_text
