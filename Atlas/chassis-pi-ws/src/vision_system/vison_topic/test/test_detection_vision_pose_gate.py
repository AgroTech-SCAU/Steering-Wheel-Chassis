from vison_topic.vision_pose_gate import detection_gate_rejection, detection_gate_ready_topic


def test_formal_detection_gate_uses_vision_pose_ready_topic():
    assert detection_gate_ready_topic() == "/vision_pose_ready"


def test_detection_start_rejects_when_no_legal_vision_pose_is_ready():
    allowed, message = detection_gate_rejection(
        require_vision_pose=True,
        vision_pose_ready=False,
        ready_topic="/vision_pose_ready",
    )

    assert not allowed
    assert "/vision_pose_ready=true" in message
