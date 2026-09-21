#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ATLAS="$(cd "$HERE/../.." && pwd)"
PI_SRC="$ATLAS/chassis-pi-ws/src"
export PYTHONPATH="$PI_SRC/app/atlas_competition_manipulation_backend:$PI_SRC/app/atlas_competition_config:$PI_SRC/vision_system/handeye_bridge:$PI_SRC/vision_system/vison_topic${PYTHONPATH:+:$PYTHONPATH}"
python -m pytest --import-mode=importlib -q \
 "$PI_SRC/app/atlas_competition_manipulation_backend/test" \
 "$PI_SRC/vision_system/handeye_bridge/test" \
 "$PI_SRC/vision_system/vison_topic/test/test_detection_vision_pose_gate.py" \
 "$PI_SRC/vision_system/vison_topic/test/test_detection_frame_lifecycle.py"
bash "$ATLAS/chassis_control_code/tests/run_serial_arm_ik_regression.sh"
bash "$ATLAS/chassis_control_code/tests/run_arm_command_result_regression.sh"
bash "$ATLAS/chassis_control_code/tests/run_arm_command_result_regression.sh" arm_command_gate_regression
bash "$PI_SRC/mcu_comm_bridge/test/run_arm_command_result_test.sh"
MODEL_OUT="$(mktemp -d)"
trap 'rm -rf "$MODEL_OUT"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror \
 -I"$HERE/model_host" -I"$PI_SRC/app/atlas_mission_yasmin/include" \
 "$PI_SRC/app/atlas_mission_yasmin/src/competition_model.cpp" \
 "$PI_SRC/app/atlas_mission_yasmin/src/pickup_scheduler.cpp" \
 "$PI_SRC/app/atlas_mission_yasmin/test/competition_model_unit_test.cpp" \
 "$HERE/model_host/model_test_main.cpp" -o "$MODEL_OUT/model_tests"
"$MODEL_OUT/model_tests"
