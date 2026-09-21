#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
TEST_NAME="${1:-arm_command_result_regression}"
OUT="${TMPDIR:-/tmp}/$TEST_NAME"
cc -std=c11 -Wall -Wextra -Werror -ffunction-sections -fdata-sections \
  -I"$SCRIPT_DIR/stubs" -I"$PROJECT_DIR/src/service" -I"$PROJECT_DIR/src/domain" \
  -I"$PROJECT_DIR/src/device" -I"$PROJECT_DIR/src/infra" \
  "$SCRIPT_DIR/$TEST_NAME.c" \
  "$PROJECT_DIR/src/service/pi_comms.c" \
  "$PROJECT_DIR/src/infra/binary_frame.c" \
  "$PROJECT_DIR/src/infra/protocol_parser.c" \
  "$PROJECT_DIR/src/infra/delay.c" "$PROJECT_DIR/src/infra/log.c" \
  -Wl,--gc-sections -lm -o "$OUT"
"$OUT"
