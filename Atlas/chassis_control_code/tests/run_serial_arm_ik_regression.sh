#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
OUT="${TMPDIR:-/tmp}/serial_arm_ik_regression"

cc -std=c11 -Wall -Wextra -Werror \
  -I"$PROJECT_DIR/src/domain" \
  "$SCRIPT_DIR/serial_arm_ik_regression.c" \
  -lm -o "$OUT"

"$OUT"
