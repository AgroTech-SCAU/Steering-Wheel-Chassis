#!/usr/bin/env bash
set -euo pipefail
TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$TEST_DIR/.." && pwd)"
OUT="${TMPDIR:-/tmp}/arm_command_result_test"
c++ -std=c++17 -Wall -Wextra -Werror -I"$PKG_DIR/include" \
  "$TEST_DIR/arm_command_result_test.cpp" "$PKG_DIR/src/binary_frame.cpp" -o "$OUT"
"$OUT"
