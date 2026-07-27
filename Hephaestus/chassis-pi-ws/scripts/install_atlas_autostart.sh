#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ATLAS_USER="${2:-${SUDO_USER:-$USER}}"
CONFIG_PATH="${3:-${WORKSPACE}/install/atlas_autonomous_transport_manager/share/atlas_autonomous_transport_manager/config/autonomous_transport.yaml}"
TEMPLATE="${WORKSPACE}/install/robot_startup/share/robot_startup/systemd/atlas-autonomous.service.in"
SERVICE_FILE="/etc/systemd/system/atlas-autonomous.service"

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "未找到 ${WORKSPACE}/install/setup.bash；请先完成 colcon build"
  exit 1
fi

if [[ ! -f "${TEMPLATE}" ]]; then
  TEMPLATE="${WORKSPACE}/src/nav_system/robot_startup/systemd/atlas-autonomous.service.in"
fi

if [[ ! -f "${TEMPLATE}" ]]; then
  echo "未找到 systemd 模板 ${TEMPLATE}"
  exit 1
fi

WORKSPACE="$(realpath "${WORKSPACE}")"
CONFIG_PATH="$(realpath "${CONFIG_PATH}")"

sed \
  -e "s|@ATLAS_USER@|${ATLAS_USER}|g" \
  -e "s|@ATLAS_WORKSPACE@|${WORKSPACE}|g" \
  -e "s|@ATLAS_CONFIG@|${CONFIG_PATH}|g" \
  "${TEMPLATE}" | sudo tee "${SERVICE_FILE}" >/dev/null

sudo systemctl daemon-reload
sudo systemctl enable atlas-autonomous.service

echo "已安装 atlas-autonomous.service"
echo "首次实车联调建议先手动启动；确认无误后执行 sudo systemctl start atlas-autonomous.service"
