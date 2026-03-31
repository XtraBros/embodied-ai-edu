#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH="${CONFIG_PATH:-$ROOT_DIR/config/config.json}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

source_if_exists() {
  local file="$1"
  if [[ -f "$file" ]]; then
    # shellcheck disable=SC1090
    source "$file"
  fi
}

if [[ -n "${BOOSTER_ROS_SETUP:-}" ]]; then
  source_if_exists "$BOOSTER_ROS_SETUP"
else
  source_if_exists /opt/ros/humble/setup.bash
  source_if_exists /opt/BoosterAgent/install/setup.bash
  source_if_exists /opt/BoosterAgent/install/local_setup.bash
  source_if_exists /opt/booster/BoosterAgent/install/setup.bash
fi

exec "$PYTHON_BIN" "$ROOT_DIR/src/main.py" --config "$CONFIG_PATH" "$@"
