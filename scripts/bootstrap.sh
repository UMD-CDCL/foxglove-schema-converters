#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXT_DIR="$REPO_ROOT/cdcl-schema-converters"

cd "$REPO_ROOT"

echo "Repo root: $REPO_ROOT"

if [ ! -d "$EXT_DIR" ]; then
  echo "ERROR: missing extension directory:"
  echo "  $EXT_DIR"
  echo
  echo "Create it first with:"
  echo "  npm create foxglove-extension@latest cdcl-schema-converters"
  exit 1
fi

if [ ! -f /ros_ws/src/cdcl_umd_msgs/msg/ObservationDataSource.msg ]; then
  echo "ERROR: expected ROS message not found:"
  echo "  /ros_ws/src/cdcl_umd_msgs/msg/ObservationDataSource.msg"
  exit 1
fi

cd "$EXT_DIR"

echo "Installing npm dependencies..."
npm install

echo "Packaging Foxglove extension..."
npm run package

echo "Bootstrap complete."
