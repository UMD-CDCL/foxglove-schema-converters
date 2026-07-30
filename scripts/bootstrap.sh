#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

EXTENSIONS=(
  "cdcl-schema-converters"
  "cdcl-topic-converters"
)

cd "$REPO_ROOT"

echo "Repo root: $REPO_ROOT"

for EXT_NAME in "${EXTENSIONS[@]}"; do
  if [ ! -d "$REPO_ROOT/$EXT_NAME" ]; then
    echo "ERROR: missing extension directory: $REPO_ROOT/$EXT_NAME"
    exit 1
  fi
done

# The generators resolve the message tree themselves (dev-container path first,
# then ~/ros2_ws, or $CDCL_MSG_ROOTS). Check it up front so a missing mount is
# reported here rather than part-way through a build.
if ! python3 -c "
import sys
sys.path.insert(0, 'scripts')
from rosmsg import resolve_msg_roots
print('Message root:', resolve_msg_roots(None)[0])
"; then
  echo
  echo "ERROR: no ROS message directory found."
  echo "Set CDCL_MSG_ROOTS to the msg/ directory of your message package."
  exit 1
fi

echo "Installing npm dependencies..."
npm install

echo "Generating converters..."
npm run generate

echo "Packaging Foxglove extensions..."
npm run build:schema
npm run build:topics

echo "Bootstrap complete."
