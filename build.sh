#!/usr/bin/env bash
#
# Builds the CDCL Foxglove converter extension and installs it into Foxglove
# Desktop. Regenerates the converters from the current cdcl_umd_msgs first, so
# the extension always matches the message definitions on disk.
#
#   ./build.sh                  generate, build, install
#   ./build.sh --no-install     stop after producing the .foxe
#   CDCL_MSG_PKG=/path/to/pkg ./build.sh
#
# Everything except the final copy runs in Docker, so no local Node or Python is
# needed.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MSG_PKG="${CDCL_MSG_PKG:-$HOME/ros2_ws/src/cdcl_umd_msgs}"
IMAGE="cdcl-foxglove-build"
EXT_ROOT="$HOME/snap/foxglove-studio/current/.foxglove-studio/extensions"

INSTALL=1
[ "${1:-}" = "--no-install" ] && INSTALL=0

cd "$REPO_ROOT"

if [ ! -d "$MSG_PKG/msg" ]; then
  echo "ERROR: no msg/ directory in $MSG_PKG"
  echo "Set CDCL_MSG_PKG to your cdcl_umd_msgs checkout."
  exit 1
fi

echo "Messages:  $MSG_PKG"

docker build -q -t "$IMAGE" "$REPO_ROOT" >/dev/null

# The message package is mounted read-only at /pkg, which the generator finds by
# default. Running as the host user keeps generated files owned by you.
docker run --rm \
  -u "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -v "$REPO_ROOT":/work \
  -v "$MSG_PKG":/pkg:ro \
  -w /work \
  "$IMAGE" \
  sh -euc '
    npm install --no-audit --no-fund --silent
    python3 scripts/generate_converters.py /pkg
    npm --workspace cdcl-converters run package
  '

FOXE="$(ls -t "$REPO_ROOT"/cdcl-converters/*.foxe 2>/dev/null | head -1)"

if [ -z "$FOXE" ]; then
  echo "ERROR: no .foxe was produced"
  exit 1
fi

echo "Packaged:  $FOXE"

if [ "$INSTALL" -eq 0 ]; then
  exit 0
fi

if [ ! -d "$(dirname "$EXT_ROOT")" ]; then
  echo "Foxglove Desktop (snap) not found; skipping install."
  echo "Upload $FOXE manually instead."
  exit 0
fi

EXT_ID="$(python3 -c "
import json
p = json.load(open('cdcl-converters/package.json'))
print(f\"{p['publisher']}.{p['name']}-{p['version']}\")
")"

mkdir -p "$EXT_ROOT"

# Drop older builds, including the two extensions this one replaced.
rm -rf "$EXT_ROOT"/umd-cdcl.cdcl-converters-* \
       "$EXT_ROOT"/umd-cdcl.cdcl-schema-converters-* \
       "$EXT_ROOT"/umd-cdcl.cdcl-topic-converters-*

mkdir -p "$EXT_ROOT/$EXT_ID"
cp -a cdcl-converters/dist \
      cdcl-converters/package.json \
      cdcl-converters/README.md \
      cdcl-converters/CHANGELOG.md \
      "$EXT_ROOT/$EXT_ID/"

echo "Installed: $EXT_ROOT/$EXT_ID"
echo
echo "Fully quit and reopen Foxglove Studio to load the extension."
