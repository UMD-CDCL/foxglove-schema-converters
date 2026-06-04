#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXT_DIR="$REPO_ROOT/cdcl-schema-converters"
FOXGLOVE_EXT_ROOT="$HOME/snap/foxglove-studio/current/.foxglove-studio/extensions"

if [ ! -d "$EXT_DIR" ]; then
  echo "ERROR: missing extension directory:"
  echo "  $EXT_DIR"
  exit 1
fi

if [ ! -f "$EXT_DIR/package.json" ]; then
  echo "ERROR: missing package.json:"
  echo "  $EXT_DIR/package.json"
  exit 1
fi

EXT_ID="$(
  cd "$EXT_DIR"
  node -p 'const p=require("./package.json"); `${p.publisher}.${p.name}-${p.version}`'
)"

HOST_EXT_DIR="$FOXGLOVE_EXT_ROOT/$EXT_ID"

echo "Extension ID: $EXT_ID"
echo "Install dir:  $HOST_EXT_DIR"

mkdir -p "$FOXGLOVE_EXT_ROOT"

# Remove old pre-rename install if present.
rm -rf "$FOXGLOVE_EXT_ROOT/unknown.observation-schema-converters-0.0.0"

# Replace current install.
rm -rf "$HOST_EXT_DIR"
mkdir -p "$HOST_EXT_DIR"

cp -a \
  "$EXT_DIR/dist" \
  "$EXT_DIR/README.md" \
  "$EXT_DIR/CHANGELOG.md" \
  "$EXT_DIR/package.json" \
  "$HOST_EXT_DIR/"

echo "Refreshed Foxglove extension:"
echo "  $HOST_EXT_DIR"
echo
echo "Fully quit and reopen Foxglove Studio to reload the extension."
