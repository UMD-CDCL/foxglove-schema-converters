#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FOXGLOVE_EXT_ROOT="$HOME/snap/foxglove-studio/current/.foxglove-studio/extensions"

EXTENSIONS=(
  "cdcl-schema-converters"
  "cdcl-topic-converters"
)

mkdir -p "$FOXGLOVE_EXT_ROOT"

rm -rf "$FOXGLOVE_EXT_ROOT/unknown.observation-schema-converters-0.0.0"

for EXT_NAME in "${EXTENSIONS[@]}"; do
  EXT_DIR="$REPO_ROOT/$EXT_NAME"

  if [ ! -d "$EXT_DIR" ] || \
     [ ! -f "$EXT_DIR/package.json" ] || \
     [ ! -d "$EXT_DIR/dist" ] || \
     [ ! -f "$EXT_DIR/README.md" ] || \
     [ ! -f "$EXT_DIR/CHANGELOG.md" ]; then
    echo "$EXT_NAME: refresh failed"
    exit 1
  fi

  EXT_ID="$(
    cd "$EXT_DIR"
    node -p 'const p=require("./package.json"); `${p.publisher}.${p.name}-${p.version}`'
  )"

  HOST_EXT_DIR="$FOXGLOVE_EXT_ROOT/$EXT_ID"

  rm -rf "$HOST_EXT_DIR"
  mkdir -p "$HOST_EXT_DIR"

  cp -a \
    "$EXT_DIR/dist" \
    "$EXT_DIR/README.md" \
    "$EXT_DIR/CHANGELOG.md" \
    "$EXT_DIR/package.json" \
    "$HOST_EXT_DIR/"

done

echo "Success! Fully quit and reopen Foxglove Studio to reload extensions."
