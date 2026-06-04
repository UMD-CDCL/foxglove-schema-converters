# Foxglove Schema Converters

Development workspace for building Foxglove extensions that convert custom ROS 2 messages into Foxglove-displayable topics.

## Goal

Programmatically generate and regenerate Foxglove converters for custom ROS 2 messages from:

```text
/ros_ws/src/cdcl_umd_msgs
```

The initial proof-of-technique message is:

```text
cdcl_umd_msgs/msg/ObservationDataSource.msg
```

Initial targets:

- Extract `sensor_msgs/CompressedImage image` from `ObservationDataSource`.
- Publish it as a displayable image topic in Foxglove.
- Extract `uint8[] raw_audio` from `ObservationDataSource`.
- Publish it as a Foxglove-compatible audio topic once encoding metadata is confirmed.

## Current Status

Verified working:

- VS Code Dev Container builds successfully.
- Foxglove extension scaffold builds successfully.
- Bootstrap script runs successfully.
- Extension installs into the host Foxglove snap profile when copied manually.
- Image extraction from `/observation_data_sources` to `/observation_data_sources/image` is working and visible in Foxglove.

Not yet completed:

- Audio converter is not yet applied/tested.
- `raw_audio` encoding metadata still needs to be confirmed:
  - sample rate
  - channel count
  - signed/unsigned encoding
  - bit depth
  - compressed vs raw PCM

## Repository Layout

```text
.
├── .devcontainer/
│   ├── Dockerfile
│   └── devcontainer.json
├── observation-schema-converters/
│   ├── src/
│   │   └── index.ts
│   ├── package.json
│   └── ...
├── scripts/
│   └── bootstrap.sh
└── README.md
```

## Development Environment

This repository uses a VS Code Dev Container.

Host requirements:

- Docker
- Docker Compose
- VS Code
- VS Code Dev Containers extension
- Git

The dev container mounts the custom ROS 2 message package at:

```text
/ros_ws/src/cdcl_umd_msgs
```

The host source path is expected to be:

```text
/home/ctitus/ros2_ws/src/cdcl_umd_msgs
```

## Bootstrap

From inside the dev container:

```bash
./scripts/bootstrap.sh
```

The bootstrap script currently:

1. Verifies the extension directory exists.
2. Verifies `ObservationDataSource.msg` is mounted.
3. Installs npm dependencies.
4. Packages the Foxglove extension.

## Extension Packaging

From inside `observation-schema-converters/`, run:

```bash
npm run package
```

This creates a `.foxe` package and builds:

```text
observation-schema-converters/dist/extension.js
```

## Installing into Foxglove Snap Profile

When developing inside the container, do not rely on:

```bash
npm run local-install
```

because it installs into the container's Foxglove profile, not the host user's profile.

For the host snap install, copy the built extension to:

```text
~/snap/foxglove-studio/current/.foxglove-studio/extensions/
```

Example from the host:

```bash
cd ~/foxglove-schema-converters/observation-schema-converters

EXT_ID="$(node -p 'const p=require("./package.json"); `${p.publisher}.${p.name}-${p.version}`')"
HOST_EXT_DIR="$HOME/snap/foxglove-studio/current/.foxglove-studio/extensions/$EXT_ID"

rm -rf "$HOST_EXT_DIR"
mkdir -p "$HOST_EXT_DIR"

cp -a dist README.md CHANGELOG.md package.json "$HOST_EXT_DIR/"
```

Then fully quit and reopen Foxglove Studio.

## Current Manual Converter

The first working converter maps:

```text
/observation_data_sources
```

to:

```text
/observation_data_sources/image
```

using:

```text
sensor_msgs/msg/CompressedImage
```

## Planned Pipeline

1. Maintain one hand-written converter for `ObservationDataSource`.
2. Prove image extraction works in Foxglove. Done.
3. Prove audio extraction or decide the correct audio display strategy.
4. Parse ROS 2 `.msg` files programmatically.
5. Generate Foxglove converter source files from message definitions.
6. Regenerate converters when message definitions change.
7. Package the extension for installation in Foxglove.
8. Add a clean host install script for snap-based Foxglove installs.
9. Add CI checks for build/package.

## Notes

Do not run:

```bash
npm audit fix
```

or:

```bash
npm audit fix --force
```

until the baseline extension builds, packages, installs, and works. Audit fixes can change dependency versions and make debugging harder.
