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
├── cdcl-schema-converters/
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
$HOME/ros2_ws/src/cdcl_umd_msgs
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

From inside `cdcl-schema-converters/`, run:

```bash
npm run package
```

This creates a `.foxe` package and builds:

```text
cdcl-schema-converters/dist/extension.js
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
cd ~/foxglove-schema-converters/cdcl-schema-converters

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

<!-- CDCL_BUILD_AND_DEBUG_START -->

## Current Extension Layout

This repo builds two Foxglove extensions:

- `cdcl-schema-converters`
  - General schema converters for CDCL ROS 2 message fields.
  - `TargetBoxArray` is intentionally excluded here.
- `cdcl-topic-converters`
  - Topic converters for `TargetBoxArray` / TBA topics.
  - Handles `/uas#/target_locations` and `/uas#/target_detections`.
  - Produces image, pose/orientation, UAV paused location, target localization, and bounding-box annotation outputs.

The VS Code Docker development environment lives in:

    .devcontainer/

The dev container is built locally by VS Code from:

    .devcontainer/Dockerfile

No separate Docker image registry upload is required for normal development. Pushing this repo uploads the Docker/devcontainer definition needed for others to rebuild the same development environment.

## Rebuilding Extensions

Open the repo in VS Code and reopen it in the dev container:

    code ~/foxglove-schema-converters

Then run inside the dev container:

    cd /workspaces/foxglove-schema-converters
    npm run build

This regenerates converter source files and packages both extensions.

Individual commands:

    npm run generate
    npm run build:schema
    npm run build:topics

Generated packages are written to:

    cdcl-schema-converters/*.foxe
    cdcl-topic-converters/*.foxe

## Local Debug Install for Foxglove Desktop

For local testing, the preferred workflow is to copy the unpacked extension files directly into the Foxglove Desktop snap extension directory.

After building inside the dev container, run this on the host:

    cd ~/foxglove-schema-converters
    ./scripts/refresh_foxglove_extension.sh

Expected success output:

    Success! Fully quit and reopen Foxglove Studio to reload extensions.

Then fully quit and reopen Foxglove Studio.

This installs both extensions locally:

    cdcl-schema-converters
    cdcl-topic-converters

The script installs into:

    ~/snap/foxglove-studio/current/.foxglove-studio/extensions/

This local install path is more reliable for development than manually uploading `.foxe` files, because it avoids cloud/org upload permissions and API issues.

## Manual `.foxe` Files

The packaged `.foxe` files can still be used for manual upload or sharing:

    cdcl-schema-converters/umdcdcl.cdcl-schema-converters-*.foxe
    cdcl-topic-converters/umdcdcl.cdcl-topic-converters-*.foxe

Note: Foxglove tooling may sanitize the publisher name in archive filenames, so `umd-cdcl` can appear as `umdcdcl` in `.foxe` filenames.

## Topic Converter Outputs

The TBA topic converter expects UAS-specific input topics:

    /uas1/target_locations
    /uas2/target_locations
    /uas3/target_locations
    /uas4/target_locations

and:

    /uas1/target_detections
    /uas2/target_detections
    /uas3/target_detections
    /uas4/target_detections

Generated output topics include:

    /uas#/target_locations/source_img
    /uas#/target_locations/uav_paused_location
    /uas#/target_locations/uav_local_pose
    /uas#/target_locations/gimbal_attitude_quaternion
    /uas#/target_locations/altimeter
    /uas#/target_locations/gimbal
    /uas#/target_locations/rangefinder
    /uas#/target_locations/bounding_boxes

and for detections:

    /uas#/target_detections/source_img
    /uas#/target_detections/uav_paused_location
    /uas#/target_detections/uav_local_pose
    /uas#/target_detections/gimbal_attitude_quaternion
    /uas#/target_detections/bounding_boxes

`target_detections` intentionally skips altimeter/gimbal/rangefinder localization outputs.

## Verifying in Foxglove

Use these panels:

- Image panel:
  - `/uas#/target_locations/source_img`
  - `/uas#/target_detections/source_img`
- Map panel:
  - `/uas#/target_locations/uav_paused_location`
  - `/uas#/target_locations/altimeter`
  - `/uas#/target_locations/gimbal`
  - `/uas#/target_locations/rangefinder`
  - `/uas#/target_detections/uav_paused_location`
- Image annotations:
  - `/uas#/target_locations/bounding_boxes`
  - `/uas#/target_detections/bounding_boxes`

## Updating Foxglove Desktop Snap

On the host:

    sudo snap refresh foxglove-studio
    snap list foxglove-studio

Then fully quit and reopen Foxglove Studio.

<!-- CDCL_BUILD_AND_DEBUG_END -->
