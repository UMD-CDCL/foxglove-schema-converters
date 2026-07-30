# CDCL Foxglove Converters

One Foxglove extension that makes `cdcl_umd_msgs` messages displayable in
Foxglove's built-in panels. The converters are generated from the `.msg` files,
so the extension always matches the messages on disk.

## Quickstart

```bash
./build.py
```

Then fully quit and reopen Foxglove Studio. Re-run after any message change.

Docker and Python 3 are the only requirements — the Node toolchain runs in the
container.

```bash
./build.py --msgs /path/to/cdcl_umd_msgs   # default: ~/ros2_ws/src/cdcl_umd_msgs
./build.py --no-install                    # just produce the .foxe
./build.py --ext-dir DIR                   # install somewhere else
```

The extension is unpacked into `~/.foxglove-studio/extensions/`, where Foxglove
Desktop looks for it. A snap install of Foxglove reads a sandboxed `$HOME`
instead, so `~/snap/foxglove-studio/current/.foxglove-studio/extensions/` is used
when a real snap is present. If Foxglove reports no installed extensions, check
which of those two it is reading and pass `--ext-dir`.

To install by hand instead, use `cdcl-converters/umdcdcl.cdcl-converters-1.0.0.foxe`
(Foxglove **Settings → Extensions → Install from file**).

## How it works

`scripts/generate_converters.py` reads every `.msg` in the package (recursively,
so `msg/radar/` is included), walks each message's fields through nested messages
and arrays, and emits a converter for every field type Foxglove can display. No
message is listed by hand.

Foxglove allows **one converter per (source schema, target schema) pair**, so a
message has one output slot per target. Fields claim slots in declaration order;
anything left without one is printed as a note during the build rather than
silently dropped.

Converted output appears under the **original topic name** with a new schema —
select the topic in a panel and Foxglove offers it. The exception is topic
converters, which create new topics; see below.

| File | Purpose |
| --- | --- |
| `build.py` | Generate, build in Docker, install |
| `scripts/generate_converters.py` | Message scanner and spec generator |
| `cdcl-converters/src/converterRuntime.ts` | All conversion logic |
| `cdcl-converters/src/converterSpecs.ts` | Generated data — do not edit |

## Conversion table

| ROS field | Foxglove schema | Panel |
| --- | --- | --- |
| `sensor_msgs/CompressedImage` | `sensor_msgs/msg/CompressedImage` | Image |
| `sensor_msgs/Image` | `sensor_msgs/msg/Image` | Image |
| First `sensor_msgs/NavSatFix` | `sensor_msgs/msg/NavSatFix` | Map |
| Any further location fields | `foxglove_msgs/msg/GeoJSON` | Map |
| `NavSatFix[]` named like a boundary | `foxglove_msgs/msg/GeoJSON` (polygon) | Map |
| `vision_msgs/BoundingBox2D` | `foxglove_msgs/msg/ImageAnnotations` | Image |
| `nav_msgs/Odometry` | `nav_msgs/msg/Odometry` | 3D |
| `geometry_msgs/Quaternion`, `Pose`, `Point`, `Vector3`, … | matching ROS schema | 3D / Raw |
| `uint8[] raw_audio` | `foxglove_msgs/msg/RawAudio` | Audio |
| Text-ish `string` fields | `foxglove_msgs/msg/Log` | Log |

Location and bounding-box fields merge into one output each, so all targets in a
message draw together. A `NavSatFix` at exactly `0, 0` is treated as unlocalized
and dropped. Every output from one message carries the root message's
`header.stamp`, so an image and its annotations line up in the Image panel.

## TargetBoxArray topic converters

`uav_target_boxes` carries three alternative localizations per target (altimeter
plane, gimbal plane, rangefinder). They all want the same GeoJSON slot, and one
schema converter cannot render them as three independently toggleable Map layers
— so these three, and only these three, use **topic converters**, which produce
real new topics:

```text
/uas#/target_locations/altimeter
/uas#/target_locations/gimbal
/uas#/target_locations/rangefinder
```

Everything else on the message — source image, bounding boxes, UAV location,
local pose, gimbal attitude — stays on schema converters and so is available on
*every* `TargetBoxArray` topic, including ones nobody configured.

Topic converters must name their input topics, so they are listed in
`TOPIC_RULES` at the top of `scripts/generate_converters.py` (currently
`/uas1..4/target_locations` and `/uas1..4/tf_localization/localized`). Add a
topic there to cover it. Which *fields* split is not hard-coded: the generator
splits only fields that contend for a slot, reading the leaves from
`TargetBox.msg`, so a fourth localization would add a fourth topic on its own.

Existing layouts: UAV location, local pose and gimbal attitude used to be
separate `/uas#/target_locations/...` topics. They are now on the parent topic
under their own schema — select `/uas#/target_locations` in the panel. The three
localization topics above are unchanged.
