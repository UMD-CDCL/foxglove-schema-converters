# CDCL Foxglove Converters

Builds a single Foxglove extension that makes custom `cdcl_umd_msgs` messages
displayable in Foxglove's built-in panels. The converters are generated from the
`.msg` files, so the extension always matches the messages on disk.

## Quickstart

```bash
./build.py
```

Then fully quit and reopen Foxglove Studio.

That regenerates the converters from `~/ros2_ws/src/cdcl_umd_msgs`, builds the
extension in Docker, and installs it into Foxglove Desktop. Docker and Python 3
are the only requirements — the Node toolchain runs in the container.

```bash
./build.py --msgs /path/to/cdcl_umd_msgs   # build against a different checkout
./build.py --no-install                    # just produce the .foxe
```

To upload by hand instead, use the packaged extension:

```text
cdcl-converters/umdcdcl.cdcl-converters-1.0.0.foxe
```

Foxglove Desktop: **Settings → Extensions → Install from file**. Foxglove web:
drag the `.foxe` onto the extensions page. The publisher is sanitized in the
filename, so `umd-cdcl` appears as `umdcdcl`.

Re-run `./build.py` after any change to the messages.

## How messages are scanned

`scripts/generate_converters.py` reads every `.msg` in the package (recursively,
so `msg/radar/` is included), then walks each message's fields — following nested
messages and arrays — and matches them against a table of types Foxglove can
display. Anything that matches becomes a converter. No message is listed by hand.

The one constraint is that Foxglove accepts **one converter per
`(fromSchemaName, toSchemaName)` pair**, so each message has a single output slot
per target schema. Fields claim slots in declaration order, each taking the first
target it can use. A field that finds no free slot is printed as a note during
the build rather than silently dropped.

Converted output appears under the **original topic name** with a new schema.
Pick the topic in a panel and Foxglove offers it — no new topic names.

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
| `geometry_msgs/Quaternion` | `geometry_msgs/msg/Quaternion` | 3D / Raw |
| `geometry_msgs/Pose`, `Point`, `Vector3`, `Twist`, … | matching ROS schema | 3D / Raw |
| `uint8[] raw_audio` | `foxglove_msgs/msg/RawAudio` | Audio |
| Text-ish `string` fields | `foxglove_msgs/msg/Log` | Log |

Location and bounding-box fields merge into one output each, so all targets in a
message are drawn together. A `NavSatFix` at exactly `0, 0` is treated as
unlocalized and dropped. Every output derived from one message carries the root
message's `header.stamp`, so an image and its annotations line up in the Image
panel.

## TargetBoxArray topic converters

`TargetBoxArray.uav_target_boxes` carries three alternative localizations per
target (altimeter plane, gimbal plane, rangefinder). They all want the same
GeoJSON slot, and one schema converter cannot render them as three independently
toggleable Map layers — so these three, and only these three, use **topic
converters**, which produce real new topics:

```text
/uas#/target_locations/altimeter
/uas#/target_locations/gimbal
/uas#/target_locations/rangefinder
```

Everything else on the message — source image, bounding boxes, UAV location,
local pose, gimbal attitude — stays on schema converters and is therefore
available on *every* `TargetBoxArray` topic, including ones nobody configured.

Topic converters must name their input topics, so they are listed in
`TOPIC_RULES` at the top of `scripts/generate_converters.py`. Currently
`/uas1..4/target_locations` and `/uas1..4/tf_localization/localized`. To cover
another TBA topic, add it to that list. Which *fields* get split is not
hard-coded: the generator splits only fields that contend for a slot, reading the
leaves from `TargetBox.msg`, so a fourth localization would add a fourth topic on
its own.

Note for existing layouts: the UAV location, local pose and gimbal attitude used
to be separate `/uas#/target_locations/...` topics. They are now on the parent
topic under their own schema — select `/uas#/target_locations` in the panel. The
three localization topics above are unchanged.
