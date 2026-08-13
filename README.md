# CDCL Foxglove Converters

One Foxglove extension that makes `cdcl_umd_msgs` messages displayable in
Foxglove's built-in panels. The converters are generated from the `.msg` files,
so the extension always matches the messages on disk.

## Quickstart

```bash
# build
./build.py
```

Then fully quit and reopen Foxglove Studio. Re-run after any message change.

```bash
# if using mcaps, just launch foxglove, open the mcap and you're good to go
foxglove-studio # or using desktop or web app

# if using db3s, launch foxglove from sourced environment and youre good to go
foxglove-studio # from sourced environment only
```

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
(Foxglove **Settings → Extensions → Install from file**). The archive drops the
hyphen from the `umd-cdcl` publisher name while the installed directory keeps it
(`umd-cdcl.cdcl-converters-1.0.0`); both spellings are correct.

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
| Casualty locations, anchored to a fiducial | `foxglove_msgs/msg/SceneUpdate` | 3D |

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

## Casualty markers in the 3D panel

```text
/known_casualty_locations/markers    foxglove_msgs/msg/SceneUpdate
```

One labelled sphere per casualty, so they can be seen in the **3D** panel rather
than only on the Map. The Map view is unaffected — the same fields still feed the
GeoJSON converter.

The 3D panel is Cartesian while a `NavSatFix` is geodetic, so the fixes are
resolved into local ENU metres against an origin. That origin is per-mission and
arrives on a **second topic**, `/launch_zone_fiducial` — which is why this is a
topic converter: a schema converter only ever sees one message.

Two consequences worth knowing:

- **The markers are emitted into `d3_fiducial_offset`**, so they share the drones'
  TF tree. That frame is presently an identity child of `uas3_home_position`, which
  means the placement is only correct while the origin fix coincides with UAS3's
  home position — a stopgap until a shared `fiducial` frame exists. Change
  `frame_id` in the rule to retarget. If the panel is empty, a display frame with
  no path to this one is the usual cause.
- **Nothing is drawn until the first fiducial arrives.** The first valid fix is
  kept for the session and later ones ignored, so the frame never drifts; a
  guessed origin would place markers wrongly without looking wrong. Seeking to a
  point before the first fiducial therefore shows an empty scene.

Add a message to `SCENE_RULES` in `scripts/generate_converters.py` to give another
schema the same treatment. The geodetic fields are found in the `.msg` files; the
rule only names the topics, the origin topic and the frame.
