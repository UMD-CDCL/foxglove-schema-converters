# Foxglove Schema Converters

Generates Foxglove extensions that expose custom CDCL ROS 2 messages to Foxglove's
built-in panels, from the message definitions in:

```text
~/ros2_ws/src/cdcl_umd_msgs        (host)
/ros_ws/src/cdcl_umd_msgs          (dev container)
```

Nothing is hand-written per message. Point the generators at a `.msg` tree and
every displayable field — images, NavSat fixes, bounding boxes, odometry,
quaternions, audio, text — becomes a converter.

## Schema converters first, topic converters only when forced

**Schema converters** are keyed by message type. One registration covers every
topic that carries that schema, now and in the future, so they are always
preferred.

The one hard limit is that Foxglove accepts **exactly one converter per
`(fromSchemaName, toSchemaName)` pair**. A message therefore has a fixed set of
output "slots", one per target schema. That is fine until a single message
carries several fields that need the *same* target and must stay visually
separate — `TargetBoxArray.uav_target_boxes` holds three alternative
localizations per target (altimeter plane, gimbal plane, rangefinder), and one
GeoJSON slot cannot render them as three independently toggleable Map layers.

**Topic converters** solve exactly that, at the cost of naming their input
topics explicitly. They are generated only for fields that genuinely contend for
a slot; see [config/converters.json](config/converters.json).

Fields are allocated to slots in declaration order, each taking the first target
it can use:

| Field | Target | Notes |
| --- | --- | --- |
| First `NavSatFix` | `sensor_msgs/msg/NavSatFix` | Rendered natively by the Map panel |
| Further location fields | `foxglove_msgs/msg/GeoJSON` | Merged into one feature collection |
| `NavSatFix[]` named like a boundary | GeoJSON `Polygon` | `fence`, `zone`, `domain`, `polygon`, … |
| `Image` / `CompressedImage` | matching `sensor_msgs` schema | |
| `vision_msgs/BoundingBox2D` | `foxglove_msgs/msg/ImageAnnotations` | All boxes merged into one overlay |
| `uint8[] raw_audio` | `foxglove_msgs/msg/RawAudio` | |
| Text-ish `string` fields | `foxglove_msgs/msg/Log` | Merged into one labelled log |
| Odometry, Pose, Quaternion, Vector3, … | matching ROS schema | One exclusive slot each |

A field that finds no free slot is **reported, never silently dropped**. Run
`npm run audit` to see them; each one is a candidate for a topic-converter rule.

## Repository layout

```text
.
├── config/
│   └── converters.json              # the only policy file: which fields need their own topic
├── shared/
│   └── converterRuntime.ts          # all conversion logic, hand-written and type-checked
├── scripts/
│   ├── rosmsg.py                    # .msg parsing, indexing, field classification
│   ├── converter_config.py          # config loading + the contention rule
│   ├── emit.py                      # generated-file helpers
│   ├── generate_schema_converters.py
│   ├── generate_topic_converters.py
│   ├── audit_convertible_fields.py  # what is exposed, and what is blocked
│   ├── test_converter_runtime.mjs   # behavioural tests over the real converters
│   ├── bootstrap.sh
│   └── refresh_foxglove_extension.sh
├── cdcl-schema-converters/
│   └── src/generated/               # copied runtime + generated spec data
└── cdcl-topic-converters/
    └── src/generated/
```

The generators emit **data only**. Every conversion behaviour lives in
`shared/converterRuntime.ts`, which is copied into both extensions and compiled
by `tsc`, so the logic is type-checked and testable rather than hidden in Python
string templates.

## Commands

```bash
npm run generate      # regenerate both extensions from the .msg tree
npm run build         # generate + package both .foxe extensions
npm run audit         # report every convertible field and every blocked one
npm test              # behavioural tests over the generated converters
npm run lint
```

The generators find the message tree automatically (dev-container path first,
then `~/ros2_ws`). Override with an argument or `$CDCL_MSG_ROOTS`:

```bash
python3 scripts/generate_schema_converters.py ~/ros2_ws/src/cdcl_umd_msgs/msg
CDCL_MSG_ROOTS=/path/to/pkg/msg npm run generate
```

Multiple message packages can be indexed at once by passing several paths.

## Adding new messages

1. Add or edit the `.msg` file.
2. `npm run generate` — new convertible fields become converters immediately.
3. `npm run audit` — check nothing you wanted is listed as `BLOCKED`.
4. If something is blocked, add a rule to `config/converters.json` naming the
   schema, the field-path prefix, and the topics to split it on.
5. `npm run build`, then `./scripts/refresh_foxglove_extension.sh`.

Subdirectories are searched recursively, so `msg/radar/*.msg` is picked up.
Adding a fourth localization to `TargetBox.msg` automatically produces a fourth
output topic — the split leaves are read from the message definitions, not
hard-coded.

## What TargetBoxArray produces

Every `TargetBoxArray` topic gets these from schema converters, with **no
configuration** — including topics not listed anywhere, such as
`/uas1/target_detections/vlm`. In Foxglove they appear under the original topic
name; pick the panel and it will offer the topic:

| Panel | Topic | Schema |
| --- | --- | --- |
| Image | `/uas#/target_locations` | `sensor_msgs/msg/CompressedImage` |
| Image annotations | `/uas#/target_locations` | `foxglove_msgs/msg/ImageAnnotations` |
| Map | `/uas#/target_locations` | `sensor_msgs/msg/NavSatFix` (UAV paused location) |
| 3D | `/uas#/target_locations` | `nav_msgs/msg/Odometry` (UAV local pose) |
| 3D / Raw | `/uas#/target_locations` | `geometry_msgs/msg/Quaternion` (gimbal attitude) |

Only the per-target localizations need their own topics, one Map layer each:

```text
/uas#/target_locations/altimeter
/uas#/target_locations/gimbal
/uas#/target_locations/rangefinder
```

The same three are generated for `/uas#/tf_localization/localized`. To cover
another TBA topic, add it to `topics` in `config/converters.json`.

### Changed from the previous layout

These generated topics no longer exist. The data moved onto the parent topic
under a different schema, so select the parent topic in the panel instead:

| Removed topic | Now |
| --- | --- |
| `/uas#/target_locations/uav_paused_location` | `/uas#/target_locations` as `sensor_msgs/msg/NavSatFix` |
| `/uas#/target_locations/uav_local_pose` | `/uas#/target_locations` as `nav_msgs/msg/Odometry` |
| `/uas#/target_locations/gimbal_attitude_quaternion` | `/uas#/target_locations` as `geometry_msgs/msg/Quaternion` |
| the same three under `/uas#/target_detections/…` | `/uas#/target_detections` under those schemas |

Saved Foxglove layouts referencing the removed names need re-pointing. The three
localization topics kept their names.

## Timestamps

All outputs derived from one input message carry the **root message's**
`header.stamp` (then `stamp`, then the message event time). An extracted image
and its bounding-box annotations are separate topics and only line up in the
Image panel if their timestamps match exactly.

## Notes on data quality

* A `NavSatFix` at exactly `0, 0` is treated as unlocalized and dropped, rather
  than plotted in the Gulf of Guinea.
* An image with no bytes produces no message.
* A GeoJSON converter still emits an empty `FeatureCollection` when nothing is
  localized, so the topic stays visible in the topic list.
* `vision_msgs/BoundingBox2D` is read in both the current (`center.position.x`)
  and older (`center.x`) layouts.

## Development environment

This repository uses a VS Code Dev Container (`.devcontainer/`), which mounts the
message package at `/ros_ws/src/cdcl_umd_msgs`. Open the repo and reopen in
container:

```bash
code ~/foxglove-schema-converters
```

Then, inside the container:

```bash
cd /workspaces/foxglove-schema-converters
npm run build
```

Packaged extensions are written to:

```text
cdcl-schema-converters/umdcdcl.cdcl-schema-converters-*.foxe
cdcl-topic-converters/umdcdcl.cdcl-topic-converters-*.foxe
```

Foxglove tooling sanitizes the publisher name in archive filenames, so
`umd-cdcl` appears as `umdcdcl`.

## Installing into Foxglove Desktop (snap)

Copying the unpacked extension into the snap profile is more reliable for local
development than uploading `.foxe` files, because it avoids cloud/org upload
permissions. After building, run **on the host**:

```bash
cd ~/foxglove-schema-converters
./scripts/refresh_foxglove_extension.sh
```

Expected output:

```text
Success! Fully quit and reopen Foxglove Studio to reload extensions.
```

Then fully quit and reopen Foxglove Studio. The script installs both extensions
into `~/snap/foxglove-studio/current/.foxglove-studio/extensions/`.

Do not use `npm run local-install` from inside the dev container: it installs
into the container's Foxglove profile, not the host user's.

To update Foxglove itself:

```bash
sudo snap refresh foxglove-studio
```

## Outstanding

* `raw_audio` encoding metadata is still assumed (`pcm-s16`, 48 kHz, mono) in
  `shared/converterRuntime.ts`. Confirm sample rate, channel count and bit depth
  against a real capture before trusting the audio output.
* `sensor_msgs/Range` (`TargetBoxArray.rangefinder_dist`) has no Foxglove display
  schema. Plot it directly from the raw topic; no converter is needed.
