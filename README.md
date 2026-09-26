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
| `sensor_msgs/Image` or `CompressedImage` | both image schemas — see below | Image |
| Track array (`TrackArray.tracks`) | `foxglove_msgs/msg/GeoJSON` | Map |
| First `sensor_msgs/NavSatFix` | `sensor_msgs/msg/NavSatFix` | Map |
| Any further location fields | `foxglove_msgs/msg/GeoJSON` | Map |
| `NavSatFix[]` named like a boundary | `foxglove_msgs/msg/GeoJSON` (polygon) | Map |
| …named like a field of view | `foxglove_msgs/msg/GeoJSON` (unfilled outline) | Map |
| `vision_msgs/BoundingBox2D` | `foxglove_msgs/msg/ImageAnnotations` | Image |
| `nav_msgs/Odometry` | `nav_msgs/msg/Odometry` | 3D |
| `geometry_msgs/Quaternion`, `Pose`, `Point`, `Vector3`, … | matching ROS schema | 3D / Raw |
| `uint8[] raw_audio` | `foxglove_msgs/msg/RawAudio` | Audio |
| Text-ish `string` fields | `foxglove_msgs/msg/Log` | Log |

Location and bounding-box fields merge into one output each, so all targets in a
message draw together. A `NavSatFix` at exactly `0, 0` is treated as unlocalized
and dropped.

A ring whose name says field of view (`OUTLINE_HINTS` in the generator, matched
against the message and field name — `CameraFOV.fov_polygon` today) is drawn as
an unfilled outline rather than a filled polygon: an FOV frames what is under it,
so a fill would tint, and take the clicks of, the detections it is there to put
in context. Any single field can be switched over the same way through
`GEOJSON_OVERRIDES`, which is how `Geofence.coordinates` is unfilled; the other
boundary rings (`search_domain`, `launch_zone`, `exclusion_zones`) stay filled. Every output from one message carries the root message's
`header.stamp`, so an image and its annotations line up in the Image panel.

## Images: raw and compressed

What a publisher puts on the wire is not settled by the `.msg` — CDCL nodes fill
a declared `sensor_msgs/Image` with JPEG bytes (`TargetBoxArray.source_img` among
them). So an image field claims **both** image slots: its own schema first, and
the other one too when no second image field wants it.

Each converter then looks at the bytes and stays silent unless they are its kind,
so the Image panel is offered exactly the schema that can decode the frame.
Classification, in order: a codec named in `encoding` (`jpeg`, or a ROS 1
`"bgr8; jpeg compressed bgr8"`), then the payload's size against `height × step`,
then the container's magic bytes, and a message with no `encoding` at all is
compressed with `format` (defaulting to JPEG). Raw output gets a `step` derived
from the encoding or the payload when the publisher left it at 0.

## Tracks on the Map panel

`TrackArray` is drawn by `TRACK_RULES` in the generator rather than by the
generic walk, because a track is more than a point: it has a covariance and a
velocity. It produces three outputs.

**One topic per track id**, on `foxglove_msgs/msg/LocationFix` — Foxglove's own
point schema, the one that takes a position covariance alongside a covariance
*type*. The leading 2×2 `[east, north]` block of the track's 4×4 covariance is
widened into the 3×3 matrix and flagged `KNOWN` (3), keeping whatever vertical
variance the fix already carried. A track whose covariance field is empty falls
back to the fix's own 3×3 estimate; with neither, the matrix stays zeros and is
flagged `UNKNOWN` (0) rather than being passed off as certainty.

The Map panel reads more than the position off one of these, so the fix carries
what it looks for:

| Field | Shape | What the panel does with it |
| --- | --- | --- |
| `position_covariance` + `_type` | 9 floats, `KNOWN` | draws the 1-sigma ellipse, from the leading 2×2 block |
| `velocity` | `{x: east, y: north}` m/s | speed and bearing in the tooltip |
| `heading` | radians, clockwise from north | orients the arrowhead marker |
| `color` | `{r, g, b, a}`, 0..1 floats | colors the marker — **a CSS string renders black** |
| `metadata` | `[{key, value}]` strings | the tooltip's rows |

`speed_mps`, `heading_deg`, `track_id` and the ellipse semi-axes are also on the
message as plain numbers, which is what the Plot panel wants.

Only tracks at `TrackState.STATUS_ACTIVE` are emitted here and on the combined
layer; the per-status layers below are where the other states are visible. A
message whose status field is missing entirely is drawn in full, so a renamed
field empties nothing.

A track that goes inactive, or drops out of the array, still publishes on its
topic — a fix with `NaN` coordinates. It has to: the Map panel redraws the last
message it saw on a topic on every frame, so a converter that simply stayed
silent would leave the stale marker frozen on the map indefinitely. A fix whose
latitude or longitude is not finite is skipped before it reaches the map, which
is what clears it.

Numeric fields are read through a helper that accepts typed arrays: a fixed-size
`float64[16]` arrives as a `Float64Array`, which `Array.isArray` rejects even
though Raw Messages prints it as a list.

```text
/active_tracks/00 … /active_tracks/63    foxglove_msgs/msg/LocationFix
```

A converter's output topic is fixed when it registers, so ids `0..max_id` are
declared up front — `max_id` is currently **63**. A track whose id lands outside
that range still appears in the layers but gets no topic of its own. Ids are
zero-padded to the width of `max_id` so the topic list sorts in id order rather
than lexically.

The tracker hands out a fresh id per newly born target instead of reusing them,
so `max_id` is a budget for how long a run can get, not for how many targets are
up at once: a 21-minute bench bag reached id 34 while never holding more than 14
tracks in the array. Raise `max_id` in `TRACK_RULES` for longer runs.

**One GeoJSON layer per tracker status** (topic converters), so a track is
visible in every state, not only while it is active:

```text
/active_tracks/tentative  /active_tracks/active
/active_tracks/dormant    /active_tracks/inactive
```

Each draws what the combined layer draws — ellipse, arrow and marker per track —
narrowed to the one status. The list comes from the `STATUS_*` constants of
`TrackState.msg`, so a state added there gets its own topic on the next
generator run without anything being listed by hand. Toggle a layer on in the
Map panel to see, say, the tentative tracks the tracker has not confirmed yet:
on that same bench bag, 34 distinct ids appear as tentative while only 11 ever
reach active.

**The whole array as one GeoJSON layer** on `/active_tracks` itself (a schema
converter — select the topic in the Map panel and pick the GeoJSON schema). Each
track draws three features in the same per-id color the LocationFix topic
reports:

| Feature | Geometry | What it is |
| --- | --- | --- |
| `covariance` | Polygon | uncertainty ellipse: semi-axes are 1 standard deviation of the position block |
| `heading` | LineString | arrow along the velocity — 10 s of travel, clamped to 8–250 m, omitted below 0.15 m/s |
| `position` | Point | the fix itself, drawn last so it stays on top |

Every feature carries `track_id`, `color`, `speed_mps`, `heading_deg`, the ellipse
semi-axes and the track's own scalar fields, so a click reports the same numbers
the per-id topic publishes. The panel paints a GeoJSON layer in its own per-topic
color rather than honoring per-feature styling, so the per-track colors show up
here as properties, not as paint — the per-id topics are where they render.

Ellipse geometry is a local flat-earth offset from the fix, exact to well under a
metre at these scales; UTM grid convergence between the tracker's
easting/northing axes and true ENU is ignored. A covariance of all zeros, or a
sigma over 20 km, draws no ellipse — the marker still draws. The ellipse is the
tracker's own uncertainty at 1:1, so a converged track with a half-metre sigma is
a half-metre ellipse: `sigma_major_m` tells you what to expect before you go
looking for it.

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

The altimeter-plane localization is the default: besides its topic above, it also
keeps the message's GeoJSON slot, so selecting a `TargetBoxArray` topic in the
Map panel draws targets straight away — including on topics that are not in the
list below. The other two are topic-only, since the slot fits just one. Which one
is the default is `default_paths` in the same rule.

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
