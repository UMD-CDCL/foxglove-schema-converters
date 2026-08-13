#!/usr/bin/env python3
"""Generates Foxglove converter specs from a ROS 2 message package.

Scans every .msg in the package, works out which fields Foxglove can display,
and writes cdcl-converters/src/converterSpecs.ts. All conversion logic lives in
converterRuntime.ts; this script emits data only.

    python3 scripts/generate_converters.py [PACKAGE_DIR] [-o OUT]
"""
from __future__ import annotations

import argparse
import json
import re
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT = REPO_ROOT / "cdcl-converters" / "src" / "converterSpecs.ts"
DEFAULT_PACKAGES = (
    "/pkg",  # where build.py mounts the package inside the container
    "~/ros2_ws/src/cdcl_umd_msgs",
    "~/ros_workspaces/cdcl_ws/src/cdcl_umd_msgs",
)

MAX_DEPTH = 4

ROS_PRIMITIVES = {
    "bool", "byte", "char", "float32", "float64", "int8", "uint8", "int16",
    "uint16", "int32", "uint32", "int64", "uint64", "string", "wstring",
}

POLYGON_HINTS = ("polygon", "fence", "domain", "zone", "boundary", "bounds", "perimeter")
TEXT_HINTS = ("transcript", "caption", "text", "description", "summary", "message")
AUDIO_FIELDS = {"raw_audio", "audio", "audio_data", "pcm", "samples"}
AUDIO_STAMP_FIELDS = ("audio_start", "audio_start_time", "start_time", "stamp")
# Ids are deliberately excluded: long and uninformative drawn over a video frame.
LABEL_HINTS = ("class", "label", "name", "confidence", "score")

COLORS = (
    "#e6194b", "#3cb44b", "#4363d8", "#f58231", "#911eb4", "#46f0f0",
    "#f032e6", "#bcf60c", "#fabebe", "#008080", "#e6beff", "#9a6324",
    "#808000", "#800000", "#aaffc3", "#ffd8b1", "#000075", "#a9a9a9",
)

# ---------------------------------------------------------------------------
# Target registry
# ---------------------------------------------------------------------------
# Foxglove registers at most one converter per (fromSchemaName, toSchemaName)
# pair, so each message has one output slot per target schema. `aggregate`
# targets merge many fields into a single converter; the rest are exclusive and
# taken by the first field that claims them.

GEOJSON = ("geojson", "foxglove_msgs/msg/GeoJSON", True)
ANNOTATIONS = ("image_annotations", "foxglove_msgs/msg/ImageAnnotations", True)
LOG = ("log", "foxglove_msgs/msg/Log", True)
AUDIO = ("audio", "foxglove_msgs/msg/RawAudio", False)


def _through(schema: str) -> tuple[str, str, bool]:
    return ("passthrough", schema, False)


# Options are tried in order; a NavSatFix prefers the native pass-through (the
# Map panel renders it directly) and falls back to GeoJSON once that is taken.
SCALAR_TARGETS: dict[str, tuple] = {
    "sensor_msgs/Image": (("image", "sensor_msgs/msg/Image", False),),
    "sensor_msgs/CompressedImage": (("image", "sensor_msgs/msg/CompressedImage", False),),
    "sensor_msgs/NavSatFix": (("navsatfix", "sensor_msgs/msg/NavSatFix", False), GEOJSON),
    "gps_msgs/GPSFix": (("navsatfix", "gps_msgs/msg/GPSFix", False), GEOJSON),
    "vision_msgs/BoundingBox2D": (ANNOTATIONS,),
    "sensor_msgs/Imu": (_through("sensor_msgs/msg/Imu"),),
    "sensor_msgs/PointCloud2": (_through("sensor_msgs/msg/PointCloud2"),),
    "sensor_msgs/LaserScan": (_through("sensor_msgs/msg/LaserScan"),),
    "nav_msgs/Odometry": (_through("nav_msgs/msg/Odometry"),),
    "nav_msgs/Path": (_through("nav_msgs/msg/Path"),),
    "geometry_msgs/Pose": (_through("geometry_msgs/msg/Pose"),),
    "geometry_msgs/PoseStamped": (_through("geometry_msgs/msg/PoseStamped"),),
    "geometry_msgs/PoseWithCovariance": (_through("geometry_msgs/msg/PoseWithCovariance"),),
    "geometry_msgs/Point": (_through("geometry_msgs/msg/Point"),),
    "geometry_msgs/PointStamped": (_through("geometry_msgs/msg/PointStamped"),),
    "geometry_msgs/Quaternion": (_through("geometry_msgs/msg/Quaternion"),),
    "geometry_msgs/Vector3": (_through("geometry_msgs/msg/Vector3"),),
    "geometry_msgs/Twist": (_through("geometry_msgs/msg/Twist"),),
    "geometry_msgs/Transform": (_through("geometry_msgs/msg/Transform"),),
    "visualization_msgs/Marker": (_through("visualization_msgs/msg/Marker"),),
    "visualization_msgs/MarkerArray": (_through("visualization_msgs/msg/MarkerArray"),),
}

# Used when a field resolves to many values (declared array, or reached through
# one). Types absent here are reported rather than truncated to their first item.
ARRAY_TARGETS: dict[str, tuple] = {
    "sensor_msgs/NavSatFix": (GEOJSON,),
    "gps_msgs/GPSFix": (GEOJSON,),
    "vision_msgs/BoundingBox2D": (ANNOTATIONS,),
    "geometry_msgs/Pose": (("pose_array", "geometry_msgs/msg/PoseArray", False),),
    "geometry_msgs/PoseStamped": (("pose_array", "geometry_msgs/msg/PoseArray", False),),
}

# ---------------------------------------------------------------------------
# Topic converter policy
# ---------------------------------------------------------------------------
# The only case a schema converter cannot handle: several fields of one message
# competing for the same output slot that must stay separately toggleable in a
# panel. Each rule names the paths it claims; the leaf fields are discovered from
# the .msg files, so a new localization in TargetBox.msg adds a topic by itself.
# Topic converters must name their inputs, so new topics do have to be listed.

TOPIC_RULES = [
    {
        # uav_target_boxes holds three alternative localizations per target.
        "schema": "cdcl_umd_msgs/msg/TargetBoxArray",
        "split_paths": ["uav_target_boxes"],
        "topics": [
            f"/uas{n}/{suffix}"
            for n in (1, 2, 3, 4)
            for suffix in ("target_locations", "tf_localization/localized")
        ],
        # Pins the suffixes existing Foxglove layouts already reference.
        "keys": {
            "uav_target_boxes.target_location_altimeter_plane": "altimeter",
            "uav_target_boxes.target_location_gimbal_plane": "gimbal",
            "uav_target_boxes.target_location_rangefinder": "rangefinder",
        },
    },
]

# ---------------------------------------------------------------------------
# 3D marker policy
# ---------------------------------------------------------------------------
# The 3D panel is Cartesian, so geodetic fixes have to be resolved against a
# local origin. That origin is per-mission and arrives on its own topic, and
# reading two topics is something only a topic converter can do.
#
# Listed per schema rather than driven by field type: a NavSatFix target would
# hand a 3D view to every message that carries one, and each needs an origin
# topic chosen for it. Fields keep their schema converters too, so the Map view
# of the same message survives.
#
# Required keys: schema, topics, origin_topic, frame_id, entity_id, suffix.
# Optional: paths (restrict which geodetic fields are drawn, by dotted prefix),
# labels (per-path marker name), label (prefix for paths `labels` does not name),
# shape ("sphere" or "cube"), properties (sibling fields shown on the marker,
# instead of every primitive sibling), accumulate_suffix (a second output topic
# whose markers pile up across the run rather than replacing one another).

SCENE_RULES = [
    {
        "schema": "cdcl_umd_msgs/msg/KnownCasualtyLocations",
        "topics": ["/known_casualty_locations"],
        "origin_topic": "/launch_zone_fiducial",
        # Markers are placed in ENU metres from the origin fix, so frame_id has to
        # name a frame already in the drones' tf tree or they float unconnected.
        # d3_fiducial_offset is currently an identity child of uas3_home_position,
        # so this holds only while the origin fix coincides with UAS3's home
        # position. Retarget to the shared "fiducial" frame once that exists.
        "frame_id": "d3_fiducial_offset",
        "entity_id": "known_casualties",
        "suffix": "markers",
        "label": "Casualty",
    },
    {
        # Each TargetBox carries up to three alternative localizations of the same
        # target; every one that is set gets its own marker, so the three methods
        # can be compared in place. The same three feed the Map layers, and colours
        # are hashed from the field path, so a marker matches its Map layer.
        "schema": "cdcl_umd_msgs/msg/TargetBoxArray",
        "topics": [f"/uas{n}/target_locations" for n in (1, 2, 3, 4)],
        "origin_topic": "/launch_zone_fiducial",
        "frame_id": "d3_fiducial_offset",
        "entity_id": "uav_targets",
        # `markers` holds the latest message's targets; `markers_all` keeps every
        # message's, so a run's detections build up into a single picture.
        "suffix": "markers",
        "accumulate_suffix": "markers_all",
        # Targets only: uav_gps_location is the drone's own fix, not a detection.
        "paths": ["uav_target_boxes"],
        "labels": {
            "uav_target_boxes.target_location_altimeter_plane": "Altimeter",
            "uav_target_boxes.target_location_gimbal_plane": "Gimbal",
            "uav_target_boxes.target_location_rangefinder": "Rangefinder",
        },
        # Cubes so targets stay distinguishable from the casualty spheres when
        # both layers are shown at once.
        "shape": "cube",
        # Every primitive sibling would be carried by default; these are the two
        # worth reading off a marker floating over a scene.
        "properties": ["detection_class", "detection_confidence"],
    },
]


# ---------------------------------------------------------------------------
# Message model
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Field:
    name: str
    base_type: str
    is_array: bool


@dataclass(frozen=True)
class Convertible:
    schema: str
    path: tuple[str, ...]
    base_type: str
    through_array: bool
    options: tuple
    siblings: tuple[str, ...]
    container: str

    @property
    def dotted(self) -> str:
        return ".".join(self.path)

    @property
    def leaf(self) -> str:
        return self.path[-1]

    @property
    def color(self) -> str:
        total = 0
        for char in f"{self.schema}.{self.dotted}":
            total = (total * 31 + ord(char)) % 1000003
        return COLORS[total % len(COLORS)]

    @property
    def geometry(self) -> str:
        """Only multi-valued fields can be rings; the message name is a hint too,
        so Geofence.coordinates is recognised though the field name is generic."""
        if not self.through_array:
            return "point"
        haystack = f"{self.schema.split('/')[-1]}.{self.dotted}".lower()
        return "polygon" if any(h in haystack for h in POLYGON_HINTS) else "point"


def humanize(name: str) -> str:
    return name.replace("_", " ").strip().capitalize() or name


def normalize_type(raw: str, package: str) -> tuple[str, bool]:
    """Returns (base_type, is_array) as `pkg/Type` or a primitive."""
    base = raw.strip()
    is_array = False

    match = re.match(r"^(.*?)\[[^\]]*\]$", base)  # Type[], Type[5], Type[<=5]
    if match:
        base, is_array = match.group(1), True

    base = base.split("<=")[0].strip()  # string<=20

    if base in ROS_PRIMITIVES:
        return base, is_array

    parts = base.split("/")
    if len(parts) == 3 and parts[1] == "msg":
        return f"{parts[0]}/{parts[2]}", is_array
    if len(parts) == 2:
        return base, is_array
    return f"{package}/{base}", is_array  # bare `TargetBox` is same-package


def parse_msg(path: Path, package: str) -> tuple[Field, ...]:
    fields = []

    for raw_line in path.read_text().splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue

        parts = re.split(r"\s+", line)
        if len(parts) < 2:
            continue

        type_str, name = parts[0], parts[1]
        # Skip constants: `uint8 ACTIVE = 1` and `uint8 ACTIVE=1`.
        if "=" in name or (len(parts) > 2 and parts[2].startswith("=")):
            continue

        base_type, is_array = normalize_type(type_str, package)
        fields.append(Field(name=name, base_type=base_type, is_array=is_array))

    return tuple(fields)


def load_package(package_dir: Path) -> tuple[str, dict[str, tuple[Field, ...]]]:
    """Indexes every .msg in the package, keyed by `pkg/Type`."""
    name = package_dir.resolve().name
    package_xml = package_dir / "package.xml"

    if package_xml.exists():
        match = re.search(r"<name>\s*([^<\s]+)\s*</name>", package_xml.read_text())
        if match:
            name = match.group(1)

    msg_root = package_dir / "msg"
    if not msg_root.is_dir():
        raise SystemExit(f"No msg/ directory in {package_dir}")

    # rglob, not glob: subdirectories such as msg/radar/ hold real messages.
    index = {
        f"{name}/{msg_file.stem}": parse_msg(msg_file, name)
        for msg_file in sorted(msg_root.rglob("*.msg"))
    }

    return name, index


def walk(schema: str, index: dict[str, tuple[Field, ...]]) -> tuple[list[Convertible], list[str]]:
    """Depth-first walk yielding every convertible field reachable from `schema`."""
    found: list[Convertible] = []
    skipped: list[str] = []

    def visit(type_name: str, prefix: tuple, depth: int, in_array: bool, stack: tuple) -> None:
        fields = index[type_name]
        siblings = tuple(f.name for f in fields if f.base_type in ROS_PRIMITIVES and not f.is_array)

        for f in fields:
            path = prefix + (f.name,)
            through_array = in_array or f.is_array
            options = (ARRAY_TARGETS if through_array else SCALAR_TARGETS).get(f.base_type, ())

            if not options and f.base_type == "uint8" and f.is_array and f.name in AUDIO_FIELDS:
                options, through_array = (AUDIO,), False
            elif (
                not options
                and f.base_type == "string"
                and not through_array
                and any(hint in f.name.lower() for hint in TEXT_HINTS)
            ):
                options = (LOG,)

            if options:
                found.append(
                    Convertible(schema, path, f.base_type, through_array, options, siblings, type_name)
                )
                continue

            if through_array and f.base_type in SCALAR_TARGETS:
                skipped.append(f"{'.'.join(path)} ({f.base_type}): no multi-value target")
                continue

            nested = f.base_type
            if nested in index and depth < MAX_DEPTH and nested not in stack:
                visit(nested, path, depth + 1, through_array, stack + (nested,))

    root = f"{schema.split('/')[0]}/{schema.split('/')[-1]}"
    visit(root, (), 0, False, (root,))
    return found, skipped


# ---------------------------------------------------------------------------
# Spec building
# ---------------------------------------------------------------------------


def label_fields(siblings: tuple[str, ...]) -> list[str]:
    ranked = [
        (rank, name)
        for name in siblings
        for rank, hint in enumerate(LABEL_HINTS)
        if hint in name.lower()
    ]
    ranked.sort(key=lambda item: item[0])
    return [name for _, name in ranked[:3]]


def property_fields(item: Convertible) -> list[str]:
    """Primitive siblings of the field, carried alongside it as context."""
    return [name for name in item.siblings if name != item.leaf][:12]


def geojson_entry(item: Convertible, label: str) -> dict:
    entry = {
        "path": list(item.path),
        "label": label,
        "geometry": item.geometry,
        "color": item.color,
    }
    properties = property_fields(item)
    if properties:
        entry["propertyFields"] = properties
    return entry


def scene_entry(item: Convertible, label: str, rule: dict) -> dict:
    entry = {
        "path": list(item.path),
        "label": label,
        "color": item.color,
    }

    shape = rule.get("shape")
    if shape:
        entry["shape"] = shape

    chosen = rule.get("properties")
    properties = (
        [name for name in chosen if name in item.siblings]
        if chosen is not None
        else property_fields(item)
    )
    if properties:
        entry["propertyFields"] = properties

    return entry


def annotation_entry(item: Convertible) -> dict:
    return {
        "containerPath": list(item.path[:-1]),
        "bboxField": item.leaf,
        "labelFields": label_fields(item.siblings),
        "color": item.color,
    }


def build_op(item: Convertible, label: str, index: dict) -> dict:
    kind = item.options[0][0]

    if kind == "geojson":
        return {"kind": "geojson", "entries": [geojson_entry(item, label)]}
    if kind == "image_annotations":
        return {"kind": "image_annotations", "entries": [annotation_entry(item)]}
    if kind == "log":
        return {"kind": "log", "entries": [{"path": list(item.path), "label": label}]}

    op = {"kind": kind, "path": list(item.path)}

    if kind == "audio":
        by_name = {f.name: f for f in index.get(item.container, ())}
        for candidate in AUDIO_STAMP_FIELDS:
            field = by_name.get(candidate)
            if field is not None and field.base_type == "builtin_interfaces/Time":
                op["stampPath"] = list(item.path[:-1]) + [candidate]
                break

    return op


def labels_for(items: list[Convertible]) -> dict[str, str]:
    """Leaf name, widened to two segments when two fields would collide."""
    counts: dict[str, int] = {}
    for item in items:
        counts[item.leaf] = counts.get(item.leaf, 0) + 1

    return {
        item.dotted: humanize(item.leaf if counts[item.leaf] == 1 else " ".join(item.path[-2:]))
        for item in items
    }


def under_paths(item: Convertible, paths: list[str] | None) -> bool:
    """Whether the field sits at, or under, one of `paths`. None matches everything."""
    if paths is None:
        return True
    return any(item.dotted == p or item.dotted.startswith(f"{p}.") for p in paths)


def claimed_by_rule(rule: dict, items: list[Convertible]) -> list[Convertible]:
    """Fields under the rule's prefixes that genuinely need their own topic.

    A field needs one only when it *contends*: two or more fields under the
    prefix resolving to the same output schema cannot all be shown separately by
    one schema converter. A lone field (a bounding box, say) has no rival and is
    left on the schema converter, keeping topic converters to a minimum.
    """
    under = [item for item in items if under_paths(item, rule["split_paths"])]

    by_target: dict[str, list[Convertible]] = {}
    for item in under:
        by_target.setdefault(item.options[0][1], []).append(item)

    contended = {item.dotted for group in by_target.values() if len(group) > 1 for item in group}
    return [item for item in under if item.dotted in contended]


def derive_key(item: Convertible, group: list[Convertible], overrides: dict) -> str:
    """Output-topic suffix: the leaf with the prefix its siblings share removed."""
    if item.dotted in overrides:
        return overrides[item.dotted]

    token_lists = [other.leaf.split("_") for other in group]
    tokens = item.leaf.split("_")
    shared = 0

    if len(group) > 1:
        shortest = min(len(t) for t in token_lists)
        while shared < shortest - 1 and len({t[shared] for t in token_lists}) == 1:
            shared += 1

    return "_".join(tokens[shared:]) or item.leaf


GEO_FIX_TYPES = ("sensor_msgs/NavSatFix", "gps_msgs/GPSFix")


def scene_specs_for(rule: dict, found: list[Convertible], index: dict) -> list[dict]:
    """One SceneUpdate topic converter per topic the rule names, per mode.

    Every geodetic field the rule selects goes into a single entity, so the whole
    message toggles as one layer in the 3D panel. The fields are also left on
    their schema converters, keeping the Map view intact.
    """
    items = [
        item
        for item in found
        if item.base_type in GEO_FIX_TYPES and under_paths(item, rule.get("paths"))
    ]
    if not items:
        return []

    labels = labels_for(items)
    overrides = rule.get("labels", {})
    base = rule.get("label")

    def label_for(item: Convertible) -> str:
        if item.dotted in overrides:
            return overrides[item.dotted]
        if base is None:
            return labels[item.dotted]
        return base if len(items) == 1 else f"{base} {labels[item.dotted]}"

    root = f"{rule['schema'].split('/')[0]}/{rule['schema'].split('/')[-1]}"
    metadata = [
        f.name for f in index.get(root, ()) if f.base_type in ROS_PRIMITIVES and not f.is_array
    ]

    def op_for(accumulate: bool) -> dict:
        op = {
            "kind": "scene_update",
            "originTopic": rule["origin_topic"],
            "frameId": rule["frame_id"],
            "entityId": rule["entity_id"],
        }
        if accumulate:
            op["accumulate"] = True
        op["entries"] = [scene_entry(item, label_for(item), rule) for item in items]
        if metadata:
            op["metadataFields"] = metadata
        return op

    # The latest-only output first; a rule opts into the second, accumulating one.
    modes = [(rule["suffix"], False)]
    if rule.get("accumulate_suffix"):
        modes.append((rule["accumulate_suffix"], True))

    return [
        {
            "inputTopics": [topic, rule["origin_topic"]],
            "outputTopic": f"{topic}/{suffix}",
            "outputSchemaName": "foxglove_msgs/msg/SceneUpdate",
            "op": op_for(accumulate),
        }
        for topic in rule["topics"]
        for suffix, accumulate in modes
    ]


def build_specs(package: str, index: dict) -> tuple[list[dict], list[dict], list[str]]:
    schema_specs: list[dict] = []
    topic_specs: list[dict] = []
    notes: list[str] = []

    rules_by_schema: dict[str, list[dict]] = {}
    for rule in TOPIC_RULES:
        rules_by_schema.setdefault(rule["schema"], []).append(rule)

    scenes_by_schema: dict[str, list[dict]] = {}
    for rule in SCENE_RULES:
        scenes_by_schema.setdefault(rule["schema"], []).append(rule)

    for type_name in sorted(index):
        schema = f"{package}/msg/{type_name.split('/')[-1]}"
        found, skipped = walk(schema, index)
        notes.extend(f"{schema}: skipped {note}" for note in skipped)

        for rule in scenes_by_schema.pop(schema, []):
            specs = scene_specs_for(rule, found, index)
            if not specs:
                notes.append(f"{schema}: scene rule matched no geodetic field")
            topic_specs.extend(specs)

        # Fields a topic converter owns are removed from the schema converter, so
        # nothing is rendered twice.
        split: list[Convertible] = []
        for rule in rules_by_schema.get(schema, []):
            claimed = claimed_by_rule(rule, found)
            split.extend(claimed)

            for item in claimed:
                key = derive_key(item, claimed, rule["keys"])
                op = build_op(item, humanize(key), index)
                for topic in rule["topics"]:
                    topic_specs.append(
                        {
                            "inputTopics": [topic],
                            "outputTopic": f"{topic}/{key}",
                            "outputSchemaName": item.options[0][1],
                            "op": op,
                        }
                    )

        split_paths = {item.dotted for item in split}
        candidates = [item for item in found if item.dotted not in split_paths]

        exclusive: dict[str, Convertible] = {}
        aggregated: dict[str, list[Convertible]] = {}

        for item in candidates:
            for kind, to_schema, is_aggregate in item.options:
                if is_aggregate:
                    aggregated.setdefault(to_schema, []).append(item)
                    break
                if to_schema not in exclusive:
                    exclusive[to_schema] = item
                    schema_specs.append(
                        {
                            "fromSchemaName": schema,
                            "toSchemaName": to_schema,
                            "op": build_op(item, humanize(item.leaf), index),
                        }
                    )
                    break
            else:
                held = ", ".join(
                    f"{s} held by {exclusive[s].dotted}"
                    for _, s, _ in item.options
                    if s in exclusive
                )
                notes.append(f"{schema}: no free slot for {item.dotted} ({held})")

        for to_schema, items in aggregated.items():
            labels = labels_for(items)
            if to_schema == "foxglove_msgs/msg/GeoJSON":
                op = {"kind": "geojson", "entries": [geojson_entry(i, labels[i.dotted]) for i in items]}
            elif to_schema == "foxglove_msgs/msg/ImageAnnotations":
                op = {"kind": "image_annotations", "entries": [annotation_entry(i) for i in items]}
            else:
                op = {
                    "kind": "log",
                    "entries": [{"path": list(i.path), "label": labels[i.dotted]} for i in items],
                }

            schema_specs.append(
                {"fromSchemaName": schema, "toSchemaName": to_schema, "op": op}
            )

    # Anything left never matched a message: almost always a typo in the rule.
    for schema in scenes_by_schema:
        raise SystemExit(f"SCENE_RULES names {schema}, which is not in the package")

    schema_specs.sort(key=lambda s: (s["fromSchemaName"], s["toSchemaName"]))
    topic_specs.sort(key=lambda s: (s["inputTopics"][0], s["outputTopic"]))

    seen_pairs = {(s["fromSchemaName"], s["toSchemaName"]) for s in schema_specs}
    if len(seen_pairs) != len(schema_specs):
        raise SystemExit("Internal error: duplicate (fromSchemaName, toSchemaName) pair")

    seen_topics = {s["outputTopic"] for s in topic_specs}
    if len(seen_topics) != len(topic_specs):
        raise SystemExit("Internal error: duplicate topic converter output topic")

    return schema_specs, topic_specs, notes


def render(schema_specs: list[dict], topic_specs: list[dict]) -> str:
    return f"""// Generated by scripts/generate_converters.py. Do not edit by hand.
//
// Data only: every conversion behaviour lives in ./converterRuntime.ts.

import {{ SchemaConverterSpec, TopicConverterSpec }} from "./converterRuntime";

export const SCHEMA_CONVERTER_SPECS: readonly SchemaConverterSpec[] =
  {json.dumps(schema_specs, indent=2)};

export const TOPIC_CONVERTER_SPECS: readonly TopicConverterSpec[] =
  {json.dumps(topic_specs, indent=2)};
"""


def resolve_package(explicit: str | None) -> Path:
    candidates = [explicit] if explicit else list(DEFAULT_PACKAGES)

    for candidate in candidates:
        path = Path(candidate).expanduser()
        if (path / "msg").is_dir():
            return path

    raise SystemExit(
        "No ROS message package found. Searched: "
        + ", ".join(str(Path(c).expanduser()) for c in candidates)
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", nargs="?", help="ROS package directory containing msg/")
    parser.add_argument("-o", "--out", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    package_dir = resolve_package(args.package)
    package, index = load_package(package_dir)
    schema_specs, topic_specs, notes = build_specs(package, index)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(render(schema_specs, topic_specs))

    print(f"{package_dir}: {len(index)} messages -> {package}")
    print(f"{len(schema_specs)} schema converters, {len(topic_specs)} topic converters")

    for note in notes:
        print(f"  note: {note}")

    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
