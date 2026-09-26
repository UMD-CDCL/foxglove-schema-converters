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
DEFAULT_PACKAGES = ("/pkg", "~/ros2_ws/src/cdcl_umd_msgs")

MAX_DEPTH = 4

ROS_PRIMITIVES = {
    "bool", "byte", "char", "float32", "float64", "int8", "uint8", "int16",
    "uint16", "int32", "uint32", "int64", "uint64", "string", "wstring",
}

POLYGON_HINTS = ("polygon", "fence", "domain", "zone", "boundary", "bounds", "perimeter")
# Polygons matching these are drawn as an unfilled outline instead. A field of
# view frames what is under it, so a fill would tint — and take the clicks of —
# the very detections it is there to put in context.
OUTLINE_HINTS = ("fov", "field_of_view")
TEXT_HINTS = ("transcript", "caption", "text", "description", "summary", "message")
AUDIO_FIELDS = {"raw_audio", "audio", "audio_data", "pcm", "samples"}
AUDIO_STAMP_FIELDS = ("audio_start", "audio_start_time", "start_time", "stamp")
# Ids are deliberately excluded: long and uninformative drawn over a video frame.
LABEL_HINTS = ("class", "label", "name", "confidence", "score")

# Per-field overrides for the derived GeoJSON style, keyed by
# "schema_name.dotted.path". "line" draws a closed boundary with no fill, so
# markers underneath the ring stay clickable.
GEOJSON_OVERRIDES: dict[str, dict] = {
    "cdcl_umd_msgs/msg/Geofence.coordinates": {"geometry": "line", "color": "#ff8000"},
}

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

# An image field is offered on both image schemas. What a publisher actually
# puts on the wire is not settled by the .msg — CDCL nodes fill a declared
# sensor_msgs/Image with JPEG bytes — so the field takes its own schema's slot
# first and the other one too if nothing else wants it (see build_specs). At
# runtime each converter inspects the bytes and emits only if they are its kind.
IMAGE_RAW = ("image", "sensor_msgs/msg/Image", False)
IMAGE_COMPRESSED = ("image", "sensor_msgs/msg/CompressedImage", False)
IMAGE_TYPES = ("sensor_msgs/Image", "sensor_msgs/CompressedImage")


def _through(schema: str) -> tuple[str, str, bool]:
    return ("passthrough", schema, False)


# Options are tried in order; a NavSatFix prefers the native pass-through (the
# Map panel renders it directly) and falls back to GeoJSON once that is taken.
SCALAR_TARGETS: dict[str, tuple] = {
    "sensor_msgs/Image": (IMAGE_RAW, IMAGE_COMPRESSED),
    "sensor_msgs/CompressedImage": (IMAGE_COMPRESSED, IMAGE_RAW),
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
        # The localization to show by default. It keeps its topic converter like
        # the other two, and additionally stays on the schema converter, so a
        # TargetBoxArray topic nobody listed above — or a panel pointed at the
        # parent topic — still draws targets somewhere.
        "default_paths": ["uav_target_boxes.target_location_altimeter_plane"],
    },
]

# ---------------------------------------------------------------------------
# Track policy
# ---------------------------------------------------------------------------
# A track is more than a point — it carries a covariance and a velocity — so the
# message holding the array is drawn by this rule instead of the generic walk,
# and produces three outputs:
#
#   * one topic per track id, on Foxglove's own point schema, carrying the
#     position covariance and the flag that marks it as a real estimate;
#   * one GeoJSON layer per tracker status (/active_tracks/tentative and so on),
#     each drawing every track in that state — the states are read from the
#     `STATUS_*` constants of the track message, so a new one adds its own topic;
#   * the whole array, active tracks only, as one GeoJSON layer on the source
#     topic, drawing each track with the covariance ellipse, heading arrow and
#     the same per-id color the individual topics use.
#
# A converter's output topic is fixed when it registers, so the per-id topics
# need the ids enumerated up front. Ids from 0 to max_id are declared; a track
# whose id lands outside that range gets no topic of its own (raise max_id if
# the tracker hands out ids beyond it) but still appears in the layers. Ids are
# handed out as targets are born rather than reused, so max_id is a run-length
# budget, not a count of simultaneous tracks: a 21-minute bench bag reached id
# 34 with never more than 14 tracks in the array at once.
# Each per-id converter emits nothing on steps where its id is absent from the
# array, so unused topics stay empty rather than showing stale points.
#
# How any of it is drawn is in converterRuntime.ts; the rule only says which
# field is which.

TRACK_RULES = [
    {
        "schema": "cdcl_umd_msgs/msg/TrackArray",
        "topics": ["/active_tracks"],
        # foxglove.LocationFix is the Map panel's native point schema and the one
        # that takes a position covariance with a covariance *type* beside it.
        "output_schema": "foxglove_msgs/msg/LocationFix",
        "array_path": ["tracks"],
        "id_field": "track_id",
        "position_path": ["position"],
        # ENU m/s; becomes velocity, speed and heading on every output.
        "velocity_field": "velocity",
        # 4x4 [x_utm, y_utm, vx, vy]; the leading 2x2 block is the position block.
        "covariance_field": "covariance",
        # Only tracks at this status are drawn on the per-id topics and the
        # combined layer: TrackState.STATUS_ACTIVE. A message that has no such
        # field is drawn in full rather than hidden. Every other state is on its
        # own /active_tracks/<status> layer.
        "status_field": "status",
        "active_status": 1,
        "max_id": 63,
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
        so Geofence.coordinates is recognised though the field name is generic.
        A ring that is a field of view is drawn unfilled ("line")."""
        if not self.through_array:
            return "point"
        haystack = f"{self.schema.split('/')[-1]}.{self.dotted}".lower()
        if not any(h in haystack for h in POLYGON_HINTS):
            return "point"
        return "line" if any(h in haystack for h in OUTLINE_HINTS) else "polygon"


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


def parse_msg(path: Path, package: str) -> tuple[tuple[Field, ...], dict[str, str]]:
    """The message's fields, and its constants as raw `NAME -> value` text."""
    fields = []
    constants: dict[str, str] = {}

    for raw_line in path.read_text().splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue

        parts = re.split(r"\s+", line)
        if len(parts) < 2:
            continue

        # Constants: `uint8 ACTIVE = 1` and `uint8 ACTIVE=1`.
        constant = re.match(r"^(\w+)\s*=\s*(.+)$", " ".join(parts[1:]))
        if constant is not None:
            constants[constant.group(1)] = constant.group(2).strip()
            continue

        base_type, is_array = normalize_type(parts[0], package)
        fields.append(Field(name=parts[1], base_type=base_type, is_array=is_array))

    return tuple(fields), constants


def load_package(
    package_dir: Path,
) -> tuple[str, dict[str, tuple[Field, ...]], dict[str, dict[str, str]]]:
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
    parsed = {
        f"{name}/{msg_file.stem}": parse_msg(msg_file, name)
        for msg_file in sorted(msg_root.rglob("*.msg"))
    }

    index = {type_name: fields for type_name, (fields, _) in parsed.items()}
    constants = {type_name: values for type_name, (_, values) in parsed.items()}

    return name, index, constants


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


def geojson_entry(item: Convertible, label: str) -> dict:
    entry = {
        "path": list(item.path),
        "label": label,
        "geometry": item.geometry,
        "color": item.color,
    }
    entry.update(GEOJSON_OVERRIDES.get(f"{item.schema}.{item.dotted}", {}))
    properties = [name for name in item.siblings if name != item.leaf][:12]
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


def track_element_type(rule: dict, index: dict) -> str | None:
    """The message type of one element of the rule's track array."""
    schema = rule["schema"]
    fields = index.get(f"{schema.split('/')[0]}/{schema.split('/')[-1]}")

    if fields is None:
        return None

    array = next((f for f in fields if f.name == rule["array_path"][0]), None)

    return array.base_type if array is not None and array.is_array else None


def check_track_rule(rule: dict, index: dict) -> list[str]:
    """Warns if the .msg files no longer have the fields the rule names."""
    schema = rule["schema"]
    element_type = track_element_type(rule, index)

    if element_type is None:
        return [f"{schema}: track rule skipped, no array field {rule['array_path'][0]}"]

    names = {f.name for f in index.get(element_type, ())}
    missing = [
        name
        for name in (
            rule["id_field"],
            rule["position_path"][0],
            rule.get("velocity_field"),
            rule.get("covariance_field"),
            rule.get("status_field"),
        )
        if name and name not in names
    ]

    return [f"{schema}: track field {name} missing from {element_type}" for name in missing]


def track_selector(
    rule: dict,
    index: dict,
    match_value: int | None = None,
    status: int | None = None,
) -> dict:
    """Where the id, position, covariance and velocity of a track are.

    `status` overrides which tracker state is kept, for the per-status layers;
    the rule's active status is what everything else draws.
    """
    select = {
        "arrayPath": list(rule["array_path"]),
        "idField": rule["id_field"],
        "positionPath": list(rule["position_path"]),
    }

    for key, rule_key in (
        ("velocityField", "velocity_field"),
        ("covarianceField", "covariance_field"),
        ("statusField", "status_field"),
    ):
        if rule.get(rule_key):
            select[key] = rule[rule_key]

    status_value = rule.get("active_status") if status is None else status
    if rule.get("status_field") and status_value is not None:
        select["statusValue"] = status_value

    # Every scalar of the track element is worth a row in a Map tooltip.
    properties = [
        f.name
        for f in index.get(track_element_type(rule, index), ())
        if f.base_type in ROS_PRIMITIVES and not f.is_array
    ][:12]
    if properties:
        select["propertyFields"] = properties

    if match_value is not None:
        select["matchValue"] = match_value

    return select


def track_op(rule: dict, index: dict) -> dict:
    """The combined GeoJSON layer: every track, drawn in full."""
    return {"kind": "track_geojson", "track": track_selector(rule, index)}


def track_statuses(rule: dict, index: dict, constants: dict) -> dict[str, int]:
    """The tracker's states as `name -> value`, from the message's constants.

    `status` names its constants `STATUS_ACTIVE`, `STATUS_DORMANT` and so on, so
    the prefix is the field's own name: a state added to the .msg turns up here
    without the rule having to list it.
    """
    status_field = rule.get("status_field")
    element_type = track_element_type(rule, index)

    if not status_field or element_type is None:
        return {}

    prefix = f"{status_field.upper()}_"
    statuses = {}

    for name, value in constants.get(element_type, {}).items():
        if name.startswith(prefix) and re.fullmatch(r"[+-]?\d+", value):
            statuses[name[len(prefix):].lower()] = int(value)

    return dict(sorted(statuses.items(), key=lambda item: item[1]))


def track_status_topic_specs(rule: dict, index: dict, constants: dict) -> list[dict]:
    """One GeoJSON layer per tracker state, e.g. /active_tracks/dormant.

    The per-id topics only carry active tracks, and a track spends most of its
    life in the other states; these layers are where those are visible. Each
    draws what the combined layer draws, narrowed to the one status.
    """
    return [
        {
            "inputTopic": topic,
            "outputTopic": f"{topic}/{status}",
            "outputSchemaName": GEOJSON[1],
            "op": {
                "kind": "track_geojson",
                "track": track_selector(rule, index, status=value),
            },
        }
        for topic in rule["topics"]
        for status, value in track_statuses(rule, index, constants).items()
    ]


def track_topic_specs(rule: dict, index: dict) -> list[dict]:
    """One LocationFix converter per pre-declared id, e.g. /active_tracks/01.

    Ids are zero-padded to the width of max_id, so the topic list sorts in id
    order instead of lexically — /active_tracks/10 lands after /active_tracks/09
    rather than between /active_tracks/1 and /active_tracks/2.
    """
    width = len(str(rule["max_id"]))

    return [
        {
            "inputTopic": topic,
            "outputTopic": f"{topic}/{element_id:0{width}d}",
            "outputSchemaName": rule["output_schema"],
            "op": {
                "kind": "location_fix_select",
                "select": track_selector(rule, index, element_id),
            },
        }
        for topic in rule["topics"]
        for element_id in range(rule["max_id"] + 1)
    ]


def build_op(item: Convertible, label: str, index: dict, to_schema: str) -> dict:
    kind = item.options[0][0]

    if kind == "geojson":
        return {"kind": "geojson", "entries": [geojson_entry(item, label)]}
    if kind == "image_annotations":
        return {"kind": "image_annotations", "entries": [annotation_entry(item)]}
    if kind == "log":
        return {"kind": "log", "entries": [{"path": list(item.path), "label": label}]}

    op = {"kind": kind, "path": list(item.path)}

    if kind == "image":
        # The slot this converter fills decides which payload it emits on.
        op["payload"] = "compressed" if to_schema == IMAGE_COMPRESSED[1] else "raw"

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


def claimed_by_rule(rule: dict, items: list[Convertible]) -> list[Convertible]:
    """Fields under the rule's prefixes that genuinely need their own topic.

    A field needs one only when it *contends*: two or more fields under the
    prefix resolving to the same output schema cannot all be shown separately by
    one schema converter. A lone field (a bounding box, say) has no rival and is
    left on the schema converter, keeping topic converters to a minimum.
    """
    under = [
        item
        for item in items
        if any(item.dotted == p or item.dotted.startswith(f"{p}.") for p in rule["split_paths"])
    ]

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


def build_specs(
    package: str, index: dict, constants: dict
) -> tuple[list[dict], list[dict], list[str]]:
    schema_specs: list[dict] = []
    topic_specs: list[dict] = []
    notes: list[str] = []

    rules_by_schema: dict[str, list[dict]] = {}
    for rule in TOPIC_RULES:
        rules_by_schema.setdefault(rule["schema"], []).append(rule)

    track_rules_by_schema: dict[str, list[dict]] = {}
    for rule in TRACK_RULES:
        notes.extend(check_track_rule(rule, index))
        topic_specs.extend(track_topic_specs(rule, index))
        topic_specs.extend(track_status_topic_specs(rule, index, constants))

        if rule.get("status_field") and not track_statuses(rule, index, constants):
            notes.append(
                f"{rule['schema']}: no {rule['status_field'].upper()}_* constants,"
                " so no per-status layers"
            )
        track_rules_by_schema.setdefault(rule["schema"], []).append(rule)

    for type_name in sorted(index):
        schema = f"{package}/msg/{type_name.split('/')[-1]}"
        found, skipped = walk(schema, index)

        # Fields under a track array are drawn by its rule, not by the generic
        # walk, so the walk having no target for them is not worth reporting.
        track_prefixes = [
            ".".join(rule["array_path"]) for rule in track_rules_by_schema.get(schema, [])
        ]
        notes.extend(
            f"{schema}: skipped {note}"
            for note in skipped
            if not any(note.startswith(f"{prefix}.") for prefix in track_prefixes)
        )

        # Fields a topic converter owns are removed from the schema converter, so
        # nothing is rendered twice.
        split: list[Convertible] = []
        for rule in rules_by_schema.get(schema, []):
            claimed = claimed_by_rule(rule, found)
            # A default path gets its topic converter *and* stays a candidate for
            # the schema converter, so the slot the group contended over is filled
            # by one of them rather than left empty.
            defaults = set(rule.get("default_paths", ()))
            split.extend(item for item in claimed if item.dotted not in defaults)

            for item in claimed:
                key = derive_key(item, claimed, rule["keys"])
                op = build_op(item, humanize(key), index, item.options[0][1])
                for topic in rule["topics"]:
                    topic_specs.append(
                        {
                            "inputTopic": topic,
                            "outputTopic": f"{topic}/{key}",
                            "outputSchemaName": item.options[0][1],
                            "op": op,
                        }
                    )

        # A track rule draws its whole array itself — position, covariance
        # ellipse and heading arrow together — so the fields underneath it are
        # taken off the generic walk and the GeoJSON slot is spoken for.
        reserved: dict[str, str] = {}
        for rule in track_rules_by_schema.get(schema, []):
            prefix = ".".join(rule["array_path"])
            split.extend(
                item
                for item in found
                if item.dotted == prefix or item.dotted.startswith(f"{prefix}.")
            )
            reserved[GEOJSON[1]] = prefix
            schema_specs.append(
                {
                    "fromSchemaName": schema,
                    "toSchemaName": GEOJSON[1],
                    "op": track_op(rule, index),
                }
            )

        split_paths = {item.dotted for item in split}
        candidates = [item for item in found if item.dotted not in split_paths]

        exclusive: dict[str, Convertible] = {}
        aggregated: dict[str, list[Convertible]] = {}

        for item in candidates:
            for kind, to_schema, is_aggregate in item.options:
                if to_schema in reserved:
                    continue
                if is_aggregate:
                    aggregated.setdefault(to_schema, []).append(item)
                    break
                if to_schema not in exclusive:
                    exclusive[to_schema] = item
                    schema_specs.append(
                        {
                            "fromSchemaName": schema,
                            "toSchemaName": to_schema,
                            "op": build_op(item, humanize(item.leaf), index, to_schema),
                        }
                    )
                    break
            else:
                held = ", ".join(
                    f"{s} held by {exclusive[s].dotted if s in exclusive else reserved[s]}"
                    for _, s, _ in item.options
                    if s in exclusive or s in reserved
                )
                notes.append(f"{schema}: no free slot for {item.dotted} ({held})")

        # Second pass: an image field also takes the other image slot when
        # nothing else claimed it, so a topic works whether the payload turns
        # out to be raw or compressed. Two image fields in one message keep one
        # slot each, since the first pass already assigned their own schemas.
        for item in candidates:
            if item.base_type not in IMAGE_TYPES:
                continue

            for _, to_schema, _ in item.options:
                if to_schema in exclusive or to_schema in reserved:
                    continue

                exclusive[to_schema] = item
                schema_specs.append(
                    {
                        "fromSchemaName": schema,
                        "toSchemaName": to_schema,
                        "op": build_op(item, humanize(item.leaf), index, to_schema),
                    }
                )

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

    schema_specs.sort(key=lambda s: (s["fromSchemaName"], s["toSchemaName"]))
    topic_specs.sort(key=lambda s: (s["inputTopic"], s["outputTopic"]))

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
    package, index, constants = load_package(package_dir)
    schema_specs, topic_specs, notes = build_specs(package, index, constants)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(render(schema_specs, topic_specs))

    print(f"{package_dir}: {len(index)} messages -> {package}")
    print(f"{len(schema_specs)} schema converters, {len(topic_specs)} topic converters")

    for note in notes:
        print(f"  note: {note}")

    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
