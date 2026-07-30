#!/usr/bin/env python3
"""Shared ROS 2 message model for the CDCL Foxglove converter generators.

Parses `.msg` files, indexes them, walks nested fields, and classifies each
reachable field into the Foxglove conversion(s) it supports. Both generators and
the audit script import from here so they always agree on what is convertible.
"""
from __future__ import annotations

import os
import re
from dataclasses import dataclass, field as dataclass_field
from pathlib import Path

# Depth 0 is a field declared directly on the message. Depth 3 covers
# TargetBoxArray -> uav_target_boxes -> target_bbox -> center without pulling in
# unbounded amounts of unrelated nesting.
DEFAULT_MAX_DEPTH = 4

# Message roots are searched in this order when none is given explicitly.
# The first entry is the dev-container mount; the rest are host checkouts.
DEFAULT_MSG_ROOTS = (
    "/ros_ws/src/cdcl_umd_msgs/msg",
    "~/ros2_ws/src/cdcl_umd_msgs/msg",
)

ROS_PRIMITIVES = {
    "bool",
    "byte",
    "char",
    "float32",
    "float64",
    "int8",
    "uint8",
    "int16",
    "uint16",
    "int32",
    "uint32",
    "int64",
    "uint64",
    "string",
    "wstring",
}

POLYGON_FIELD_HINTS = ("polygon", "fence", "domain", "zone", "boundary", "bounds", "perimeter")
TEXT_FIELD_HINTS = ("transcript", "caption", "text", "description", "summary", "message")
AUDIO_BYTE_FIELD_NAMES = {"raw_audio", "audio", "audio_data", "pcm", "samples"}
AUDIO_STAMP_FIELD_NAMES = ("audio_start", "audio_start_time", "start_time", "stamp")

# Sibling fields preferred when labelling a bounding box, in priority order.
# Deliberately excludes ids: they are long, uninformative on a video frame, and
# are already carried in the GeoJSON feature properties for tooltips.
LABEL_FIELD_HINTS = ("class", "label", "name", "confidence", "score")

MAX_LABEL_FIELDS = 3
MAX_PROPERTY_FIELDS = 12

# Deterministic palette; a field keeps its colour across regenerations.
FIELD_COLORS = (
    "#e6194b",
    "#3cb44b",
    "#4363d8",
    "#f58231",
    "#911eb4",
    "#46f0f0",
    "#f032e6",
    "#bcf60c",
    "#fabebe",
    "#008080",
    "#e6beff",
    "#9a6324",
    "#808000",
    "#800000",
    "#aaffc3",
    "#ffd8b1",
    "#000075",
    "#a9a9a9",
)


@dataclass(frozen=True)
class TargetOption:
    """One way a field can be exposed to Foxglove.

    `op_kind` names a variant of `ConverterOp` in shared/converterRuntime.ts.
    `aggregate` marks targets where several fields of the same message merge into
    a single converter (GeoJSON feature collections, image annotations, logs);
    non-aggregate targets occupy an exclusive slot, because Foxglove allows only
    one converter per (fromSchemaName, toSchemaName) pair.
    """

    op_kind: str
    schema: str
    aggregate: bool = False


@dataclass(frozen=True)
class Field:
    type_str: str
    name: str
    base_type: str
    is_array: bool


@dataclass(frozen=True)
class MessageDef:
    name: str
    package: str
    path: Path
    fields: tuple[Field, ...]

    @property
    def full_name(self) -> str:
        return f"{self.package}/msg/{self.name}"


@dataclass(frozen=True)
class ConvertibleField:
    """A field reachable from a root message that Foxglove can display."""

    root_schema: str
    path: tuple[str, ...]
    base_type: str
    #: True when the path crosses a repeated field, so it may resolve to many values.
    through_array: bool
    options: tuple[TargetOption, ...]
    #: Primitive sibling fields of the container, usable as properties/labels.
    sibling_fields: tuple[str, ...]
    container_type: str | None

    @property
    def dotted(self) -> str:
        return ".".join(self.path)

    @property
    def leaf(self) -> str:
        return self.path[-1]

    @property
    def color(self) -> str:
        return stable_color(f"{self.root_schema}.{self.dotted}")

    @property
    def label(self) -> str:
        return humanize(self.leaf)

    @property
    def geometry(self) -> str:
        """GeoJSON geometry for a location field: a closed ring, or one point per fix.

        Only multi-valued fields can be polygons. The hint is matched against the
        message name as well as the field path, so `Geofence.coordinates` is
        recognised even though the field name alone gives nothing away.
        """
        if not self.through_array:
            return "point"

        haystack = f"{self.root_schema.split('/')[-1]}.{self.dotted}".lower()

        return "polygon" if any(hint in haystack for hint in POLYGON_FIELD_HINTS) else "point"


def stable_color(value: str) -> str:
    total = 0
    for char in value:
        total = (total * 31 + ord(char)) % 1000003
    return FIELD_COLORS[total % len(FIELD_COLORS)]


def humanize(name: str) -> str:
    return name.replace("_", " ").strip().capitalize() or name


def normalize_type(type_str: str, current_package: str) -> tuple[str, bool]:
    """Returns (base_type, is_array) with the base type as `pkg/Type` or a primitive."""
    base = type_str.strip()
    is_array = False

    # Handles `Type[]`, `Type[5]`, and bounded `Type[<=5]`.
    match = re.match(r"^(.*?)\[[^\]]*\]$", base)
    if match:
        base = match.group(1)
        is_array = True

    # Strip a string bound such as `string<=20`.
    base = base.split("<=")[0].strip()

    if base in ROS_PRIMITIVES:
        return base, is_array

    parts = base.split("/")

    if len(parts) == 3 and parts[1] == "msg":
        return f"{parts[0]}/{parts[2]}", is_array

    if len(parts) == 2:
        return base, is_array

    if len(parts) == 1 and base:
        # A bare `TargetBox` refers to a message in the same package.
        return f"{current_package}/{base}", is_array

    return base, is_array


def parse_msg(path: Path, package: str) -> MessageDef:
    fields: list[Field] = []

    for raw_line in path.read_text().splitlines():
        line = raw_line.split("#", 1)[0].strip()

        if not line:
            continue

        parts = re.split(r"\s+", line)

        if len(parts) < 2:
            continue

        type_str, name = parts[0], parts[1]

        # Skip constants (`uint8 ACTIVE = 1`, `uint8 ACTIVE=1`).
        if "=" in name or (len(parts) > 2 and parts[2].startswith("=")):
            continue

        base_type, is_array = normalize_type(type_str, package)
        fields.append(Field(type_str=type_str, name=name, base_type=base_type, is_array=is_array))

    return MessageDef(name=path.stem, package=package, path=path, fields=tuple(fields))


def infer_package(msg_root: Path) -> str:
    """Derives the ROS package name from a `.../<package>/msg[/...]` layout."""
    resolved = msg_root.resolve()

    for candidate in (resolved, *resolved.parents):
        if (candidate / "package.xml").exists():
            try:
                text = (candidate / "package.xml").read_text()
                match = re.search(r"<name>\s*([^<\s]+)\s*</name>", text)
                if match:
                    return match.group(1)
            except OSError:
                pass
            return candidate.name

    if resolved.name == "msg" and resolved.parent.name:
        return resolved.parent.name

    return resolved.name


class MessageIndex:
    """Recursively indexes `.msg` files, keyed by both local and qualified name."""

    def __init__(self) -> None:
        self._by_full_name: dict[str, MessageDef] = {}
        self._by_local_name: dict[str, list[MessageDef]] = {}

    def add_root(self, msg_root: Path, package: str | None = None) -> int:
        package = package or infer_package(msg_root)
        count = 0

        # rglob, not glob: subdirectories such as msg/radar/ hold real messages.
        for msg_file in sorted(msg_root.rglob("*.msg")):
            definition = parse_msg(msg_file, package)
            self._by_full_name[definition.full_name] = definition
            self._by_local_name.setdefault(definition.name, []).append(definition)
            count += 1

        return count

    def get(self, type_ref: str) -> MessageDef | None:
        parts = type_ref.split("/")

        if len(parts) == 3 and parts[1] == "msg":
            type_ref = f"{parts[0]}/{parts[2]}"
            parts = type_ref.split("/")

        if len(parts) == 2:
            direct = self._by_full_name.get(f"{parts[0]}/msg/{parts[1]}")
            if direct is not None:
                return direct
            # Fall back to the local name: a message may reference a type by a
            # package alias that does not match the directory it lives in.
            candidates = self._by_local_name.get(parts[1], [])
            return candidates[0] if len(candidates) == 1 else None

        candidates = self._by_local_name.get(type_ref, [])
        return candidates[0] if len(candidates) == 1 else None

    def messages(self) -> list[MessageDef]:
        return [self._by_full_name[key] for key in sorted(self._by_full_name)]

    def __len__(self) -> int:
        return len(self._by_full_name)


# ---------------------------------------------------------------------------
# Type -> Foxglove target registry
# ---------------------------------------------------------------------------

GEOJSON = TargetOption("geojson", "foxglove_msgs/msg/GeoJSON", aggregate=True)
IMAGE_ANNOTATIONS = TargetOption(
    "image_annotations", "foxglove_msgs/msg/ImageAnnotations", aggregate=True
)
LOG = TargetOption("log", "foxglove_msgs/msg/Log", aggregate=True)
AUDIO = TargetOption("audio", "foxglove_msgs/msg/RawAudio")


def _passthrough(schema: str) -> TargetOption:
    return TargetOption("passthrough", schema)


#: Scalar (or single-valued) targets keyed by normalized `pkg/Type`.
SCALAR_TARGETS: dict[str, tuple[TargetOption, ...]] = {
    "sensor_msgs/Image": (TargetOption("image", "sensor_msgs/msg/Image"),),
    "sensor_msgs/CompressedImage": (TargetOption("image", "sensor_msgs/msg/CompressedImage"),),
    # A NavSatFix prefers a native pass-through (the Map panel renders it
    # directly and lets the user pick a colour); GeoJSON is the fallback once
    # that exclusive slot is taken by another field of the same message.
    "sensor_msgs/NavSatFix": (
        TargetOption("navsatfix", "sensor_msgs/msg/NavSatFix"),
        GEOJSON,
    ),
    "gps_msgs/GPSFix": (
        TargetOption("navsatfix", "gps_msgs/msg/GPSFix"),
        GEOJSON,
    ),
    "vision_msgs/BoundingBox2D": (IMAGE_ANNOTATIONS,),
    "sensor_msgs/Imu": (_passthrough("sensor_msgs/msg/Imu"),),
    "sensor_msgs/PointCloud2": (_passthrough("sensor_msgs/msg/PointCloud2"),),
    "sensor_msgs/LaserScan": (_passthrough("sensor_msgs/msg/LaserScan"),),
    "nav_msgs/Odometry": (_passthrough("nav_msgs/msg/Odometry"),),
    "nav_msgs/Path": (_passthrough("nav_msgs/msg/Path"),),
    "geometry_msgs/Pose": (_passthrough("geometry_msgs/msg/Pose"),),
    "geometry_msgs/PoseStamped": (_passthrough("geometry_msgs/msg/PoseStamped"),),
    "geometry_msgs/PoseWithCovariance": (_passthrough("geometry_msgs/msg/PoseWithCovariance"),),
    "geometry_msgs/PoseWithCovarianceStamped": (
        _passthrough("geometry_msgs/msg/PoseWithCovarianceStamped"),
    ),
    "geometry_msgs/Point": (_passthrough("geometry_msgs/msg/Point"),),
    "geometry_msgs/PointStamped": (_passthrough("geometry_msgs/msg/PointStamped"),),
    "geometry_msgs/Quaternion": (_passthrough("geometry_msgs/msg/Quaternion"),),
    "geometry_msgs/Vector3": (_passthrough("geometry_msgs/msg/Vector3"),),
    "geometry_msgs/Vector3Stamped": (_passthrough("geometry_msgs/msg/Vector3Stamped"),),
    "geometry_msgs/Twist": (_passthrough("geometry_msgs/msg/Twist"),),
    "geometry_msgs/TwistStamped": (_passthrough("geometry_msgs/msg/TwistStamped"),),
    "geometry_msgs/Transform": (_passthrough("geometry_msgs/msg/Transform"),),
    "geometry_msgs/TransformStamped": (_passthrough("geometry_msgs/msg/TransformStamped"),),
    "visualization_msgs/Marker": (_passthrough("visualization_msgs/msg/Marker"),),
    "visualization_msgs/MarkerArray": (_passthrough("visualization_msgs/msg/MarkerArray"),),
}

#: Targets used when a field resolves to many values (declared array, or reached
#: through one). Types absent here have no meaningful multi-value rendering and
#: are reported as unconvertible rather than silently truncated to the first item.
ARRAY_TARGETS: dict[str, tuple[TargetOption, ...]] = {
    "sensor_msgs/NavSatFix": (GEOJSON,),
    "gps_msgs/GPSFix": (GEOJSON,),
    "vision_msgs/BoundingBox2D": (IMAGE_ANNOTATIONS,),
    "geometry_msgs/Pose": (TargetOption("pose_array", "geometry_msgs/msg/PoseArray"),),
    "geometry_msgs/PoseStamped": (TargetOption("pose_array", "geometry_msgs/msg/PoseArray"),),
}


def target_options(base_type: str, through_array: bool) -> tuple[TargetOption, ...]:
    if through_array:
        return ARRAY_TARGETS.get(base_type, ())
    return SCALAR_TARGETS.get(base_type, ())


def primitive_sibling_fields(definition: MessageDef | None) -> tuple[str, ...]:
    if definition is None:
        return ()

    return tuple(
        f.name for f in definition.fields if f.base_type in ROS_PRIMITIVES and not f.is_array
    )


def pick_label_fields(sibling_fields: tuple[str, ...]) -> tuple[str, ...]:
    scored: list[tuple[int, str]] = []

    for name in sibling_fields:
        lowered = name.lower()
        for rank, hint in enumerate(LABEL_FIELD_HINTS):
            if hint in lowered:
                scored.append((rank, name))
                break

    scored.sort(key=lambda item: item[0])
    return tuple(name for _, name in scored[:MAX_LABEL_FIELDS])


@dataclass
class WalkResult:
    convertible: list[ConvertibleField] = dataclass_field(default_factory=list)
    #: (dotted path, base type, reason) for fields that look displayable but are not.
    skipped: list[tuple[str, str, str]] = dataclass_field(default_factory=list)


def walk_message(
    definition: MessageDef,
    index: MessageIndex,
    max_depth: int = DEFAULT_MAX_DEPTH,
) -> WalkResult:
    """Depth-first walk yielding every convertible field reachable from `definition`."""
    result = WalkResult()
    root_schema = definition.full_name

    def visit(
        current: MessageDef,
        prefix: tuple[str, ...],
        depth: int,
        through_array: bool,
        type_stack: tuple[str, ...],
    ) -> None:
        sibling_fields = primitive_sibling_fields(current)

        for f in current.fields:
            path = prefix + (f.name,)
            field_through_array = through_array or f.is_array

            options = target_options(f.base_type, field_through_array)

            if options:
                result.convertible.append(
                    ConvertibleField(
                        root_schema=root_schema,
                        path=path,
                        base_type=f.base_type,
                        through_array=field_through_array,
                        options=options,
                        sibling_fields=sibling_fields,
                        container_type=current.full_name,
                    )
                )
                continue

            # A known displayable type that has no multi-value representation.
            if field_through_array and f.base_type in SCALAR_TARGETS:
                result.skipped.append(
                    (
                        ".".join(path),
                        f.base_type,
                        "reached through an array and has no multi-value Foxglove target",
                    )
                )
                continue

            if f.base_type == "uint8" and f.is_array and f.name.lower() in AUDIO_BYTE_FIELD_NAMES:
                result.convertible.append(
                    ConvertibleField(
                        root_schema=root_schema,
                        path=path,
                        base_type="uint8[]",
                        through_array=False,
                        options=(AUDIO,),
                        sibling_fields=sibling_fields,
                        container_type=current.full_name,
                    )
                )
                continue

            if (
                f.base_type == "string"
                and not f.is_array
                and not through_array
                and any(hint in f.name.lower() for hint in TEXT_FIELD_HINTS)
            ):
                result.convertible.append(
                    ConvertibleField(
                        root_schema=root_schema,
                        path=path,
                        base_type="string",
                        through_array=False,
                        options=(LOG,),
                        sibling_fields=sibling_fields,
                        container_type=current.full_name,
                    )
                )
                continue

            if f.base_type in ROS_PRIMITIVES:
                continue

            nested = index.get(f.base_type)

            if nested is None or depth >= max_depth:
                continue

            # Guard against self-referential or mutually recursive definitions.
            if nested.full_name in type_stack:
                continue

            visit(
                nested,
                path,
                depth + 1,
                field_through_array,
                type_stack + (nested.full_name,),
            )

    visit(definition, (), 0, False, (definition.full_name,))
    return result


def resolve_msg_roots(explicit: list[str] | None) -> list[Path]:
    """Resolves message roots from CLI args, then $CDCL_MSG_ROOTS, then defaults."""
    candidates: list[str] = []

    if explicit:
        candidates = list(explicit)
    elif os.environ.get("CDCL_MSG_ROOTS"):
        candidates = [part for part in os.environ["CDCL_MSG_ROOTS"].split(os.pathsep) if part]
    else:
        candidates = list(DEFAULT_MSG_ROOTS)

    resolved = [Path(candidate).expanduser() for candidate in candidates]
    existing = [path for path in resolved if path.is_dir()]

    if not existing:
        searched = "\n  ".join(str(path) for path in resolved)
        raise SystemExit(f"No ROS message directory found. Searched:\n  {searched}")

    if explicit:
        missing = [path for path in resolved if not path.is_dir()]
        if missing:
            raise SystemExit(f"Message directory not found: {missing[0]}")
        return resolved

    # Auto-discovery: the first existing default wins.
    return existing[:1]


def build_index(msg_roots: list[Path]) -> MessageIndex:
    index = MessageIndex()

    for root in msg_roots:
        index.add_root(root)

    return index
