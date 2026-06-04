#!/usr/bin/env python3
from pathlib import Path
import re
import os
import sys
from collections import defaultdict

MSG_ROOT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/ros_ws/src/cdcl_umd_msgs/msg")
OUT_FILE = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("cdcl-schema-converters/src/generatedSchemaConverters.ts")
PKG_NAME = sys.argv[3] if len(sys.argv) > 3 else "cdcl_umd_msgs"

TEXT_NAME_HINTS = ("transcript", "caption", "text", "description", "label", "status")
AUDIO_BYTE_FIELD_NAMES = {"raw_audio", "audio", "audio_data"}
POLYGON_LOCATION_FIELD_HINTS = ("polygon", "fence", "domain", "zone", "boundary", "bounds", "perimeter")
MAX_NESTING_DEPTH = 3

# Optional workaround for Foxglove Map layer color overriding GeoJSON feature colors.
# When set, only GeoJSON fields whose full field path matches this regex are generated.
# Examples:
#   CDCL_GEOJSON_FIELD_REGEX=target_location_altimeter_plane
#   CDCL_GEOJSON_FIELD_REGEX=target_location_gimbal_plane
#   CDCL_GEOJSON_FIELD_REGEX=target_location_rangefinder
GEOJSON_FIELD_REGEX = os.environ.get("CDCL_GEOJSON_FIELD_REGEX", "target_location_altimeter_plane")

FIELD_COLORS = [
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
    "#fffac8",
    "#800000",
    "#aaffc3",
    "#808000",
    "#ffd8b1",
    "#000075",
]

SCALAR_CONVERTIBLE_TYPES = {
    "sensor_msgs/Image": "image",
    "sensor_msgs/CompressedImage": "image",
    "sensor_msgs/NavSatFix": "location",
    "gps_msgs/GPSFix": "location",
    "nav_msgs/Odometry": "odometry",
    "geometry_msgs/Pose": "pose",
    "geometry_msgs/PoseStamped": "pose",
    "geometry_msgs/PoseWithCovariance": "pose",
    "geometry_msgs/PoseWithCovarianceStamped": "pose",
    "geometry_msgs/Point": "point",
    "geometry_msgs/PointStamped": "point",
    "geometry_msgs/Quaternion": "orientation",
    "geometry_msgs/Vector3": "vector",
    "geometry_msgs/Vector3Stamped": "vector",
}

ARRAY_TARGET_OVERRIDES = {
    "sensor_msgs/NavSatFix": "location_array",
    "gps_msgs/GPSFix": "location_array",
    "geometry_msgs/Point": "point_array",
    "geometry_msgs/Pose": "pose_array",
    "geometry_msgs/PoseStamped": "pose_array",
    "geometry_msgs/Vector3": "vector_array",
}

PASS_THROUGH_TO_SCHEMA = {
    "sensor_msgs/Image": "sensor_msgs/msg/Image",
    "sensor_msgs/CompressedImage": "sensor_msgs/msg/CompressedImage",
    "nav_msgs/Odometry": "nav_msgs/msg/Odometry",
    "geometry_msgs/Pose": "geometry_msgs/msg/Pose",
    "geometry_msgs/PoseStamped": "geometry_msgs/msg/PoseStamped",
    "geometry_msgs/PoseWithCovariance": "geometry_msgs/msg/PoseWithCovariance",
    "geometry_msgs/PoseWithCovarianceStamped": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "geometry_msgs/Point": "geometry_msgs/msg/Point",
    "geometry_msgs/PointStamped": "geometry_msgs/msg/PointStamped",
    "geometry_msgs/Quaternion": "geometry_msgs/msg/Quaternion",
    "geometry_msgs/Vector3": "geometry_msgs/msg/Vector3",
    "geometry_msgs/Vector3Stamped": "geometry_msgs/msg/Vector3Stamped",
}


def parse_field_type(field_type: str) -> tuple[str, bool]:
    if field_type.endswith("[]"):
        return field_type[:-2], True

    match = re.match(r"^(.+)\[[^\]]*\]$", field_type)
    if match:
        return match.group(1), True

    return field_type, False


def local_type_name(type_name: str) -> str:
    return type_name.split("/")[-1]


def sanitize_identifier(value: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_]+", "_", value)
    value = re.sub(r"_+", "_", value).strip("_")
    return value or "unnamed"


def converter_name(msg_name: str, field_path: str, target: str) -> str:
    field_part = sanitize_identifier(field_path.replace(".", "__"))
    return f"{sanitize_identifier(msg_name)}__{field_part}__to__{sanitize_identifier(target)}"


def stable_color(value: str) -> str:
    total = 0
    for char in value:
        total = (total * 31 + ord(char)) % 1000003
    return FIELD_COLORS[total % len(FIELD_COLORS)]


def parse_msg(path: Path) -> list[tuple[str, str]]:
    fields = []

    for raw_line in path.read_text().splitlines():
        line = raw_line.split("#", 1)[0].strip()

        if not line or "=" in line:
            continue

        parts = re.split(r"\s+", line)

        if len(parts) < 2:
            continue

        fields.append((parts[0], parts[1]))

    return fields


def load_msg_index(msg_root: Path) -> dict[str, list[tuple[str, str]]]:
    return {
        msg_file.stem: parse_msg(msg_file)
        for msg_file in sorted(msg_root.glob("*.msg"))
    }


def is_polygon_like_location_array(field_path: str) -> bool:
    path = field_path.lower()
    return any(hint in path for hint in POLYGON_LOCATION_FIELD_HINTS)


def classify_target(base_type: str, field_path: str, effective_is_array: bool) -> str | None:
    if base_type in {"sensor_msgs/NavSatFix", "gps_msgs/GPSFix"}:
        if effective_is_array:
            return "polygon" if is_polygon_like_location_array(field_path) else "location_array"
        return "location"

    if effective_is_array and base_type in ARRAY_TARGET_OVERRIDES:
        return ARRAY_TARGET_OVERRIDES[base_type]

    if not effective_is_array and base_type in SCALAR_CONVERTIBLE_TYPES:
        return SCALAR_CONVERTIBLE_TYPES[base_type]

    return None


def to_schema_name(target: str, source_type: str, base_type: str) -> str | None:
    if target in {"location", "location_array", "polygon"}:
        return "foxglove_msgs/msg/GeoJSON"

    if target == "audio":
        return "foxglove_msgs/msg/RawAudio"

    if target == "text":
        return "foxglove_msgs/msg/Log"

    return PASS_THROUGH_TO_SCHEMA.get(base_type)


def make_result(
    msg_name: str,
    field_path: str,
    target: str,
    source_type: str,
    base_type: str,
    cardinality: str,
) -> dict[str, str] | None:
    schema_name = to_schema_name(target, source_type, base_type)
    if schema_name is None:
        return None

    return {
        "msg_name": msg_name,
        "from_schema": f"{PKG_NAME}/msg/{msg_name}",
        "field": field_path,
        "target": target,
        "source_type": source_type,
        "base_type": base_type,
        "cardinality": cardinality,
        "converter": converter_name(msg_name, field_path, target),
        "to_schema": schema_name,
        "color": stable_color(f"{msg_name}.{field_path}.{target}"),
    }


def infer_field(
    root_msg_name: str,
    field_type: str,
    field_path: str,
    msg_index: dict[str, list[tuple[str, str]]],
    depth: int,
    parent_is_array: bool = False,
) -> list[dict[str, str]]:
    base_type, is_array = parse_field_type(field_type)
    effective_is_array = parent_is_array or is_array
    cardinality = "array" if effective_is_array else "scalar"

    target = classify_target(base_type, field_path, effective_is_array)
    if target is not None:
        result = make_result(root_msg_name, field_path, target, field_type, base_type, cardinality)
        return [result] if result is not None else []

    if field_type == "uint8[]" and field_path.split(".")[-1] in AUDIO_BYTE_FIELD_NAMES:
        result = make_result(root_msg_name, field_path, "audio", field_type, "uint8", "array")
        return [result] if result is not None else []

    if field_type == "string" and any(hint in field_path.lower() for hint in TEXT_NAME_HINTS):
        result = make_result(root_msg_name, field_path, "text", field_type, "string", cardinality)
        return [result] if result is not None else []

    nested_msg_name = local_type_name(base_type)

    if depth >= MAX_NESTING_DEPTH or nested_msg_name not in msg_index:
        return []

    results = []
    for nested_type, nested_name in msg_index[nested_msg_name]:
        nested_path = f"{field_path}.{nested_name}"
        results.extend(
            infer_field(
                root_msg_name=root_msg_name,
                field_type=nested_type,
                field_path=nested_path,
                msg_index=msg_index,
                depth=depth + 1,
                parent_is_array=effective_is_array,
            )
        )

    return results


def infer_convertibles(
    msg_name: str,
    fields: list[tuple[str, str]],
    msg_index: dict[str, list[tuple[str, str]]],
) -> list[dict[str, str]]:
    results = []

    for field_type, field_name in fields:
        results.extend(
            infer_field(
                root_msg_name=msg_name,
                field_type=field_type,
                field_path=field_name,
                msg_index=msg_index,
                depth=0,
            )
        )

    return results


def ts_string(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def ts_array(values: list[str]) -> str:
    return "[" + ", ".join(ts_string(value) for value in values) + "]"


def generate_geojson_registration(from_schema: str, items: list[dict[str, str]]) -> str:
    fields = []
    for item in items:
        fields.append(
            "{ "
            f"path: {ts_array(item['field'].split('.'))}, "
            f"mode: {ts_string(item['target'])}, "
            f"field: {ts_string(item['field'])}, "
            f"color: {ts_string(item['color'])}"
            " }"
        )

    field_configs = "[\n      " + ",\n      ".join(fields) + "\n    ]"

    return f"""  // Aggregated GeoJSON converter for {from_schema}
  extensionContext.registerMessageConverter({{
    type: "schema",
    fromSchemaName: {ts_string(from_schema)},
    toSchemaName: "foxglove_msgs/msg/GeoJSON",
    converter: (message: Immutable<unknown>, event: Immutable<MessageEvent<unknown>>) =>
      convertGeoJsonFields(message, event, {field_configs}),
  }});"""


def generate_field_registration(item: dict[str, str]) -> str:
    field_parts_ts = ts_array(item["field"].split("."))

    if item["target"] == "audio":
        converter_expr = f"convertRawAudio(message, event, {field_parts_ts})"
    elif item["target"] == "text":
        converter_expr = f"convertTextLog(message, event, {field_parts_ts}, {ts_string(item['field'])})"
    else:
        converter_expr = f"convertPassThrough(message, event, {field_parts_ts}, {ts_string(item['target'])})"

    return f"""  // {item['converter']}
  extensionContext.registerMessageConverter({{
    type: "schema",
    fromSchemaName: {ts_string(item['from_schema'])},
    toSchemaName: {ts_string(item['to_schema'])},
    converter: (message: Immutable<unknown>, event: Immutable<MessageEvent<unknown>>) =>
      {converter_expr},
  }});"""


def generate_ts(converters: list[dict[str, str]]) -> str:
    geojson_by_schema: dict[str, list[dict[str, str]]] = defaultdict(list)
    non_geojson = []

    for item in converters:
        if item["to_schema"] == "foxglove_msgs/msg/GeoJSON":
            if GEOJSON_FIELD_REGEX is not None and re.search(GEOJSON_FIELD_REGEX, item["field"]) is None:
                continue
            geojson_by_schema[item["from_schema"]].append(item)
        else:
            non_geojson.append(item)

    registrations = []

    for from_schema in sorted(geojson_by_schema):
        registrations.append(generate_geojson_registration(from_schema, geojson_by_schema[from_schema]))

    for item in non_geojson:
        registrations.append(generate_field_registration(item))

    return f"""// This file is generated by scripts/generate_schema_converters.py.
// Do not edit by hand.

import {{ ExtensionContext, Immutable, MessageEvent }} from "@foxglove/extension";

type AnyMessage = Record<string, unknown>;
type FoxgloveTime = {{ sec: number; nsec: number }};
type RosTime = {{ sec: number; nanosec: number }};
type GeoJsonFieldConfig = {{
  path: readonly string[];
  mode: "location" | "location_array" | "polygon";
  field: string;
  color: string;
}};

const AUDIO_FORMAT = "pcm-s16";
const AUDIO_SAMPLE_RATE = 48000;
const AUDIO_CHANNELS = 1;

function asObject(value: unknown): AnyMessage | undefined {{
  if (typeof value !== "object" || value == undefined || Array.isArray(value)) {{
    return undefined;
  }}
  return value as AnyMessage;
}}

function eventTime(event: Immutable<MessageEvent<unknown>>): FoxgloveTime {{
  const stamp = event.publishTime ?? event.receiveTime;
  return {{
    sec: Number(stamp?.sec ?? 0),
    nsec: Number(stamp?.nsec ?? 0),
  }};
}}

function eventRosTime(event: Immutable<MessageEvent<unknown>>): RosTime {{
  const stamp = eventTime(event);
  return {{
    sec: stamp.sec,
    nanosec: stamp.nsec,
  }};
}}

function normalizeBytes(value: unknown): Uint8Array {{
  if (value instanceof Uint8Array) {{
    return value;
  }}

  if (Array.isArray(value)) {{
    return new Uint8Array(value as readonly number[]);
  }}

  return new Uint8Array();
}}

function getValuesAtPath(value: unknown, path: readonly string[]): unknown[] {{
  if (value == undefined) {{
    return [];
  }}

  if (path.length === 0) {{
    return Array.isArray(value) ? value : [value];
  }}

  const [head, ...tail] = path;

  if (head == undefined) {{
    return [];
  }}

  if (Array.isArray(value)) {{
    return value.flatMap((entry) => getValuesAtPath(entry, path));
  }}

  const objectValue = asObject(value);
  if (objectValue == undefined) {{
    return [];
  }}

  return getValuesAtPath(objectValue[head], tail);
}}

function getFirstAtPath(value: unknown, path: readonly string[]): unknown | undefined {{
  return getValuesAtPath(value, path)[0];
}}

function isFiniteNumber(value: unknown): value is number {{
  return typeof value === "number" && Number.isFinite(value);
}}

function isValidNavSatFix(value: unknown): value is AnyMessage {{
  const fix = asObject(value);
  if (fix == undefined) {{
    return false;
  }}

  return (
    isFiniteNumber(fix.latitude) &&
    isFiniteNumber(fix.longitude) &&
    Math.abs(fix.latitude) <= 90 &&
    Math.abs(fix.longitude) <= 180
  );
}}

function navSatFixToCoordinates(fix: AnyMessage): number[] {{
  const coordinates = [fix.longitude as number, fix.latitude as number];

  if (isFiniteNumber(fix.altitude)) {{
    coordinates.push(fix.altitude);
  }}

  return coordinates;
}}

function geoJsonStyle(color: string): AnyMessage {{
  return {{
    "marker-color": color,
    "marker-size": "large",
    "marker-symbol": "circle",
    "stroke-width": 4,
    "stroke-opacity": 1.0,
    "fill-opacity": 0.35,
    color,
    markerColor: color,
    strokeColor: color,
    fillColor: color,
    fillOpacity: 0.35,
    strokeWidth: 4,
    strokeOpacity: 1.0,
    radius: 8,
    opacity: 1.0,
    stroke: color,
    fill: color,
  }};
}}

function geoJsonProperties(config: GeoJsonFieldConfig, event: Immutable<MessageEvent<unknown>>, index?: number): AnyMessage {{
  return {{
    ...geoJsonStyle(config.color),
    name: index == undefined ? config.field : `${{config.field}} ${{index}}`,
    field: config.field,
    mode: config.mode,
    index,
    source_topic: event.topic,
    color: config.color,
  }};
}}

function emptyFeatureCollection(event: Immutable<MessageEvent<unknown>>): unknown {{
  return {{
    timestamp: eventTime(event),
    frame_id: "",
    geojson: JSON.stringify({{
      type: "FeatureCollection",
      features: [],
    }}),
  }};
}}

function convertGeoJsonFields(
  message: Immutable<unknown>,
  event: Immutable<MessageEvent<unknown>>,
  configs: readonly GeoJsonFieldConfig[],
): unknown {{
  const features: unknown[] = [];
  let frameId = "";

  for (const config of configs) {{
    const fixes = getValuesAtPath(message, config.path).filter(isValidNavSatFix);

    if (fixes.length === 0) {{
      continue;
    }}

    if (frameId.length === 0) {{
      const header = asObject(asObject(fixes[0])?.header);
      frameId = String(header?.frame_id ?? "");
    }}

    if (config.mode === "polygon") {{
      const ring = fixes.map(navSatFixToCoordinates);
      const first = ring[0];
      const last = ring[ring.length - 1];

      if (first != undefined && last != undefined) {{
        if (first[0] !== last[0] || first[1] !== last[1]) {{
          ring.push([...first]);
        }}
      }}

      if (ring.length >= 4) {{
        features.push({{
          type: "Feature",
          geometry: {{
            type: "Polygon",
            coordinates: [ring],
          }},
          properties: geoJsonProperties(config, event),
        }});
      }}

      continue;
    }}

    fixes.forEach((fix, index) => {{
      const objectFix = fix as AnyMessage;
      features.push({{
        type: "Feature",
        geometry: {{
          type: "Point",
          coordinates: navSatFixToCoordinates(objectFix),
        }},
        properties: {{
          ...geoJsonProperties(config, event, index),
          latitude: objectFix.latitude,
          longitude: objectFix.longitude,
          altitude: objectFix.altitude,
        }},
      }});
    }});
  }}

  if (features.length === 0) {{
    return emptyFeatureCollection(event);
  }}

  return {{
    timestamp: eventTime(event),
    frame_id: frameId,
    geojson: JSON.stringify({{
      type: "FeatureCollection",
      features,
    }}),
  }};
}}

function convertRawAudio(
  message: Immutable<unknown>,
  event: Immutable<MessageEvent<unknown>>,
  path: readonly string[],
): unknown {{
  const value = getFirstAtPath(message, path);
  const bytes = normalizeBytes(value);

  if (bytes.length === 0) {{
    return undefined;
  }}

  const root = asObject(message);
  const audioStart = asObject(root?.audio_start);
  const timestamp =
    audioStart != undefined
      ? {{
          sec: Number(audioStart.sec ?? 0),
          nsec: Number(audioStart.nsec ?? audioStart.nanosec ?? 0),
        }}
      : eventTime(event);

  return {{
    timestamp,
    data: bytes,
    format: AUDIO_FORMAT,
    sample_rate: AUDIO_SAMPLE_RATE,
    number_of_channels: AUDIO_CHANNELS,
  }};
}}

function convertTextLog(
  message: Immutable<unknown>,
  event: Immutable<MessageEvent<unknown>>,
  path: readonly string[],
  fieldLabel: string,
): unknown {{
  const value = getFirstAtPath(message, path);

  if (value == undefined || String(value).length === 0) {{
    return undefined;
  }}

  return {{
    timestamp: eventTime(event),
    level: 2,
    message: String(value),
    name: fieldLabel,
    file: "",
    line: 0,
  }};
}}

function convertPassThrough(
  message: Immutable<unknown>,
  event: Immutable<MessageEvent<unknown>>,
  path: readonly string[],
  target: string,
): unknown {{
  const value = getFirstAtPath(message, path);

  if (value == undefined) {{
    return undefined;
  }}

  const objectValue = asObject(value);

  if (objectValue != undefined && target === "image") {{
    return {{
      ...objectValue,
      header:
        objectValue.header ??
        {{
          stamp: eventRosTime(event),
          frame_id: "",
        }},
      data: normalizeBytes(objectValue.data),
    }};
  }}

  return value;
}}

export function registerGeneratedSchemaConverters(extensionContext: ExtensionContext): void {{
{chr(10).join(registrations)}
}}
"""


def main() -> None:
    if not MSG_ROOT.exists():
        raise SystemExit(f"Message folder not found: {MSG_ROOT}")

    msg_index = load_msg_index(MSG_ROOT)
    converters = []

    for msg_name in sorted(msg_index):
        converters.extend(infer_convertibles(msg_name, msg_index[msg_name], msg_index))

    geojson_groups = len({item["from_schema"] for item in converters if item["to_schema"] == "foxglove_msgs/msg/GeoJSON"})
    non_geojson_count = sum(1 for item in converters if item["to_schema"] != "foxglove_msgs/msg/GeoJSON")

    OUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    OUT_FILE.write_text(generate_ts(converters))

    print(f"Found {len(converters)} convertible fields")
    print(f"Generated {geojson_groups} aggregated GeoJSON schema converters")
    print(f"Generated {non_geojson_count} non-GeoJSON schema converters")
    print(f"Wrote {OUT_FILE}")


if __name__ == "__main__":
    main()
