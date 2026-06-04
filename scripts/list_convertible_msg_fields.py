#!/usr/bin/env python3
from pathlib import Path
import re
import sys

MSG_ROOT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/ros_ws/src/cdcl_umd_msgs/msg")

SCALAR_CONVERTIBLE_TYPES = {
    "sensor_msgs/Image": "image",
    "sensor_msgs/CompressedImage": "image",

    "foxglove_msgs/RawAudio": "audio",
    "audio_common_msgs/AudioData": "audio",
    "audio_common_msgs/AudioDataStamped": "audio",

    "sensor_msgs/NavSatFix": "location",
    "gps_msgs/GPSFix": "location",

    "sensor_msgs/Imu": "imu",
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
    "geometry_msgs/Twist": "velocity",
    "geometry_msgs/TwistStamped": "velocity",
    "geometry_msgs/TwistWithCovariance": "velocity",
    "geometry_msgs/TwistWithCovarianceStamped": "velocity",

    "nav_msgs/Path": "path",
    "trajectory_msgs/JointTrajectory": "trajectory",
    "trajectory_msgs/MultiDOFJointTrajectory": "trajectory",

    "sensor_msgs/PointCloud2": "pointcloud",
    "sensor_msgs/LaserScan": "laserscan",

    "visualization_msgs/Marker": "marker",
    "visualization_msgs/MarkerArray": "marker_array",
}

ARRAY_TARGET_OVERRIDES = {
    "sensor_msgs/NavSatFix": "location_array",
    "gps_msgs/GPSFix": "location_array",

    "geometry_msgs/Point": "point_array",
    "geometry_msgs/Pose": "pose_array",
    "geometry_msgs/PoseStamped": "pose_array",
    "geometry_msgs/Vector3": "vector_array",
}

POLYGON_LOCATION_FIELD_HINTS = (
    "polygon",
    "fence",
    "domain",
    "zone",
    "boundary",
    "bounds",
    "perimeter",
)

TEXT_NAME_HINTS = ("transcript", "caption", "text", "description", "label", "status")
AUDIO_BYTE_FIELD_NAMES = {"raw_audio", "audio", "audio_data"}

MAX_NESTING_DEPTH = 3


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


def recommended_output(target: str, source_type: str) -> str:
    if target in {"location", "location_array", "polygon"}:
        return "foxglove_msgs/msg/GeoJSON"
    if target == "image" and source_type == "sensor_msgs/CompressedImage":
        return "sensor_msgs/msg/CompressedImage"
    if target == "image" and source_type == "sensor_msgs/Image":
        return "sensor_msgs/msg/Image"
    if target == "audio":
        return "foxglove_msgs/msg/RawAudio"
    return target


def make_result(
    msg_name: str,
    field_path: str,
    target: str,
    source_type: str,
    base_type: str,
    cardinality: str,
) -> dict[str, str]:
    return {
        "field": field_path,
        "target": target,
        "source_type": source_type,
        "base_type": base_type,
        "cardinality": cardinality,
        "converter": converter_name(msg_name, field_path, target),
        "recommended_output": recommended_output(target, source_type),
    }


def infer_field(
    root_msg_name: str,
    field_type: str,
    field_path: str,
    msg_index: dict[str, list[tuple[str, str]]],
    depth: int,
    parent_is_array: bool = False,
) -> list[dict[str, str]]:
    results = []

    base_type, is_array = parse_field_type(field_type)
    effective_is_array = parent_is_array or is_array
    cardinality = "array" if effective_is_array else "scalar"

    target = classify_target(base_type, field_path, effective_is_array)
    if target is not None:
        results.append(
            make_result(root_msg_name, field_path, target, field_type, base_type, cardinality)
        )
        return results

    if field_type == "uint8[]" and field_path.split(".")[-1] in AUDIO_BYTE_FIELD_NAMES:
        target = "audio"
        results.append(
            make_result(root_msg_name, field_path, target, field_type, "uint8", "array")
        )
        return results

    if field_type == "string" and any(hint in field_path.lower() for hint in TEXT_NAME_HINTS):
        target = "text"
        results.append(
            make_result(root_msg_name, field_path, target, field_type, "string", cardinality)
        )
        return results

    nested_msg_name = local_type_name(base_type)

    if depth >= MAX_NESTING_DEPTH or nested_msg_name not in msg_index:
        return results

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


def main() -> None:
    if not MSG_ROOT.exists():
        raise SystemExit(f"Message folder not found: {MSG_ROOT}")

    msg_index = load_msg_index(MSG_ROOT)

    for msg_name in sorted(msg_index):
        convertibles = infer_convertibles(msg_name, msg_index[msg_name], msg_index)

        if not convertibles:
            continue

        print(f"{msg_name}:")

        for item in convertibles:
            print(
                f"  - {item['field']}: {item['target']} "
                f"({item['source_type']}, {item['cardinality']}) "
                f"-> {item['converter']} "
                f"[{item['recommended_output']}]"
            )


if __name__ == "__main__":
    main()
