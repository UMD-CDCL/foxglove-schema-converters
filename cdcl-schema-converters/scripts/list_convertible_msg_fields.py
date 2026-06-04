#!/usr/bin/env python3
from pathlib import Path
import re
import sys

MSG_ROOT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/ros_ws/src/cdcl_umd_msgs/msg")

CONVERTIBLE_TYPES = {
    "sensor_msgs/CompressedImage": "image",
    "sensor_msgs/Image": "image",
    "foxglove_msgs/RawAudio": "audio",
    "audio_common_msgs/AudioData": "audio",
    "audio_common_msgs/AudioDataStamped": "audio",
}

def parse_msg(path: Path):
    fields = []
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue

        # Skip constants like uint8 FOO=1
        if "=" in line:
            continue

        parts = re.split(r"\s+", line)
        if len(parts) < 2:
            continue

        field_type, field_name = parts[0], parts[1]
        fields.append((field_type, field_name))

    return fields

def infer_convertibles(fields):
    results = []

    field_map = {name: typ for typ, name in fields}

    for field_type, field_name in fields:
        if field_type in CONVERTIBLE_TYPES:
            results.append((field_name, CONVERTIBLE_TYPES[field_type], field_type))

        # Project-specific heuristic:
        # ObservationDataSource-style raw PCM bytes.
        if field_type == "uint8[]" and field_name in {"raw_audio", "audio", "audio_data"}:
            results.append((field_name, "audio", "uint8[] raw PCM bytes"))

        # Optional useful text fields.
        if field_type == "string" and any(k in field_name.lower() for k in ["transcript", "caption", "text"]):
            results.append((field_name, "text", "string"))

    return results

def main():
    if not MSG_ROOT.exists():
        raise SystemExit(f"Message folder not found: {MSG_ROOT}")

    for msg_file in sorted(MSG_ROOT.glob("*.msg")):
        fields = parse_msg(msg_file)
        convertibles = infer_convertibles(fields)

        if not convertibles:
            continue

        print(f"{msg_file.stem}:")
        for field_name, target, reason in convertibles:
            print(f"  - {field_name}: {target} ({reason})")

if __name__ == "__main__":
    main()
