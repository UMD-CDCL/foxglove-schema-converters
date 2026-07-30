#!/usr/bin/env python3
"""Generates Foxglove schema converters for every convertible field of every .msg.

Schema converters are the default and require no configuration: any message that
gains a NavSatFix, image, bounding box, pose, audio or text field automatically
gains a converter the next time this runs.

Foxglove permits one converter per (fromSchemaName, toSchemaName) pair, so each
message has a limited number of output "slots". Fields are allocated to slots in
declaration order, each taking the first target it can:

  * the first NavSatFix takes the native `sensor_msgs/msg/NavSatFix` slot, which
    the Map panel renders directly;
  * any further location fields merge into one aggregated GeoJSON converter;
  * bounding boxes merge into one ImageAnnotations converter;
  * text fields merge into one Log converter;
  * everything else takes its own exclusively-typed slot.

Fields that find no free slot are reported as topic-converter candidates rather
than silently dropped. Fields claimed by a `split_paths` rule in
config/converters.json are skipped here, because a topic converter owns them.
"""
from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from converter_config import load_config  # noqa: E402
from emit import (  # noqa: E402
    GENERATED_BANNER,
    REPO_ROOT,
    remove_stale,
    sync_runtime,
    ts_literal,
    write_if_changed,
)
from rosmsg import (  # noqa: E402
    AUDIO_STAMP_FIELD_NAMES,
    MAX_PROPERTY_FIELDS,
    ConvertibleField,
    MessageIndex,
    build_index,
    humanize,
    pick_label_fields,
    resolve_msg_roots,
    walk_message,
)

DEFAULT_OUT_DIR = REPO_ROOT / "cdcl-schema-converters" / "src" / "generated"

STALE_FILES = [
    REPO_ROOT / "cdcl-schema-converters" / "src" / "generatedSchemaConverters.ts",
]


def geojson_entry(item: ConvertibleField, label: str) -> dict[str, object]:
    entry: dict[str, object] = {
        "path": list(item.path),
        "label": label,
        "geometry": item.geometry,
        "color": item.color,
    }

    property_fields = [name for name in item.sibling_fields if name != item.leaf]

    if property_fields:
        entry["propertyFields"] = property_fields[:MAX_PROPERTY_FIELDS]

    return entry


def annotation_entry(item: ConvertibleField) -> dict[str, object]:
    return {
        "containerPath": list(item.path[:-1]),
        "bboxField": item.leaf,
        "labelFields": list(pick_label_fields(item.sibling_fields)),
        "color": item.color,
    }


def audio_op(item: ConvertibleField, index: MessageIndex) -> dict[str, object]:
    op: dict[str, object] = {"kind": "audio", "path": list(item.path)}

    container = index.get(item.container_type) if item.container_type else None

    if container is not None:
        by_name = {f.name: f for f in container.fields}
        for candidate in AUDIO_STAMP_FIELD_NAMES:
            field = by_name.get(candidate)
            if field is not None and field.base_type == "builtin_interfaces/Time":
                op["stampPath"] = list(item.path[:-1]) + [candidate]
                break

    return op


def disambiguate_labels(items: list[ConvertibleField]) -> dict[str, str]:
    """Labels each field by its leaf name, falling back to a longer path on ties."""
    by_leaf: dict[str, list[ConvertibleField]] = defaultdict(list)

    for item in items:
        by_leaf[item.leaf].append(item)

    labels: dict[str, str] = {}

    for leaf, group in by_leaf.items():
        for item in group:
            if len(group) == 1:
                labels[item.dotted] = humanize(leaf)
            else:
                labels[item.dotted] = humanize(" ".join(item.path[-2:]))

    return labels


@dataclass
class Allocation:
    """How one message's convertible fields map onto Foxglove's output slots."""

    schema: str
    #: (field, op kind, output schema) for every field that found a slot.
    placed: list[tuple[ConvertibleField, str, str]]
    #: Output schema -> fields merged into a single aggregate converter.
    aggregated: dict[str, list[ConvertibleField]]
    #: (field, description of the conflict) for fields with no free slot.
    unplaced: list[tuple[ConvertibleField, str]]
    #: Fields a topic-converter rule claimed, so they are not emitted here.
    delegated: list[ConvertibleField]
    #: (path, type, reason) for displayable-looking fields with no target.
    skipped: list[tuple[str, str, str]]


def allocate_message(definition, index: MessageIndex, config) -> Allocation:
    """Assigns fields to output slots in declaration order, first free target wins."""
    schema = definition.full_name
    walked = walk_message(definition, index)

    split = config.split_paths_for(schema, walked.convertible)
    delegated = [item for item in walked.convertible if item.dotted in split]
    candidates = [item for item in walked.convertible if item.dotted not in split]

    exclusive: dict[str, ConvertibleField] = {}
    aggregated: dict[str, list[ConvertibleField]] = defaultdict(list)
    placed: list[tuple[ConvertibleField, str, str]] = []
    unplaced: list[tuple[ConvertibleField, str]] = []

    for item in candidates:
        for option in item.options:
            if option.aggregate:
                aggregated[option.schema].append(item)
                placed.append((item, option.op_kind, option.schema))
                break

            if option.schema not in exclusive:
                exclusive[option.schema] = item
                placed.append((item, option.op_kind, option.schema))
                break
        else:
            taken = ", ".join(
                f"{option.schema} (held by {exclusive[option.schema].dotted})"
                for option in item.options
                if option.schema in exclusive
            )
            unplaced.append((item, taken))

    return Allocation(
        schema=schema,
        placed=placed,
        aggregated=dict(aggregated),
        unplaced=unplaced,
        delegated=delegated,
        skipped=walked.skipped,
    )


def allocate_all(index: MessageIndex, config) -> list[Allocation]:
    return [allocate_message(definition, index, config) for definition in index.messages()]


def build_specs(index: MessageIndex, config) -> tuple[list[dict[str, object]], list[str]]:
    specs: list[dict[str, object]] = []
    notes: list[str] = []

    for allocation in allocate_all(index, config):
        schema = allocation.schema
        aggregated = allocation.aggregated

        if allocation.delegated:
            notes.append(
                f"{schema}: {len(allocation.delegated)} field(s) delegated to topic converters "
                f"({', '.join(item.dotted for item in allocation.delegated)})"
            )

        for item, op_kind, to_schema in allocation.placed:
            if op_kind in {"geojson", "image_annotations", "log"}:
                continue

            if op_kind == "audio":
                op: dict[str, object] = audio_op(item, index)
            else:
                op = {"kind": op_kind, "path": list(item.path)}

            specs.append({"fromSchemaName": schema, "toSchemaName": to_schema, "op": op})

        geojson_items = aggregated.get("foxglove_msgs/msg/GeoJSON", [])
        if geojson_items:
            labels = disambiguate_labels(geojson_items)
            specs.append(
                {
                    "fromSchemaName": schema,
                    "toSchemaName": "foxglove_msgs/msg/GeoJSON",
                    "op": {
                        "kind": "geojson",
                        "entries": [
                            geojson_entry(item, labels[item.dotted]) for item in geojson_items
                        ],
                    },
                }
            )

        annotation_items = aggregated.get("foxglove_msgs/msg/ImageAnnotations", [])
        if annotation_items:
            specs.append(
                {
                    "fromSchemaName": schema,
                    "toSchemaName": "foxglove_msgs/msg/ImageAnnotations",
                    "op": {
                        "kind": "image_annotations",
                        "entries": [annotation_entry(item) for item in annotation_items],
                    },
                }
            )

        log_items = aggregated.get("foxglove_msgs/msg/Log", [])
        if log_items:
            labels = disambiguate_labels(log_items)
            specs.append(
                {
                    "fromSchemaName": schema,
                    "toSchemaName": "foxglove_msgs/msg/Log",
                    "op": {
                        "kind": "log",
                        "entries": [
                            {"path": list(item.path), "label": labels[item.dotted]}
                            for item in log_items
                        ],
                    },
                }
            )

        for item, taken in allocation.unplaced:
            notes.append(
                f"{schema}: no free output slot for '{item.dotted}' ({item.base_type}); "
                f"{taken or 'all targets taken'}. Add a topic converter rule in "
                f"config/converters.json to expose it."
            )

        for dotted, base_type, reason in allocation.skipped:
            notes.append(f"{schema}: skipped '{dotted}' ({base_type}) - {reason}")

    specs.sort(key=lambda spec: (spec["fromSchemaName"], spec["toSchemaName"]))

    seen: set[tuple[str, str]] = set()
    for spec in specs:
        key = (str(spec["fromSchemaName"]), str(spec["toSchemaName"]))
        if key in seen:
            raise SystemExit(
                f"Internal error: duplicate schema converter for {key[0]} -> {key[1]}. "
                "Foxglove allows only one converter per schema pair."
            )
        seen.add(key)

    return specs, notes


def render_module(specs: list[dict[str, object]]) -> str:
    banner = GENERATED_BANNER.format(script="scripts/generate_schema_converters.py")

    return f"""{banner}
//
// One entry per (fromSchemaName, toSchemaName) pair. All conversion logic lives
// in ./converterRuntime.ts; this file is data only.

import {{ SchemaConverterSpec }} from "./converterRuntime";

export const SCHEMA_CONVERTER_SPECS: readonly SchemaConverterSpec[] = {ts_literal(specs)};
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "msg_roots",
        nargs="*",
        help="ROS message directories. Defaults to $CDCL_MSG_ROOTS or a known checkout.",
    )
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument(
        "--quiet", action="store_true", help="Suppress the per-message notes report."
    )
    args = parser.parse_args()

    msg_roots = resolve_msg_roots(args.msg_roots)
    index = build_index(msg_roots)
    config = load_config(args.config)

    specs, notes = build_specs(index, config)

    sync_runtime(args.out_dir)
    changed = write_if_changed(args.out_dir / "schemaConverterSpecs.ts", render_module(specs))
    removed = remove_stale(STALE_FILES)

    by_target: dict[str, int] = defaultdict(int)
    for spec in specs:
        by_target[str(spec["toSchemaName"])] += 1

    print(f"Message roots: {', '.join(str(root) for root in msg_roots)}")
    print(f"Indexed {len(index)} message definitions")
    print(f"Generated {len(specs)} schema converters across {len({s['fromSchemaName'] for s in specs})} schemas")

    for target in sorted(by_target):
        print(f"  {by_target[target]:3d}  -> {target}")

    if notes and not args.quiet:
        print("\nNotes:")
        for note in notes:
            print(f"  - {note}")

    for path in removed:
        print(f"\nRemoved superseded file: {path.relative_to(REPO_ROOT)}")

    print(f"\nWrote {(args.out_dir / 'schemaConverterSpecs.ts').relative_to(REPO_ROOT)}"
          f"{'' if changed else ' (unchanged)'}")


if __name__ == "__main__":
    main()
