#!/usr/bin/env python3
"""Generates the minimum set of Foxglove topic converters.

Topic converters exist only for fields that a schema converter physically cannot
expose: several same-typed outputs from one source schema. Foxglove allows one
converter per (fromSchemaName, toSchemaName) pair, so a TargetBoxArray can render
only one GeoJSON layer via a schema converter, while its `uav_target_boxes` carry
three alternative localizations that each need their own Map layer.

Which fields those are is not hard-coded. Each rule in config/converters.json
names a schema and the field-path prefixes it claims; the leaf fields are then
discovered from the .msg definitions, so adding a fourth localization to
TargetBox.msg produces a fourth output topic with no change to this script.
"""
from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from converter_config import TopicConverterRule, load_config  # noqa: E402
from emit import (  # noqa: E402
    GENERATED_BANNER,
    REPO_ROOT,
    remove_stale,
    sync_runtime,
    ts_literal,
    write_if_changed,
)
from rosmsg import (  # noqa: E402
    MAX_PROPERTY_FIELDS,
    ConvertibleField,
    MessageIndex,
    build_index,
    humanize,
    pick_label_fields,
    resolve_msg_roots,
    walk_message,
)

DEFAULT_OUT_DIR = REPO_ROOT / "cdcl-topic-converters" / "src" / "generated"

STALE_FILES = [
    REPO_ROOT / "cdcl-topic-converters" / "src" / "generatedTbaTopicConverters.ts",
]


def derive_keys(items: list[ConvertibleField]) -> dict[str, str]:
    """Derives an output-topic suffix per field, stripping the prefix its siblings share.

    `target_location_altimeter_plane` / `target_location_gimbal_plane` /
    `target_location_rangefinder` all share `target_location_`, so they become
    `altimeter_plane` / `gimbal_plane` / `rangefinder`.
    """
    keys: dict[str, str] = {}
    by_container: dict[tuple[str, ...], list[ConvertibleField]] = defaultdict(list)

    for item in items:
        by_container[item.path[:-1]].append(item)

    for group in by_container.values():
        token_lists = [item.leaf.split("_") for item in group]
        shared = 0

        if len(group) > 1:
            shortest = min(len(tokens) for tokens in token_lists)
            while shared < shortest - 1 and len({tokens[shared] for tokens in token_lists}) == 1:
                shared += 1

        for item, tokens in zip(group, token_lists):
            keys[item.dotted] = "_".join(tokens[shared:]) or item.leaf

    return keys


def build_op(item: ConvertibleField, key: str) -> tuple[dict[str, object], str]:
    """Builds a ConverterOp and its output schema from the field's primary target."""
    option = item.options[0]

    if option.op_kind == "geojson":
        entry: dict[str, object] = {
            "path": list(item.path),
            "label": humanize(key),
            "geometry": item.geometry,
            "color": item.color,
        }

        property_fields = [name for name in item.sibling_fields if name != item.leaf]

        if property_fields:
            entry["propertyFields"] = property_fields[:MAX_PROPERTY_FIELDS]

        return {"kind": "geojson", "entries": [entry]}, option.schema

    if option.op_kind == "image_annotations":
        return (
            {
                "kind": "image_annotations",
                "entries": [
                    {
                        "containerPath": list(item.path[:-1]),
                        "bboxField": item.leaf,
                        "labelFields": list(pick_label_fields(item.sibling_fields)),
                        "color": item.color,
                    }
                ],
            },
            option.schema,
        )

    if option.op_kind == "log":
        return (
            {"kind": "log", "entries": [{"path": list(item.path), "label": humanize(key)}]},
            option.schema,
        )

    return {"kind": option.op_kind, "path": list(item.path)}, option.schema


def fields_for_rule(
    rule: TopicConverterRule, index: MessageIndex
) -> tuple[list[ConvertibleField], list[str]]:
    notes: list[str] = []
    definition = index.get(rule.schema)

    if definition is None:
        raise SystemExit(
            f"{rule.schema} is declared in the converter config but no matching "
            f".msg was found in the indexed message roots."
        )

    walked = walk_message(definition, index)
    owned = rule.claimed_fields(walked.convertible)

    for prefix in rule.split_paths:
        covered = [item for item in walked.convertible if item.dotted.startswith(prefix)]

        if not covered:
            notes.append(
                f"{rule.schema}: split_path '{prefix}' matched no convertible field. "
                f"Check the path against {definition.path.name}."
            )
            continue

        left_behind = [item for item in covered if item not in owned]

        if left_behind:
            notes.append(
                f"{rule.schema}: '{prefix}' also covers "
                f"{', '.join(item.dotted for item in left_behind)}, which had no rival for "
                f"an output slot and stayed on the schema converter."
            )

    return owned, notes


def build_specs(index: MessageIndex, config) -> tuple[list[dict[str, object]], list[str]]:
    specs: list[dict[str, object]] = []
    notes: list[str] = []

    for rule in config.rules:
        owned, rule_notes = fields_for_rule(rule, index)
        notes.extend(rule_notes)

        if not owned:
            continue

        keys = derive_keys(owned)
        keys.update({path: key for path, key in rule.key_overrides.items() if path in keys})

        unknown_overrides = [path for path in rule.key_overrides if path not in keys]
        for path in unknown_overrides:
            notes.append(
                f"{rule.schema}: key_override '{path}' does not match any split field "
                f"and was ignored."
            )

        input_topics = rule.expand_topics()

        if not input_topics:
            notes.append(f"{rule.schema}: rule declares no topics, so nothing was generated.")
            continue

        for input_topic in input_topics:
            for item in owned:
                key = keys[item.dotted]
                op, output_schema = build_op(item, key)
                output_topic = rule.output_topic_template.format(
                    input_topic=input_topic, key=key, field=item.leaf
                )

                specs.append(
                    {
                        "inputTopic": input_topic,
                        "outputTopic": output_topic,
                        "outputSchemaName": output_schema,
                        "op": op,
                    }
                )

    specs.sort(key=lambda spec: (spec["inputTopic"], spec["outputTopic"]))

    seen: set[str] = set()
    for spec in specs:
        output_topic = str(spec["outputTopic"])
        if output_topic in seen:
            raise SystemExit(
                f"Duplicate output topic '{output_topic}'. Two rules or two fields "
                f"resolved to the same name; use key_overrides to disambiguate."
            )
        seen.add(output_topic)

    return specs, notes


def render_module(specs: list[dict[str, object]]) -> str:
    banner = GENERATED_BANNER.format(script="scripts/generate_topic_converters.py")

    return f"""{banner}
//
// One entry per output topic. All conversion logic lives in
// ./converterRuntime.ts; this file is data only.

import {{ TopicConverterSpec }} from "./converterRuntime";

export const TOPIC_CONVERTER_SPECS: readonly TopicConverterSpec[] = {ts_literal(specs)};
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
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    msg_roots = resolve_msg_roots(args.msg_roots)
    index = build_index(msg_roots)
    config = load_config(args.config)

    specs, notes = build_specs(index, config)

    sync_runtime(args.out_dir)
    changed = write_if_changed(args.out_dir / "topicConverterSpecs.ts", render_module(specs))
    removed = remove_stale(STALE_FILES)

    input_topics = sorted({str(spec["inputTopic"]) for spec in specs})

    print(f"Config: {config.path.relative_to(REPO_ROOT) if config.path.exists() else config.path}")
    print(f"Generated {len(specs)} topic converters over {len(input_topics)} input topics")

    for input_topic in input_topics:
        outputs = [
            str(spec["outputTopic"]) for spec in specs if str(spec["inputTopic"]) == input_topic
        ]
        print(f"  {input_topic} -> {', '.join(sorted(outputs))}")

    if notes and not args.quiet:
        print("\nNotes:")
        for note in notes:
            print(f"  - {note}")

    for path in removed:
        print(f"\nRemoved superseded file: {path.relative_to(REPO_ROOT)}")

    print(f"\nWrote {(args.out_dir / 'topicConverterSpecs.ts').relative_to(REPO_ROOT)}"
          f"{'' if changed else ' (unchanged)'}")


if __name__ == "__main__":
    main()
