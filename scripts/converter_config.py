#!/usr/bin/env python3
"""Loads config/converters.json, the policy that decides schema vs topic converters.

Foxglove allows exactly one message converter per (fromSchemaName, toSchemaName)
pair, so a message with several alternative localizations can expose only one of
them through a schema converter. This config declares the fields that must
instead be split into their own output topics, and the topics to split them on.

The same declarations are read by both generators, so a field can never be
emitted twice (once merged into a schema converter, once as its own topic).
"""
from __future__ import annotations

import itertools
import json
import re
from dataclasses import dataclass, field as dataclass_field
from pathlib import Path
from typing import Sequence

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "converters.json"

_VARIABLE_PATTERN = re.compile(r"\{([A-Za-z_][A-Za-z0-9_]*)\}")


def strip_json_comments(text: str) -> str:
    """Removes `//` line comments that fall outside string literals."""
    out: list[str] = []
    in_string = False
    escaped = False
    index = 0

    while index < len(text):
        char = text[index]

        if in_string:
            out.append(char)
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            index += 1
            continue

        if char == '"':
            in_string = True
            out.append(char)
            index += 1
            continue

        if char == "/" and index + 1 < len(text) and text[index + 1] == "/":
            while index < len(text) and text[index] != "\n":
                index += 1
            continue

        out.append(char)
        index += 1

    return "".join(out)


@dataclass
class TopicConverterRule:
    schema: str
    #: Dotted path prefixes whose location fields become their own output topics.
    split_paths: tuple[str, ...]
    #: Topic name patterns, optionally containing `{variable}` placeholders.
    topics: tuple[str, ...]
    variables: dict[str, list[object]]
    output_topic_template: str = "{input_topic}/{key}"
    #: Dotted field path -> output topic suffix, overriding the derived name.
    key_overrides: dict[str, str] = dataclass_field(default_factory=dict)
    #: Output schemas to split. Empty means "split only where fields contend".
    split_schemas: tuple[str, ...] = ()
    reason: str = ""

    def expand_topics(self) -> list[str]:
        expanded: list[str] = []

        for pattern in self.topics:
            names = _VARIABLE_PATTERN.findall(pattern)
            missing = [name for name in names if name not in self.variables]

            if missing:
                raise SystemExit(
                    f"Topic pattern {pattern!r} for {self.schema} uses undefined "
                    f"variable(s): {', '.join(sorted(set(missing)))}"
                )

            if not names:
                expanded.append(pattern)
                continue

            ordered = list(dict.fromkeys(names))
            for combo in itertools.product(*(self.variables[name] for name in ordered)):
                expanded.append(pattern.format(**dict(zip(ordered, combo))))

        # Preserve order while removing accidental duplicates.
        return list(dict.fromkeys(expanded))

    def covers_path(self, dotted_path: str) -> bool:
        """True when a field sits at or under one of this rule's split prefixes."""
        for prefix in self.split_paths:
            if dotted_path == prefix or dotted_path.startswith(f"{prefix}."):
                return True
        return False

    def claimed_fields(self, items: Sequence) -> list:
        """Selects the fields under this rule's prefixes that genuinely need a topic.

        A field only needs its own topic when it *contends* for an output slot:
        two or more fields under the prefix that resolve to the same output
        schema cannot all be shown separately by a single schema converter. A
        lone field (a bounding box, say) has no rival, so it is left to the
        schema converter — keeping topic converters to the necessary minimum.

        Setting `split_schemas` overrides the heuristic and splits every field
        targeting the listed schemas, contended or not.
        """
        under = [item for item in items if self.covers_path(item.dotted) and item.options]
        by_target: dict[str, list] = {}

        for item in under:
            by_target.setdefault(item.options[0].schema, []).append(item)

        claimed: list = []

        for target, group in by_target.items():
            if self.split_schemas:
                if target in self.split_schemas:
                    claimed.extend(group)
            elif len(group) > 1:
                claimed.extend(group)

        # Preserve the original declaration order.
        claimed_paths = {item.dotted for item in claimed}
        return [item for item in under if item.dotted in claimed_paths]


@dataclass
class ConverterConfig:
    rules: tuple[TopicConverterRule, ...]
    path: Path

    def rules_for(self, schema: str) -> list[TopicConverterRule]:
        return [rule for rule in self.rules if rule.schema == schema]

    def split_paths_for(self, schema: str, items: Sequence) -> set[str]:
        """Dotted paths a topic converter owns, which schema converters must skip."""
        claimed: set[str] = set()

        for rule in self.rules_for(schema):
            claimed.update(item.dotted for item in rule.claimed_fields(items))

        return claimed


def _require(mapping: dict, key: str, context: str) -> object:
    if key not in mapping:
        raise SystemExit(f"{context}: missing required key {key!r}")
    return mapping[key]


def load_config(path: Path | None = None) -> ConverterConfig:
    config_path = path or DEFAULT_CONFIG_PATH

    if not config_path.exists():
        # An absent config is valid: everything then goes through schema converters.
        return ConverterConfig(rules=(), path=config_path)

    try:
        raw = json.loads(strip_json_comments(config_path.read_text()))
    except json.JSONDecodeError as error:
        raise SystemExit(f"{config_path}: invalid JSON ({error})") from error

    if not isinstance(raw, dict):
        raise SystemExit(f"{config_path}: expected a JSON object at the top level")

    rules: list[TopicConverterRule] = []

    for position, entry in enumerate(raw.get("topic_converters", [])):
        context = f"{config_path}: topic_converters[{position}]"

        if not isinstance(entry, dict):
            raise SystemExit(f"{context}: expected an object")

        schema = str(_require(entry, "schema", context))
        topics = entry.get("topics", [])
        split_paths = entry.get("split_paths", [])

        if not isinstance(topics, list) or not all(isinstance(item, str) for item in topics):
            raise SystemExit(f"{context}: 'topics' must be a list of strings")

        if not isinstance(split_paths, list) or not all(
            isinstance(item, str) for item in split_paths
        ):
            raise SystemExit(f"{context}: 'split_paths' must be a list of strings")

        variables = entry.get("variables", {})

        if not isinstance(variables, dict) or not all(
            isinstance(value, list) for value in variables.values()
        ):
            raise SystemExit(f"{context}: 'variables' must map names to lists")

        key_overrides = entry.get("key_overrides", {})

        if not isinstance(key_overrides, dict):
            raise SystemExit(f"{context}: 'key_overrides' must be an object")

        split_schemas = entry.get("split_schemas", [])

        if not isinstance(split_schemas, list) or not all(
            isinstance(item, str) for item in split_schemas
        ):
            raise SystemExit(f"{context}: 'split_schemas' must be a list of strings")

        rules.append(
            TopicConverterRule(
                schema=schema,
                split_paths=tuple(split_paths),
                topics=tuple(topics),
                variables={str(k): list(v) for k, v in variables.items()},
                output_topic_template=str(
                    entry.get("output_topic_template", "{input_topic}/{key}")
                ),
                key_overrides={str(k): str(v) for k, v in key_overrides.items()},
                split_schemas=tuple(split_schemas),
                reason=str(entry.get("reason", "")),
            )
        )

    return ConverterConfig(rules=tuple(rules), path=config_path)
