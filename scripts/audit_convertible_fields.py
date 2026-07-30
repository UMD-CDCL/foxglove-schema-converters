#!/usr/bin/env python3
"""Reports what each ROS message exposes to Foxglove, and what it cannot.

Runs the exact allocation the schema generator uses, so the report never drifts
from the generated output. Use it after changing a .msg file to see which new
fields became convertible, and which ones need a topic-converter rule because
they collide with another field for the same output slot.

    python3 scripts/audit_convertible_fields.py
    python3 scripts/audit_convertible_fields.py --only TargetBoxArray
    python3 scripts/audit_convertible_fields.py --candidates-only
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from converter_config import load_config  # noqa: E402
from generate_schema_converters import allocate_all  # noqa: E402
from rosmsg import build_index, resolve_msg_roots  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("msg_roots", nargs="*")
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--only", default=None, help="Substring filter on the message name.")
    parser.add_argument(
        "--candidates-only",
        action="store_true",
        help="Show only messages with fields that need a topic converter.",
    )
    args = parser.parse_args()

    msg_roots = resolve_msg_roots(args.msg_roots)
    index = build_index(msg_roots)
    config = load_config(args.config)

    allocations = allocate_all(index, config)

    total_fields = 0
    total_candidates = 0
    candidate_lines: list[str] = []

    for allocation in allocations:
        if args.only and args.only.lower() not in allocation.schema.lower():
            continue

        has_content = bool(
            allocation.placed or allocation.delegated or allocation.unplaced or allocation.skipped
        )

        if not has_content:
            continue

        if args.candidates_only and not allocation.unplaced:
            continue

        total_fields += len(allocation.placed)
        total_candidates += len(allocation.unplaced)

        print(f"\n{allocation.schema}")

        aggregate_schemas = set(allocation.aggregated)

        for item, op_kind, to_schema in allocation.placed:
            grouped = ""
            if to_schema in aggregate_schemas and len(allocation.aggregated[to_schema]) > 1:
                grouped = f"  [merged with {len(allocation.aggregated[to_schema]) - 1} other field(s)]"

            cardinality = "array" if item.through_array else "scalar"
            print(f"  schema   {item.dotted}  ({item.base_type}, {cardinality})")
            print(f"           -> {to_schema} via {op_kind}{grouped}")

        for item in allocation.delegated:
            print(f"  topic    {item.dotted}  ({item.base_type})")
            print("           -> own output topic (see config/converters.json)")

        for item, taken in allocation.unplaced:
            print(f"  BLOCKED  {item.dotted}  ({item.base_type})")
            print(f"           {taken or 'no free output slot'}")
            candidate_lines.append(f"{allocation.schema}  {item.dotted}")

        for dotted, base_type, reason in allocation.skipped:
            print(f"  skipped  {dotted}  ({base_type})")
            print(f"           {reason}")

    print(f"\n{'=' * 72}")
    print(f"{total_fields} field(s) reachable through schema converters")
    print(f"{total_candidates} field(s) blocked and needing a topic converter")

    if candidate_lines:
        print("\nAdd a rule to config/converters.json for these, for example:")
        print('  { "schema": "<schema>", "split_paths": ["<path prefix>"],')
        print('    "topics": ["/your/topic"] }')
        print()
        for line in candidate_lines:
            print(f"  {line}")


if __name__ == "__main__":
    main()
