"""Command line: ``python -m robotics_platform.packs {validate,types,schema}``."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from . import DOCUMENT_KINDS, PackError, load_pack, registry, resolve_scenario, schema, type_catalog
from ._document import canonical_file, read_yaml


def _validate(arguments: argparse.Namespace) -> int:
    path = Path(arguments.pack)
    try:
        kind = read_yaml(canonical_file(path)).get("kind")
        if kind == "scenario":
            resolved = resolve_scenario(path, strict=arguments.strict)
            if arguments.dump is not None:
                resolved.dump(Path(arguments.dump))
            gaps = resolved.unresolved
        else:
            if arguments.dump is not None:
                raise PackError("--dump requires a scenario")
            gaps = load_pack(path, strict=arguments.strict).unresolved
    except PackError as error:
        print(f"INVALID {path}", file=sys.stderr)
        for problem in error.problems:
            print(f"  {problem}", file=sys.stderr)
        return 1
    print(f"OK {kind} {path}")
    for item in gaps:
        step = item["required_from_step"]
        print(f"  unresolved {item['kind']} {item['pack']}:{item['id']}"
              + (f" (step {step})" if step is not None else ""))
    return 0


def _types(arguments: argparse.Namespace) -> int:
    if arguments.json:
        print(json.dumps(type_catalog(), indent=2))
    else:
        for category, types in registry().items():
            print(f"{category}: {', '.join(types)}")
    return 0


def _schema(arguments: argparse.Namespace) -> int:
    print(json.dumps(schema(arguments.kind), indent=2))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m robotics_platform.packs")
    commands = parser.add_subparsers(dest="command", required=True)
    validate = commands.add_parser("validate", help="validate a pack or scenario (no runtime/ROS)")
    validate.add_argument("pack", help="pack folder or file")
    validate.add_argument("--strict", action="store_true", help="reject unresolved dependencies")
    validate.add_argument("--dump", metavar="JSON", help="write the resolved scenario manifest")
    validate.set_defaults(run=_validate)
    types = commands.add_parser("types", help="list built-in types")
    types.add_argument("--json", action="store_true", help="parameter schemas as JSON")
    types.set_defaults(run=_types)
    kind = commands.add_parser("schema", help="print a document kind's JSON Schema")
    kind.add_argument("kind", choices=DOCUMENT_KINDS)
    kind.set_defaults(run=_schema)
    arguments = parser.parse_args(argv)
    result: int = arguments.run(arguments)
    return result


if __name__ == "__main__":
    sys.exit(main())
