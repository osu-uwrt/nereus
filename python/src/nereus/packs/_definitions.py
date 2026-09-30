"""The single definition corpus: bundled JSON Schemas and the type registry derived from it.

``definitions/*.json`` is authoritative. Parameter definitions tagged ``x-category`` form the
registry; objects tagged ``x-typed`` are expanded here into a ``type`` enum plus per-type
``parameters`` schemas, so document schemas and ``types --json`` cannot drift apart.
"""

from __future__ import annotations

import json
from functools import lru_cache
from importlib import resources
from typing import Any

from jsonschema import Draft202012Validator

from ._document import DOCUMENT_KINDS

_SHARED = ("common", "types")
_DIALECT = "https://json-schema.org/draft/2020-12/schema"


def _load(name: str) -> dict[str, Any]:
    folder = resources.files(__package__).joinpath("definitions")
    text = folder.joinpath(f"{name}.json").read_text("utf-8")
    result: dict[str, Any] = json.loads(text)
    return result


def _rewrite(node: Any, owner: str) -> Any:
    """Rewrite cross-file refs to one flat bundle: common/types defs gain a prefix."""
    if isinstance(node, dict):
        result = {key: _rewrite(value, owner) for key, value in node.items()}
        reference = result.get("$ref")
        if isinstance(reference, str):
            document, _, pointer = reference.partition("#")
            source = document.removesuffix(".schema.json") if document else owner
            name = pointer.removeprefix("/$defs/")
            if not pointer.startswith("/$defs/") or "/" in name:
                raise AssertionError(f"unsupported reference {reference!r}")
            result["$ref"] = f"#/$defs/{_bundled_name(source, name)}"
        return result
    if isinstance(node, list):
        return [_rewrite(item, owner) for item in node]
    return node


def _bundled_name(source: str, name: str) -> str:
    return f"{source}__{name}" if source in _SHARED else name


@lru_cache(maxsize=None)
def _registry() -> dict[str, dict[str, str]]:
    """category -> type name -> bundled parameter definition name."""
    result: dict[str, dict[str, str]] = {}
    for name, definition in _load("types")["$defs"].items():
        category = definition.get("x-category")
        if category is not None:
            result.setdefault(category, {})[name] = _bundled_name("types", name)
    return {category: dict(sorted(types.items())) for category, types in sorted(result.items())}


def registry() -> dict[str, dict[str, str]]:
    """Return a detached catalog; callers cannot mutate loader definitions."""
    return {category: dict(types) for category, types in _registry().items()}


def _expand(node: Any) -> Any:
    if isinstance(node, dict):
        result = {key: _expand(value) for key, value in node.items() if key != "x-typed"}
        category = node.get("x-typed")
        if category is not None:
            types = registry()[category]
            properties = dict(result.get("properties", {}))
            properties["type"] = {"enum": list(types)}
            properties["parameters"] = {"type": "object"}
            result["properties"] = properties
            result["allOf"] = [
                {
                    "if": {"properties": {"type": {"const": name}}, "required": ["type"]},
                    "then": {"properties": {"parameters": {"$ref": f"#/$defs/{definition}"}}},
                }
                for name, definition in types.items()
            ]
        return result
    if isinstance(node, list):
        return [_expand(item) for item in node]
    return node


def _shared_defs() -> dict[str, Any]:
    defs: dict[str, Any] = {}
    for source in _SHARED:
        for name, definition in _load(source)["$defs"].items():
            defs[_bundled_name(source, name)] = _rewrite(definition, source)
    return defs


@lru_cache(maxsize=None)
def _schema(kind: str) -> str:
    if kind not in DOCUMENT_KINDS:
        raise KeyError(kind)
    document = _rewrite(_load(kind), kind)
    defs = _shared_defs()
    for name, definition in document.pop("$defs", {}).items():
        if name in defs:
            raise AssertionError(f"definition name collision: {name}")
        defs[name] = definition
    document.pop("$schema", None)
    bundle = {"$schema": _DIALECT, "$id": f"urn:nereus:pack:{kind}:1", **document}
    bundle["$defs"] = defs
    expanded = _expand(bundle)
    Draft202012Validator.check_schema(expanded)
    return json.dumps(expanded)


def schema(kind: str) -> dict[str, Any]:
    """Self-contained JSON Schema (draft 2020-12) for one document kind."""
    result: dict[str, Any] = json.loads(_schema(kind))
    return result


def type_catalog() -> dict[str, Any]:
    """Every registered built-in type and its parameter schema, grouped by category."""
    defs = _shared_defs()
    return {
        "$schema": _DIALECT,
        "$id": "urn:nereus:types:1",
        "categories": {
            category: {name: {"$ref": f"#/$defs/{definition}"} for name, definition in types.items()}
            for category, types in registry().items()
        },
        "$defs": _expand(defs),
    }


@lru_cache(maxsize=None)
def validator(kind: str) -> Draft202012Validator:
    return Draft202012Validator(json.loads(_schema(kind)))


def schema_problems(kind: str, data: dict[str, Any]) -> list[str]:
    problems = []
    errors = validator(kind).iter_errors(data)
    for error in sorted(errors, key=lambda item: [str(part) for part in item.path]):
        location = "/" + "/".join(str(part) for part in error.path)
        problems.append(f"{location}: {error.message[:300]}")
    return problems
