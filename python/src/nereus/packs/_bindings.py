"""Task-file values bound to fixed run options of the tasks pack.

Where the schema allows it (a visual's ``texture``, an open crate's ``class``), a task file may write
``{option: <key>}`` or ``{option: <key>, values: {<choice>: <value>, ...}}``. The key names a ``fixed`` choice
option of the tasks pack. Resolving a scenario replaces each binding with the chosen choice, or the value
``values`` maps it to, so the runtime only ever sees plain values.
"""

from __future__ import annotations

from collections.abc import Iterator
from typing import Any


def is_binding(node: Any) -> bool:
    """An ``option`` key with at most a ``values`` map beside it (the schema allows these only where a value
    may be bound)."""
    return isinstance(node, dict) and "option" in node and set(node) <= {"option", "values"}


def find(node: Any, pointer: str = "") -> Iterator[tuple[str, dict[str, Any]]]:
    """(JSON pointer, binding) for every binding under `node`."""
    if is_binding(node):
        yield pointer, node
    elif isinstance(node, dict):
        for key, value in node.items():
            yield from find(value, f"{pointer}/{key}")
    elif isinstance(node, list):
        for index, value in enumerate(node):
            yield from find(value, f"{pointer}/{index}")


def outcomes(binding: dict[str, Any], options: dict[str, dict[str, Any]] | None) -> list[str]:
    """Every value the binding can take; empty when that depends on run options that aren't known."""
    if "values" in binding:
        return list(binding["values"].values())
    option = (options or {}).get(binding["option"])
    return list(option["choices"]) if option and option["type"] == "choice" else []


def check(data: dict[str, Any], options: dict[str, dict[str, Any]] | None) -> list[str]:
    """Problems with one task file's bindings against the tasks pack's run options (by key; None when the
    file is checked on its own)."""
    problems: list[str] = []
    if options is None:
        return problems
    for pointer, binding in find(data):
        key = binding["option"]
        option = options.get(key)
        if option is None:
            problems.append(f"{pointer}: unknown run option '{key}'")
        elif not option.get("fixed", False):
            problems.append(
                f"{pointer}: run option '{key}' is not fixed (a run Start could change it)"
            )
        elif option["type"] != "choice":
            problems.append(f"{pointer}: run option '{key}' is not a choice")
        elif "values" in binding and set(binding["values"]) != set(option["choices"]):
            problems.append(f"{pointer}/values: must map exactly the choices {option['choices']}")
    return problems


def apply(node: Any, values: dict[str, Any]) -> Any:
    """`node` with every binding replaced by its value under the resolved run option values."""
    if is_binding(node):
        chosen = values[node["option"]]
        return node["values"][chosen] if "values" in node else chosen
    if isinstance(node, dict):
        return {key: apply(value, values) for key, value in node.items()}
    if isinstance(node, list):
        return [apply(value, values) for value in node]
    return node
