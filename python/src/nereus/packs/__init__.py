"""Data packs: strict YAML loading, lossless saving and scenario resolution.

Robot, pool, tasks, bridge and scenario packs are data only. Validation never imports task
hooks and never constructs a runtime or ROS interface; ``resolve_scenario`` produces the
reproducible manifest a runtime factory consumes.
"""

from ._definitions import registry, schema, type_catalog
from ._document import DOCUMENT_KINDS, PACK_KINDS, PackDocument, PackError
from ._resolve import ResolvedScenario, load_pack, resolve_scenario

__all__ = [
    "DOCUMENT_KINDS",
    "PACK_KINDS",
    "PackDocument",
    "PackError",
    "ResolvedScenario",
    "load_pack",
    "registry",
    "resolve_scenario",
    "schema",
    "type_catalog",
]
