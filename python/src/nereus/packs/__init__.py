"""Data packs: strict YAML loading, lossless saving and scenario resolution.

Robot, pool, tasks, bridge and scenario packs are data only. Validation never constructs a
runtime or ROS interface; ``resolve_scenario`` produces the reproducible document the simulator
runs, and ``set_course`` edits a scenario's course in place.
"""

from ._course import set_course
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
    "set_course",
    "type_catalog",
]
