"""Round-trip YAML pack documents: strict loading, lossless saving, plain-data views."""

from __future__ import annotations

import hashlib
import math
import os
import stat
import tempfile
from dataclasses import dataclass, field
from io import StringIO
from pathlib import Path
from typing import Any

from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedMap
from ruamel.yaml.error import YAMLError
from ruamel.yaml.events import (
    AliasEvent,
    MappingEndEvent,
    MappingStartEvent,
    SequenceEndEvent,
    SequenceStartEvent,
)

# Kinds a pack folder's canonical <kind>.yaml may have; "task" documents only live inside a
# tasks pack (as includes), so they are a document kind but not a pack kind.
PACK_KINDS = ("robot", "pool", "tasks", "bridge", "equipment", "scenario")
DOCUMENT_KINDS = PACK_KINDS + ("task",)


class PackError(ValueError):
    """A pack document or scenario composition is invalid; message lists every problem found."""

    def __init__(self, problems: list[str] | str) -> None:
        self.problems = [problems] if isinstance(problems, str) else list(problems)
        super().__init__("\n".join(self.problems))


# A fresh round-trip loader/dumper per call, configured identically for loading and saving.
def _yaml() -> YAML:
    yaml = YAML(typ="rt")  # YAML 1.2 core schema: 'on'/'yes' stay strings
    yaml.preserve_quotes = True
    yaml.allow_duplicate_keys = False
    yaml.width = 4096  # never re-wrap long flow sequences
    yaml.indent(mapping=2, sequence=2, offset=0)
    return yaml


# Recursive worker for ``plain``. ``where`` is the JSON-pointer-like path used in error messages;
# ``active`` holds the ids of containers on the current recursion stack (alias cycle detection).
def _plain(value: Any, where: str, active: set[int]) -> Any:
    # Containers: copy into plain dict/list, refusing a container that contains itself
    if isinstance(value, (dict, list, tuple)):
        if id(value) in active:
            raise PackError(f"{where or '/'}: recursive alias")
        active.add(id(value))
        try:
            if isinstance(value, dict):
                result: dict[str, Any] = {}
                for key, item in value.items():
                    if not isinstance(key, str):
                        raise PackError(f"{where or '/'}: mapping key {key!r} must be a string")
                    result[str(key)] = _plain(item, f"{where}/{key}", active)
                return result
            return [_plain(item, f"{where}/{index}", active) for index, item in enumerate(value)]
        finally:
            active.discard(id(value))

    # Scalars: unwrap ruamel's scalar subclasses (ScalarFloat, ScalarBoolean, ...) to builtins.
    # bool is checked before int because bool is an int subclass.
    if isinstance(value, bool):
        return bool(value)
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise PackError(f"{where or '/'}: non-finite number {value}")
        return float(value)
    if value is None or isinstance(value, str):
        return None if value is None else str(value)
    raise PackError(f"{where or '/'}: unsupported YAML value type {type(value).__name__}")


def plain(value: Any) -> Any:
    """Detached JSON-compatible data; rejects non-string keys, recursion and non-finite floats."""
    return _plain(value, "", set())


@dataclass(frozen=True)
class Source:
    """Exact bytes of one file as read, and their digest."""

    path: Path
    data: bytes

    @property
    def text(self) -> str:
        try:
            return self.data.decode("utf-8")  # no newline translation: CRLF is kept
        except UnicodeDecodeError:
            raise PackError(f"{self.path}: not UTF-8 text") from None

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.data).hexdigest()


# Read a file's raw bytes, turning OS errors into a PackError.
def read_source(path: Path) -> Source:
    try:
        return Source(path, path.read_bytes())
    except OSError as error:
        raise PackError(f"{path}: cannot read ({error.strerror})") from None


def _recursive_alias(text: str) -> str | None:
    """Anchor referenced inside its own collection; ruamel would silently build None there."""
    # Stack of anchors of the collections currently open in the event stream
    open_anchors: list[str | None] = []
    for event in _yaml().parse(text):
        if isinstance(event, (MappingStartEvent, SequenceStartEvent)):
            open_anchors.append(event.anchor)
        elif isinstance(event, (MappingEndEvent, SequenceEndEvent)):
            open_anchors.pop()
        elif isinstance(event, AliasEvent) and event.anchor in open_anchors:
            return str(event.anchor)
    return None


# Strictly load one YAML file as a round-trip tree: rejects recursive aliases, duplicate keys,
# non-mapping roots and anything ``plain`` cannot represent (the tree itself is returned unchanged).
def read_yaml(path: Path, source: Source | None = None) -> CommentedMap:
    source = source if source is not None else read_source(path)
    try:
        recursive = _recursive_alias(source.text)
        if recursive is not None:
            raise PackError(f"{path}: recursive alias '*{recursive}'")
        data = _yaml().load(source.text)
    except YAMLError as error:  # includes duplicate keys
        raise PackError(f"{path}: invalid YAML: {error}") from None
    if not isinstance(data, CommentedMap):
        raise PackError(f"{path}: document root must be a mapping")

    # Validate only; the plain copy is discarded
    try:
        plain(data)
    except PackError as error:
        raise PackError([f"{path}:{problem}" for problem in error.problems]) from None
    return data


def canonical_file(path: Path) -> Path:
    """Return a pack file: the file itself, or the single canonical <kind>.yaml in a folder."""
    if path.is_dir():
        found = [path / f"{kind}.yaml" for kind in PACK_KINDS if (path / f"{kind}.yaml").is_file()]
        if len(found) != 1:
            names = ", ".join(f"{kind}.yaml" for kind in PACK_KINDS)
            raise PackError(f"{path}: pack folder must contain exactly one of {names}")
        return found[0]
    if not path.is_file():
        raise PackError(f"{path}: no such pack file or folder")
    return path


def _file_mode(path: Path) -> int:
    """An existing file's permission bits, else a new file's under the process umask."""
    try:
        return stat.S_IMODE(path.stat().st_mode)
    except FileNotFoundError:
        umask = os.umask(0)
        os.umask(umask)
        return 0o666 & ~umask


@dataclass
class PackDocument:
    """One loaded pack document. ``data`` is the round-trip tree; edit it, then ``save``.

    An unedited document saves its original bytes (including line endings). After edits the
    round-trip emitter keeps values, comments, key order, quoting and metadata; line endings,
    wrapping of multi-line flow collections and float spelling (same binary value) may be
    normalized. ``includes`` holds a tasks pack's validated task documents.
    """

    path: Path
    kind: str
    data: CommentedMap
    source: Source | None = field(default=None, repr=False, compare=False)
    includes: list[PackDocument] = field(default_factory=list, repr=False, compare=False)
    _baseline: str | None = field(default=None, repr=False, compare=False)

    def __post_init__(self) -> None:
        # Remember how the freshly loaded tree emits, so an unedited document can save its
        # original bytes instead of the emitter's normalization.
        if self.source is not None:
            self._baseline = self._emit()

    @property
    def root(self) -> Path:
        """Folder that bounds every pack-relative path in this document."""
        return self.path.parent

    def plain(self) -> dict[str, Any]:
        result: dict[str, Any] = plain(self.data)
        return result

    def _emit(self) -> str:
        stream = StringIO()
        _yaml().dump(self.data, stream)
        return stream.getvalue()

    # Original bytes when the tree is unchanged since loading, else freshly emitted UTF-8
    def _bytes(self) -> bytes:
        text = self._emit()
        if self.source is not None and text == self._baseline:
            return self.source.data
        return text.encode("utf-8")

    def dumps(self) -> str:
        return self._bytes().decode("utf-8")

    def save(self, path: Path | None = None) -> Path:
        """Write comments, key order, quoting and metadata unchanged; atomic replace."""
        target = Path(path) if path is not None else self.path
        data = self._bytes()

        # Write a hidden temp file in the target folder, then rename over the target. The temp file is private
        # (0600), so it takes the target's permissions first (a new file: the umask's, as open() would give).
        descriptor, temporary = tempfile.mkstemp(prefix=f".{target.name}.", dir=target.parent)
        try:
            with os.fdopen(descriptor, "wb") as stream:
                stream.write(data)
            os.chmod(temporary, _file_mode(target))
            os.replace(temporary, target)
        except BaseException:
            Path(temporary).unlink(missing_ok=True)
            raise
        return target
