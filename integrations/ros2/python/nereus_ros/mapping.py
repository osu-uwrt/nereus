"""Declarative field maps between native values and ROS messages, checked before stepping.

A map is ``{destination path: {from: path} | {constant: value} | {from: path, enum_map: {...}}}``.
Paths are dotted names with integer indexes; there is no expression language. Every
destination is resolved against the ROS type description and every source against a typed
native value specification, so unknown fields and incompatible shapes fail at startup.
"""

from __future__ import annotations

import re
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

_PATH = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(\[[0-9]+\])*(\.[A-Za-z_][A-Za-z0-9_]*(\[[0-9]+\])*)*$")
_TOKEN = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)((?:\[[0-9]+\])*)")
_FLOAT = {"float", "double", "float32", "float64"}
_INT_RANGE = {
    "int8": (-(2**7), 2**7 - 1), "uint8": (0, 2**8 - 1), "byte": (0, 2**8 - 1),
    "octet": (0, 2**8 - 1), "char": (0, 2**8 - 1),
    "int16": (-(2**15), 2**15 - 1), "uint16": (0, 2**16 - 1),
    "int32": (-(2**31), 2**31 - 1), "uint32": (0, 2**32 - 1),
    "int64": (-(2**63), 2**63 - 1), "uint64": (0, 2**64 - 1),
}
_VECTOR_MESSAGES = {"geometry_msgs/Vector3", "geometry_msgs/Point"}
_QUATERNION = "geometry_msgs/Quaternion"
_TIME = "builtin_interfaces/Time"


class MappingError(ValueError):
    """A declared field map cannot be applied to the declared ROS or native types."""


# ------------------------------------------------------------------ native value specs


@dataclass(frozen=True)
class Spec:
    """Typed native value: dtype in float/int/bool/string/time; shape () or dims (None=any)."""

    dtype: str
    shape: tuple[int | None, ...] = ()

    def describe(self) -> str:
        return self.dtype + ("" if not self.shape else str(list(self.shape)))


SCALAR, INTEGER, BOOLEAN, STRING, TIME = (
    Spec("float"), Spec("int"), Spec("bool"), Spec("string"), Spec("time"))
VECTOR3, QUATERNION_WXYZ, MATRIX3 = Spec("float", (3,)), Spec("float", (4,)), Spec("float", (3, 3))
SpecTree = Mapping[str, Any]  # nested mapping whose leaves are Spec


def parse_path(path: str) -> list[tuple[str, tuple[int, ...]]]:
    if not _PATH.match(path):
        raise MappingError(f"invalid field path {path!r}")
    return [
        (match.group(1), tuple(int(index) for index in re.findall(r"\[([0-9]+)\]", match.group(2))))
        for match in _TOKEN.finditer(path)
    ]


def source_spec(tree: SpecTree, path: str) -> Spec:
    node: Any = tree
    for name, indexes in parse_path(path):
        if not isinstance(node, Mapping) or name not in node:
            raise MappingError(f"unknown native field {path!r}")
        node = node[name]
        for index in indexes:
            if not isinstance(node, Spec) or not node.shape:
                raise MappingError(f"native field {path!r} is not indexable")
            size = node.shape[0]
            if size is not None and index >= size:
                raise MappingError(f"index {index} out of range in native field {path!r}")
            node = Spec(node.dtype, node.shape[1:])
    if not isinstance(node, Spec):
        raise MappingError(f"native field {path!r} is a structure, not a value")
    return node


def read_source(values: Any, path: str) -> Any:
    node = values
    for name, indexes in parse_path(path):
        node = node[name] if isinstance(node, Mapping) else getattr(node, name)
        for index in indexes:
            node = node[index]
    return node


# ------------------------------------------------------------------ ROS type descriptions


@dataclass(frozen=True)
class RosType:
    base: str  # primitive name or 'pkg/Message'
    length: int | None = None  # fixed array length
    sequence: bool = False  # unbounded or bounded sequence

    @property
    def is_array(self) -> bool:
        return self.length is not None or self.sequence

    @property
    def element(self) -> RosType:
        return RosType(self.base)

    @property
    def is_message(self) -> bool:
        return "/" in self.base


def parse_type(text: str) -> RosType:
    match = re.fullmatch(r"sequence<([^,>]+)(?:,\s*[0-9]+)?>", text)
    if match:
        return RosType(_base(match.group(1)), sequence=True)
    match = re.fullmatch(r"(.+)\[([0-9]+)\]", text)
    if match:
        return RosType(_base(match.group(1)), length=int(match.group(2)))
    match = re.fullmatch(r"(.+)\[<=([0-9]+)\]", text)
    if match:
        return RosType(_base(match.group(1)), sequence=True)
    return RosType(_base(text))


def _base(text: str) -> str:
    return re.sub(r"<=[0-9]+$", "", text.strip())  # bounded strings are strings


def message_class(type_name: str) -> Any:
    from rosidl_runtime_py.utilities import get_message

    parts = type_name.split("/")
    name = type_name if len(parts) == 3 else f"{parts[0]}/msg/{parts[-1]}"
    try:
        return get_message(name)
    except (AttributeError, ModuleNotFoundError, ValueError) as error:
        raise MappingError(f"ROS message type {type_name!r} is not installed ({error})") from None


def service_class(type_name: str) -> Any:
    from rosidl_runtime_py.utilities import get_service

    try:
        return get_service(type_name)
    except (AttributeError, ModuleNotFoundError, ValueError) as error:
        raise MappingError(f"ROS service type {type_name!r} is not installed ({error})") from None


def ros_field(cls: Any, path: str, *, writable: bool = False) -> RosType:
    """ROS type of a message field path, applying integer indexes to array fields.

    ``writable`` rejects indexes into sequences: a default-constructed message has an empty
    sequence, so only fixed-size arrays have addressable elements to assign.
    """
    current: RosType | None = None
    owner = cls
    for position, (name, indexes) in enumerate(parse_path(path)):
        if current is not None:
            if not current.is_message or current.is_array:
                raise MappingError(f"{path!r}: {name!r} is below a non-message field")
            owner = message_class(current.base)
        fields = owner.get_fields_and_field_types()
        if name not in fields:
            raise MappingError(f"{path!r}: {owner.__name__} has no field {name!r}")
        current = parse_type(fields[name])
        for index in indexes:
            if not current.is_array:
                raise MappingError(f"{path!r}: {name!r} is not an array")
            if writable and current.sequence:
                raise MappingError(f"{path!r}: cannot assign an element of sequence {name!r}; "
                                   "assign the whole sequence")
            if current.length is not None and index >= current.length:
                raise MappingError(f"{path!r}: index {index} exceeds {name}[{current.length}]")
            current = current.element
    assert current is not None
    return current


# ------------------------------------------------------------------ conversions


def _scalar_converter(ros: RosType, spec: Spec, where: str) -> Callable[[Any], Any]:
    base = ros.base
    if base in _FLOAT and spec.dtype in ("float", "int"):
        return float
    if base in _INT_RANGE and spec.dtype == "int":
        low, high = _INT_RANGE[base]

        def integer(value: Any) -> int:
            result = int(value)
            if not low <= result <= high:
                raise MappingError(f"{where}: {result} out of range for {base}")
            return result
        return integer
    if base == "boolean" and spec.dtype == "bool":
        return bool
    if base in ("string", "wstring") and spec.dtype == "string":
        return str
    raise MappingError(f"{where}: cannot assign native {spec.describe()} to ROS {base}")


def converter(ros: RosType, spec: Spec, where: str) -> Callable[[Any], Any]:
    """Conversion from a native value of ``spec`` to the ROS field type, or MappingError."""
    if ros.base == _QUATERNION and not ros.is_array:
        raise MappingError(f"{where}: quaternions are never assigned whole; map w/x/y/z "
                           "components explicitly")
    if ros.base == _TIME and not ros.is_array:
        if spec != TIME:
            raise MappingError(f"{where}: {_TIME} needs a native time source")
        time_class = message_class(_TIME)

        def stamp(value: Any) -> Any:
            nanoseconds = int(value)
            return time_class(sec=nanoseconds // 1_000_000_000,
                              nanosec=nanoseconds % 1_000_000_000)
        return stamp
    if ros.base in _VECTOR_MESSAGES and not ros.is_array:
        if spec != VECTOR3:
            raise MappingError(f"{where}: {ros.base} needs a native float[3] vector")
        vector_class = message_class(ros.base)

        def vector(value: Any) -> Any:
            x, y, z = (float(item) for item in np.asarray(value, float).reshape(3))
            return vector_class(x=x, y=y, z=z)
        return vector
    if ros.is_message and not ros.is_array:
        raise MappingError(f"{where}: whole {ros.base} assignment is not supported; map its fields")
    if ros.is_array:
        if ros.is_message or not spec.shape:
            raise MappingError(f"{where}: cannot assign native {spec.describe()} to ROS "
                               f"{ros.base} array")
        size = 1
        for dimension in spec.shape:
            size = -1 if dimension is None or size < 0 else size * dimension
        if ros.length is not None and size != ros.length:
            raise MappingError(f"{where}: native {spec.describe()} (row-major) does not fill "
                               f"{ros.base}[{ros.length}]")
        element = _scalar_converter(ros.element, Spec(spec.dtype), where)

        def array(value: Any) -> list[Any]:
            flat = np.asarray(value).reshape(-1)
            if ros.length is not None and flat.size != ros.length:
                raise MappingError(f"{where}: runtime size {flat.size} != {ros.length}")
            return [element(item) for item in flat]
        return array
    if spec.shape:
        raise MappingError(f"{where}: cannot assign native {spec.describe()} to scalar {ros.base}")
    return _scalar_converter(ros, spec, where)


def constant_spec(value: Any, where: str) -> Spec:
    if isinstance(value, bool):
        return BOOLEAN
    if isinstance(value, int):
        return INTEGER
    if isinstance(value, float):
        return SCALAR
    if isinstance(value, str):
        return STRING
    if isinstance(value, list) and value:
        inner = {constant_spec(item, where) for item in value}
        if any(item.shape for item in inner):
            raise MappingError(f"{where}: nested constant arrays are not supported")
        dtypes = {item.dtype for item in inner}
        if dtypes == {"float", "int"}:
            dtypes = {"float"}  # YAML-natural numeric lists such as [1, 0, 0.5]
        if len(dtypes) != 1:
            raise MappingError(f"{where}: mixed constant array")
        return Spec(dtypes.pop(), (len(value),))
    raise MappingError(f"{where}: unsupported constant {value!r}")


def _assign(message: Any, path: str, value: Any) -> None:
    tokens = parse_path(path)
    node = message
    for name, indexes in tokens[:-1]:
        node = getattr(node, name)
        for index in indexes:
            node = node[index]
    name, indexes = tokens[-1]
    if not indexes:
        setattr(node, name, value)
        return
    container = getattr(node, name)
    for index in indexes[:-1]:
        container = container[index]
    container[indexes[-1]] = value


# ------------------------------------------------------------------ compiled maps


@dataclass(frozen=True)
class Writer:
    """Builds one ROS message from a native value tree (publish direction)."""

    cls: Any
    steps: tuple[tuple[str, Callable[[Any], Any]], ...]

    def __call__(self, values: Any) -> Any:
        message = self.cls()
        for path, produce in self.steps:
            _assign(message, path, produce(values))
        return message


def compile_writer(cls: Any, fields: Mapping[str, Mapping[str, Any]], sources: SpecTree,
                   *, frame_id: str | None = None, where: str = "") -> Writer:
    """Validate a publish/response map against ``cls`` and native ``sources``."""
    steps: list[tuple[str, Callable[[Any], Any]]] = []
    stamped = "header" in cls.get_fields_and_field_types()
    if frame_id is not None:
        if frame_id and not stamped:
            raise MappingError(f"{where}: frame_id {frame_id!r} set on unstamped {cls.__name__}")
        if stamped and not frame_id:
            raise MappingError(f"{where}: stamped {cls.__name__} requires a frame_id")
        if stamped:
            if "header.stamp" not in fields:
                raise MappingError(f"{where}: stamped {cls.__name__} must map header.stamp")
            steps.append(("header.frame_id", lambda _values, frame=frame_id: frame))
    for destination, source in fields.items():
        at = f"{where}/{destination}"
        ros = ros_field(cls, destination, writable=True)
        if "constant" in source:
            constant = source["constant"]
            convert = converter(ros, constant_spec(constant, at), at)
            fixed = convert(constant)
            steps.append((destination, lambda _values, fixed=fixed: fixed))
            continue
        path = source["from"]
        spec = source_spec(sources, path)
        if "enum_map" in source:
            table = dict(source["enum_map"])
            if spec != STRING:
                raise MappingError(f"{at}: enum_map needs a native string state, got "
                                   f"{spec.describe()}")
            convert = converter(ros, INTEGER, at)
            for value in table.values():
                convert(value)

            def lookup(values: Any, path: str = path, table: dict[str, int] = table,
                       convert: Callable[[Any], Any] = convert, at: str = at) -> Any:
                state = str(read_source(values, path))
                if state not in table:
                    raise MappingError(f"{at}: native state {state!r} has no enum_map entry")
                return convert(table[state])
            steps.append((destination, lookup))
            continue
        convert = converter(ros, spec, at)
        steps.append((destination, lambda values, path=path, convert=convert:
                      convert(read_source(values, path))))
    return Writer(cls, tuple(steps))


def ros_spec(ros: RosType, where: str) -> Spec:
    """Native spec of a ROS field read by a subscribe/request map."""
    if ros.base == _QUATERNION and not ros.is_array:
        raise MappingError(f"{where}: read quaternion w/x/y/z components explicitly")
    if ros.base in _VECTOR_MESSAGES and not ros.is_array:
        return VECTOR3
    if ros.is_message:
        raise MappingError(f"{where}: cannot read whole {ros.base}")
    if ros.base in _FLOAT:
        dtype = "float"
    elif ros.base in _INT_RANGE:
        dtype = "int"
    elif ros.base == "boolean":
        dtype = "bool"
    elif ros.base in ("string", "wstring"):
        dtype = "string"
    else:
        raise MappingError(f"{where}: unsupported ROS type {ros.base}")
    if ros.is_array:
        return Spec(dtype, (ros.length,))
    return Spec(dtype)


def _compatible(expected: Spec, actual: Spec) -> bool:
    if expected.dtype == "float" and actual.dtype not in ("float", "int"):
        return False
    if expected.dtype != "float" and expected.dtype != actual.dtype:
        return False
    if len(expected.shape) != len(actual.shape):
        return False
    return all(a is None or b is None or a == b for a, b in zip(expected.shape, actual.shape))


def _read(message: Any, path: str, spec: Spec) -> Any:
    try:
        value = read_source(message, path)
    except IndexError:
        raise MappingError(f"{path!r}: message sequence is shorter than the mapped index") from None
    if spec == VECTOR3 and hasattr(value, "x"):
        return np.array([value.x, value.y, value.z], float)
    if spec.shape:
        array = np.asarray(value, float if spec.dtype == "float" else object)
        if array.ndim != len(spec.shape) or any(
                expected is not None and expected != actual
                for expected, actual in zip(spec.shape, array.shape)):
            raise MappingError(f"{path!r}: received shape {list(array.shape)} does not match "
                               f"native {spec.describe()}")
        return array
    return {"float": float, "int": int, "bool": bool, "string": str}[spec.dtype](value)


@dataclass(frozen=True)
class Reader:
    """Extracts native arguments from a ROS message (subscribe/request direction)."""

    steps: tuple[tuple[str, str, Spec], ...]
    filters: tuple[tuple[str, Any], ...]

    def accepts(self, message: Any) -> bool:
        try:
            return all(read_source(message, path) == value for path, value in self.filters)
        except IndexError:
            raise MappingError("accept_if field index beyond the received sequence") from None

    def __call__(self, message: Any) -> dict[str, Any]:
        return {argument: _read(message, path, spec) for argument, path, spec in self.steps}


def compile_reader(cls: Any, fields: Mapping[str, Mapping[str, Any]],
                   arguments: Mapping[str, Spec], accept_if: list[Mapping[str, Any]] | None = None,
                   *, where: str = "") -> Reader:
    """Validate a subscribe/request map: every native argument mapped exactly, types agree."""
    unknown = set(fields) - set(arguments)
    missing = set(arguments) - set(fields)
    if unknown or missing:
        raise MappingError(f"{where}: native arguments must be exactly {sorted(arguments)} "
                           f"(unknown {sorted(unknown)}, missing {sorted(missing)})")
    steps = []
    for argument, source in fields.items():
        at = f"{where}/{argument}"
        if "from" not in source or len(source) != 1:
            raise MappingError(f"{at}: inbound maps accept only {{from: path}}")
        actual = ros_spec(ros_field(cls, source["from"]), at)
        if not _compatible(arguments[argument], actual):
            raise MappingError(f"{at}: ROS {actual.describe()} does not provide native "
                               f"{arguments[argument].describe()}")
        steps.append((argument, source["from"], arguments[argument]))
    filters = []
    for condition in accept_if or []:
        at = f"{where}/accept_if/{condition['field']}"
        actual = ros_spec(ros_field(cls, condition["field"]), at)
        if actual.shape or not _compatible(actual, constant_spec(condition["equals"], at)):
            raise MappingError(f"{at}: cannot compare ROS {actual.describe()} with "
                               f"{condition['equals']!r}")
        filters.append((condition["field"], condition["equals"]))
    return Reader(tuple(steps), tuple(filters))
