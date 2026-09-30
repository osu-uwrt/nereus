"""Generic data-driven ROS 2 bridge for nereus scenarios (no robot-specific code)."""

from .core import BridgeCore, BridgeError
from .mapping import MappingError, compile_reader, compile_writer

__all__ = ["BridgeCore", "BridgeError", "MappingError", "compile_reader", "compile_writer"]
