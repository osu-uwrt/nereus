"""Built-in camera message formatting; headers use field maps, pixels use bulk buffers.

This module does not capture, schedule, step or publish. Formatting belongs on the camera
worker, outside the physics loop. The resulting ROS message owns its byte buffer.
"""

from __future__ import annotations

import array
import sys
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from . import mapping
from .mapping import INTEGER, TIME, MappingError, Spec

INFO_SPEC = {"width": INTEGER, "height": INTEGER, "k": Spec("float", (9,)),
             "p": Spec("float", (12,))}


def _bytes(buffer: Any) -> array.array[int]:
    result: array.array[int] = array.array("B")
    result.frombytes(memoryview(buffer).cast("B"))
    return result


@dataclass(frozen=True)
class ImageWriter:
    """Format one top-down native Frame using an already-validated stream configuration."""

    header: mapping.Writer
    output: str
    encoding: str
    jpeg_format: str | None
    jpeg_quality: int | None

    def __call__(self, frame: Any, acquired_ros_ns: int) -> Any:
        message = self.header({"sample": {"time": acquired_ros_ns}})
        if self.jpeg_format is not None:
            if not frame.jpeg:
                raise MappingError("camera did not produce the configured JPEG")
            message.format = self.jpeg_format
            message.data = _bytes(frame.jpeg)
            return message
        width, height = int(frame.width), int(frame.height)
        if width <= 0 or height <= 0:
            raise MappingError("camera frame dimensions must be positive")
        if self.output == "depth_left":
            pixels = np.asarray(frame.depth)
            if pixels.dtype != np.float32 or pixels.shape != (height, width):
                raise MappingError("depth frame must be native float32 with shape (height, width)")
            step = width * 4
        else:
            pixels = np.asarray(frame.rgb)
            if pixels.dtype != np.uint8 or pixels.shape != (height, width, 3):
                raise MappingError("RGB frame must be uint8 with shape (height, width, 3)")
            if self.encoding == "bgr8":
                pixels = pixels[:, :, ::-1]
            step = width * 3
        message.width, message.height, message.step = width, height, step
        message.encoding = self.encoding
        message.is_bigendian = int(sys.byteorder == "big")
        message.data = _bytes(np.ascontiguousarray(pixels))
        return message


def compile_image_writer(stream: Mapping[str, Any]) -> ImageWriter:
    where = f"streams/{stream['id']}"
    output = stream["native"].partition(".")[2]
    options = stream["image"]
    if output not in {"rgb_left", "rgb_right", "depth_left"}:
        raise MappingError(f"{where}: image requires an RGB or depth camera output")
    if stream["direction"] != "publish":
        raise MappingError(f"{where}: camera images must be published")
    fields = stream["fields"]
    if fields != {"header.stamp": {"from": "sample.time"}}:
        raise MappingError(f"{where}: image fields must map header.stamp from sample.time; "
                           "frame_id and image settings supply the other fields")
    quality = None
    jpeg_format = None
    if "compression" in options:
        if (output == "depth_left" or options["compression"] != "jpeg"
                or options["source_encoding"] not in {"rgb8", "bgr8"}
                or options["compressed_encoding"] != "bgr8"):
            raise MappingError(f"{where}: native JPEG uses RGB input and BGR compression")
        if stream["message_type"] != "sensor_msgs/msg/CompressedImage":
            raise MappingError(f"{where}: JPEG requires sensor_msgs/msg/CompressedImage")
        quality = options["quality"]
        if not isinstance(quality, int) or isinstance(quality, bool) or not 1 <= quality <= 100:
            raise MappingError(f"{where}: JPEG quality must be an integer in [1, 100]")
        encoding = options["source_encoding"]
        jpeg_format = f"{encoding}; jpeg compressed bgr8"
    else:
        encoding = options["encoding"]
        allowed = {"32FC1"} if output == "depth_left" else {"rgb8", "bgr8"}
        if encoding not in allowed or stream["message_type"] != "sensor_msgs/msg/Image":
            raise MappingError(f"{where}: {output} needs sensor_msgs/msg/Image with {sorted(allowed)}")
    header = mapping.compile_writer(mapping.message_class(stream["message_type"]), fields,
                                    {"sample": {"time": TIME}}, frame_id=stream["frame_id"],
                                    where=where)
    return ImageWriter(header, output, encoding, jpeg_format, quality)


def compile_info_writer(stream: Mapping[str, Any]) -> mapping.Writer:
    where = f"streams/{stream['id']}"
    output = stream["native"].partition(".")[2]
    if stream["direction"] != "publish" or output not in {"camera_info", "camera_info_right"}:
        raise MappingError(f"{where}: camera_info requires a camera metadata publication")
    if stream["message_type"] != "sensor_msgs/msg/CameraInfo" or "image" in stream:
        raise MappingError(f"{where}: camera_info requires sensor_msgs/msg/CameraInfo")
    if stream["fields"].get("header.stamp") != {"from": "sample.time"}:
        raise MappingError(f"{where}: CameraInfo header.stamp must use sample.time")
    if "header.frame_id" in stream["fields"]:
        raise MappingError(f"{where}: use frame_id for CameraInfo frame names")
    return mapping.compile_writer(mapping.message_class(stream["message_type"]), stream["fields"],
                                  {"sample": {"time": TIME}, "info": INFO_SPEC},
                                  frame_id=stream["frame_id"], where=where)
