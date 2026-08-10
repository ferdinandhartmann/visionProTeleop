"""Compact, low-latency point-cloud wire protocol shared with visionOS."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Tuple

import numpy as np


POINTCLOUD_MAGIC = b"PCD2"
POINTCLOUD_VERSION = 2
POINTCLOUD_FLAGS = 0x03  # bit 0: float16 XYZ, bit 1: uint8 RGB
POINTCLOUD_RECORD_BYTES = 9
POINTCLOUD_MAX_POINTS = 100_000
POINTCLOUD_HEADER = struct.Struct("<4sBBHIQIHHII6f")
POINTCLOUD_CHUNK_PAYLOAD_BYTES = 48 * 1024 - POINTCLOUD_HEADER.size

_POINT_DTYPE = np.dtype(
    [
        ("position", "<f2", (3,)),
        ("color", "u1", (3,)),
    ],
    align=False,
)

_POINT_FIELD_DTYPES = {
    2: "u1",   # PointField.UINT8
    4: "u2",   # PointField.UINT16
    6: "u4",   # PointField.UINT32
    7: "f4",   # PointField.FLOAT32
    8: "f8",   # PointField.FLOAT64
}


@dataclass(frozen=True)
class EncodedPointCloudFrame:
    """One complete point-cloud frame represented as WebRTC messages."""

    sequence: int
    timestamp_ns: int
    point_count: int
    payload_bytes: int
    chunks: Tuple[bytes, ...]


def extract_xyz_rgb(pointcloud) -> Tuple[np.ndarray, np.ndarray]:
    """Vectorize XYZ and packed RealSense RGB from a PointCloud2-like object."""
    fields = {field.name: field for field in pointcloud.fields}
    rgb_name = "rgb" if "rgb" in fields else "rgba" if "rgba" in fields else None
    required = ("x", "y", "z")
    if rgb_name is None or any(name not in fields for name in required):
        raise ValueError("point cloud must contain x, y, z and rgb/rgba fields")

    height = max(1, int(pointcloud.height))
    width = int(pointcloud.width)
    point_step = int(pointcloud.point_step)
    row_step = int(pointcloud.row_step)
    if width <= 0 or point_step <= 0 or row_step < width * point_step:
        raise ValueError("invalid PointCloud2 dimensions or strides")
    required_bytes = (height - 1) * row_step + width * point_step
    if len(pointcloud.data) < required_bytes:
        raise ValueError("PointCloud2 data is shorter than its declared layout")

    endian = ">" if pointcloud.is_bigendian else "<"

    def field_view(name: str) -> np.ndarray:
        field = fields[name]
        dtype_code = _POINT_FIELD_DTYPES.get(int(field.datatype))
        if dtype_code is None or int(field.count) != 1:
            raise ValueError(f"unsupported field layout for '{name}'")
        dtype = np.dtype(endian + dtype_code)
        if int(field.offset) + dtype.itemsize > point_step:
            raise ValueError(f"field '{name}' exceeds point_step")
        view = np.ndarray(
            shape=(height, width),
            dtype=dtype,
            buffer=pointcloud.data,
            offset=int(field.offset),
            strides=(row_step, point_step),
        )
        return np.asarray(view).reshape(-1)

    xyz_fields = []
    for name in required:
        field = fields[name]
        if int(field.datatype) not in (7, 8):
            raise ValueError(f"field '{name}' must be floating point")
        xyz_fields.append(field_view(name).astype(np.float32, copy=False))
    xyz = np.column_stack(xyz_fields)

    rgb_field = fields[rgb_name]
    rgb_values = field_view(rgb_name)
    if int(rgb_field.datatype) == 7:
        packed_rgb = (
            np.ascontiguousarray(rgb_values)
            .view(np.dtype(endian + "u4"))
            .astype(np.uint32, copy=False)
        )
    elif int(rgb_field.datatype) == 6:
        packed_rgb = rgb_values.astype(np.uint32, copy=False)
    else:
        raise ValueError("rgb/rgba must use FLOAT32 or UINT32 packing")

    colors = np.empty((packed_rgb.size, 3), dtype=np.uint8)
    colors[:, 0] = (packed_rgb >> 16) & 0xFF
    colors[:, 1] = (packed_rgb >> 8) & 0xFF
    colors[:, 2] = packed_rgb & 0xFF

    valid = np.isfinite(xyz).all(axis=1)
    if valid.all():
        return xyz, colors
    return xyz[valid], colors[valid]


def transform_xyz(positions: np.ndarray, transform) -> np.ndarray:
    """Apply a TransformStamped-like rigid transform to an XYZ array."""
    xyz = np.asarray(positions, dtype=np.float32)
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    q = np.asarray([rotation.x, rotation.y, rotation.z, rotation.w], dtype=np.float64)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid transform quaternion")
    x, y, z, w = q / norm
    matrix = np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )
    offset = np.asarray(
        [translation.x, translation.y, translation.z],
        dtype=np.float32,
    )
    return np.ascontiguousarray(xyz @ matrix.T + offset)


def encode_point_cloud_frame(
    positions: np.ndarray,
    colors: np.ndarray,
    *,
    sequence: int,
    timestamp_ns: int = 0,
) -> EncodedPointCloudFrame:
    """Encode float XYZ and byte RGB arrays into point-aligned PCD2 chunks."""
    xyz = np.asarray(positions, dtype=np.float32)
    rgb = np.asarray(colors, dtype=np.uint8)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("positions must have shape (N, 3)")
    if rgb.shape != xyz.shape:
        raise ValueError("colors must have the same (N, 3) shape as positions")
    if xyz.shape[0] == 0:
        raise ValueError("point cloud must not be empty")
    if xyz.shape[0] > POINTCLOUD_MAX_POINTS:
        raise ValueError(
            f"point cloud exceeds the {POINTCLOUD_MAX_POINTS}-point limit"
        )
    if not np.isfinite(xyz).all():
        raise ValueError("positions must be finite")
    bounds_min = xyz.min(axis=0)
    bounds_max = xyz.max(axis=0)
    half_limit = np.finfo(np.float16).max
    if (bounds_min < -half_limit).any() or (bounds_max > half_limit).any():
        raise ValueError("positions exceed the float16 wire-format range")
    point_count = int(xyz.shape[0])
    packed = np.empty(point_count, dtype=_POINT_DTYPE)
    packed["position"] = xyz
    packed["color"] = rgb
    packed_bytes = memoryview(packed).cast("B")

    bounds = tuple(float(value) for value in np.concatenate((bounds_min, bounds_max)))

    points_per_chunk = (
        POINTCLOUD_CHUNK_PAYLOAD_BYTES // POINTCLOUD_RECORD_BYTES
    )
    chunk_count = math.ceil(point_count / points_per_chunk)
    if chunk_count > 0xFFFF:
        raise ValueError("point cloud requires too many chunks")

    chunks = []
    for chunk_index in range(chunk_count):
        first_point = chunk_index * points_per_chunk
        chunk_points = min(points_per_chunk, point_count - first_point)
        payload_start = first_point * POINTCLOUD_RECORD_BYTES
        payload_end = payload_start + chunk_points * POINTCLOUD_RECORD_BYTES
        header = POINTCLOUD_HEADER.pack(
            POINTCLOUD_MAGIC,
            POINTCLOUD_VERSION,
            POINTCLOUD_FLAGS,
            POINTCLOUD_HEADER.size,
            sequence & 0xFFFFFFFF,
            timestamp_ns & 0xFFFFFFFFFFFFFFFF,
            point_count,
            chunk_index,
            chunk_count,
            first_point,
            chunk_points,
            *bounds,
        )
        chunks.append(
            b"".join((header, packed_bytes[payload_start:payload_end]))
        )

    return EncodedPointCloudFrame(
        sequence=sequence & 0xFFFFFFFF,
        timestamp_ns=timestamp_ns & 0xFFFFFFFFFFFFFFFF,
        point_count=point_count,
        payload_bytes=packed.nbytes,
        chunks=tuple(chunks),
    )


def unpack_chunk_header(chunk: bytes):
    """Return a decoded header tuple for tests and diagnostics."""
    if len(chunk) < POINTCLOUD_HEADER.size:
        raise ValueError("point-cloud chunk is shorter than its header")
    return POINTCLOUD_HEADER.unpack_from(chunk)
