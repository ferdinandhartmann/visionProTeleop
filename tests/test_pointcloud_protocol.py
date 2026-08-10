from types import SimpleNamespace
import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np

_MODULE_PATH = Path(__file__).parents[1] / "avp_stream" / "pointcloud_protocol.py"
_SPEC = importlib.util.spec_from_file_location("pointcloud_protocol", _MODULE_PATH)
pointcloud_protocol = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = pointcloud_protocol
_SPEC.loader.exec_module(pointcloud_protocol)

from pointcloud_protocol import (  # noqa: E402
    POINTCLOUD_FLAGS,
    POINTCLOUD_CHUNK_PAYLOAD_BYTES,
    POINTCLOUD_HEADER,
    POINTCLOUD_MAGIC,
    POINTCLOUD_RECORD_BYTES,
    POINTCLOUD_VERSION,
    encode_point_cloud_frame,
    extract_xyz_rgb,
    transform_xyz,
    unpack_chunk_header,
)


def _field(name, offset, datatype):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=1)


def _cloud(rows, *, rgb_datatype=7, row_padding=0):
    point_step = 16
    width = len(rows)
    row_step = width * point_step + row_padding
    data = bytearray(row_step)
    for index, (x, y, z, rgb) in enumerate(rows):
        offset = index * point_step
        data[offset : offset + 12] = np.asarray([x, y, z], dtype="<f4").tobytes()
        if rgb_datatype == 7:
            data[offset + 12 : offset + 16] = np.asarray(
                [rgb], dtype="<u4"
            ).view("<f4").tobytes()
        else:
            data[offset + 12 : offset + 16] = np.asarray([rgb], dtype="<u4").tobytes()
    return SimpleNamespace(
        fields=[
            _field("x", 0, 7),
            _field("y", 4, 7),
            _field("z", 8, 7),
            _field("rgb", 12, rgb_datatype),
        ],
        height=1,
        width=width,
        point_step=point_step,
        row_step=row_step,
        is_bigendian=False,
        data=data,
    )


class PointCloudProtocolTests(unittest.TestCase):
    def test_extract_xyz_and_exact_rgb(self):
        for rgb_datatype in (6, 7):
            with self.subTest(rgb_datatype=rgb_datatype):
                cloud = _cloud(
                    [
                        (1.0, 2.0, 3.0, 0x00FF0000),
                        (4.0, 5.0, 6.0, 0x0000FF00),
                        (7.0, 8.0, 9.0, 0x000000FF),
                    ],
                    rgb_datatype=rgb_datatype,
                    row_padding=12,
                )

                xyz, rgb = extract_xyz_rgb(cloud)

                np.testing.assert_array_equal(
                    xyz,
                    np.asarray(
                        [[1, 2, 3], [4, 5, 6], [7, 8, 9]],
                        dtype=np.float32,
                    ),
                )
                np.testing.assert_array_equal(
                    rgb,
                    np.asarray(
                        [[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                        dtype=np.uint8,
                    ),
                )

    def test_extract_filters_nan_with_matching_color(self):
        cloud = _cloud(
            [
                (1.0, 2.0, 3.0, 0x00112233),
                (np.nan, 5.0, 6.0, 0x00AABBCC),
            ]
        )

        xyz, rgb = extract_xyz_rgb(cloud)

        np.testing.assert_array_equal(xyz, [[1.0, 2.0, 3.0]])
        np.testing.assert_array_equal(rgb, [[0x11, 0x22, 0x33]])

    def test_transform_xyz_normalizes_quaternion(self):
        transform = SimpleNamespace(
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=2.0, w=2.0),
            )
        )

        result = transform_xyz(
            np.asarray([[1.0, 0.0, 0.0]], dtype=np.float32),
            transform,
        )

        np.testing.assert_allclose(result, [[1.0, 3.0, 3.0]], atol=1e-6)

    def test_encoding_is_interleaved_chunked_and_versioned(self):
        points_per_chunk = (
            POINTCLOUD_CHUNK_PAYLOAD_BYTES // POINTCLOUD_RECORD_BYTES
        )
        point_count = points_per_chunk + 1
        positions = np.zeros((point_count, 3), dtype=np.float32)
        colors = np.zeros((point_count, 3), dtype=np.uint8)
        positions[0] = [1.0, 2.0, 3.0]
        positions[-1] = [-1.0, -2.0, -3.0]
        colors[0] = [255, 0, 0]
        colors[-1] = [1, 2, 3]

        frame = encode_point_cloud_frame(
            positions,
            colors,
            sequence=42,
            timestamp_ns=1234,
        )

        self.assertEqual(frame.sequence, 42)
        self.assertEqual(frame.point_count, point_count)
        self.assertEqual(
            frame.payload_bytes,
            point_count * POINTCLOUD_RECORD_BYTES,
        )
        self.assertEqual(len(frame.chunks), 2)

        first = unpack_chunk_header(frame.chunks[0])
        self.assertEqual(
            first[:11],
            (
                POINTCLOUD_MAGIC,
                POINTCLOUD_VERSION,
                POINTCLOUD_FLAGS,
                POINTCLOUD_HEADER.size,
                42,
                1234,
                point_count,
                0,
                2,
                0,
                points_per_chunk,
            ),
        )
        first_payload = frame.chunks[0][POINTCLOUD_HEADER.size :]
        np.testing.assert_array_equal(
            np.frombuffer(first_payload[:6], dtype="<f2").astype(np.float32),
            positions[0],
        )
        self.assertEqual(first_payload[6:9], bytes([255, 0, 0]))

    def test_encoder_rejects_invalid_shapes_and_nonfinite_positions(self):
        with self.assertRaises(ValueError):
            encode_point_cloud_frame(
                np.zeros((2, 2), dtype=np.float32),
                np.zeros((2, 3), dtype=np.uint8),
                sequence=0,
            )
        with self.assertRaises(ValueError):
            encode_point_cloud_frame(
                np.asarray([[np.nan, 0, 0]], dtype=np.float32),
                np.zeros((1, 3), dtype=np.uint8),
                sequence=0,
            )
        with self.assertRaises(ValueError):
            encode_point_cloud_frame(
                np.zeros((100_001, 3), dtype=np.float32),
                np.zeros((100_001, 3), dtype=np.uint8),
                sequence=0,
            )

    def test_100k_frame_fits_expected_chunk_budget(self):
        positions = np.zeros((100_000, 3), dtype=np.float32)
        colors = np.zeros((100_000, 3), dtype=np.uint8)

        frame = encode_point_cloud_frame(
            positions,
            colors,
            sequence=0,
        )

        self.assertEqual(frame.payload_bytes, 900_000)
        self.assertEqual(len(frame.chunks), 19)
        self.assertTrue(all(len(chunk) <= 48 * 1024
                            for chunk in frame.chunks))


if __name__ == "__main__":
    unittest.main()
