from __future__ import annotations

import asyncio
import threading
import unittest

import numpy as np

from avp_stream.pointcloud_protocol import encode_point_cloud_frame
from avp_stream.streamer import VisionProStreamer


class _DrainingChannel:
    readyState = "open"

    def __init__(self, streamer):
        self.streamer = streamer
        self.buffer_checks = 0
        self.sent = []

    @property
    def bufferedAmount(self):
        self.buffer_checks += 1
        return 256 * 1024 if self.buffer_checks == 1 else 0

    def send(self, chunk):
        self.sent.append(chunk)
        self.streamer._pointcloud_running = False


class PointCloudSenderTests(unittest.TestCase):
    def test_busy_channel_retries_latest_frame_after_drain(self):
        streamer = object.__new__(VisionProStreamer)
        frame = encode_point_cloud_frame(
            np.asarray([[1.0, 2.0, 3.0]], dtype=np.float32),
            np.asarray([[10, 20, 30]], dtype=np.uint8),
            sequence=7,
        )
        channel = _DrainingChannel(streamer)
        streamer._pointcloud_running = True
        streamer._pointcloud_sender_generation = 3
        streamer._webrtc_point_ready = True
        streamer._webrtc_point_channel = channel
        streamer._pointcloud_lock = threading.Lock()
        streamer._pointcloud_frame = frame
        streamer._pointcloud_dirty = True
        streamer._pointcloud_wakeup = None
        streamer._pointcloud_sent_frames = 0
        streamer._pointcloud_backpressure_drops = 0
        streamer._log = lambda *args, **kwargs: None
        streamer.verbose = False

        asyncio.run(streamer._pointcloud_send_loop(3))

        self.assertEqual(streamer._pointcloud_backpressure_drops, 1)
        self.assertEqual(streamer._pointcloud_sent_frames, 1)
        self.assertFalse(streamer._pointcloud_dirty)
        self.assertEqual(channel.sent, list(frame.chunks))


if __name__ == "__main__":
    unittest.main()
