VPStreamer & Reset Flow

Overview
- Focused description of how `VPStreamer` interacts with `avp_stream/streamer.py`.

Components
- `VPStreamer` : ROS2 node at `ros2_ws/src/teleoperation/scripts/vp_streamer.py`.
- `VisionProStreamer` : Python streamer at `avp_stream/streamer.py` (handles VisionOS comms).
- Control channel : WebRTC data channel named `control` used to send reset commands.

Goal
- Perform a safe MuJoCo reset without race conditions.

Two-phase reset protocol (implemented)
- Phase 1 — Start (pause):
  - VisionOS sends `{"type":"reset"}` to the streamer.
  - `avp_stream/streamer.py` immediately invokes registered reset callbacks with `(None, None)`.
  - `VPStreamer._on_streamer_reset(None, None)` sets `self._reset_state = "paused"`.
  - While paused, `VPStreamer._update_scene()` returns early. No joint updates or stepping.
- Phase 2 — Final (swap & reset):
  - The streamer finishes reloading the MuJoCo model/data.
  - `avp_stream/streamer.py` calls registered callbacks again with `(model, data)`.
  - `VPStreamer._on_streamer_reset(model, data)` stores `self._pending_model` / `self._pending_data` and sets `self._reset_state = "requested"`.
  - The next `VPStreamer._update_scene()` call performs the atomic swap and full reset.

Why two phases?
- Prevents applying ROS joint/state updates while the streamer is reloading the MuJoCo scene.
- Avoids model/data lifetime races and segfaults caused by mid-reload access to MuJoCo structures.

Key implementation notes (in `vp_streamer.py`)
- Locking: `self._reset_lock` serializes notifications and the sim-thread swap.
- States: `idle`, `paused`, `requested`, `handling` used to coordinate steps.
- Pending swap: `self._pending_model` and `self._pending_data` hold the new objects until the sim thread swaps them.
- Skip frames: After a reset we set `self._skip_joint_apply_frames = 30` to let the scene settle.
- Joint buffer: We clear `self._latest_joint_state` while holding `self._joint_state_lock` to avoid applying stale commands.

Expected log sequence (short lines)
- From streamer: `[CONTROL] Reset command received from VisionOS`
- From VPStreamer (start): `MuJoCo reset start received; pausing simulation updates.`
- From streamer: `MuJoCo simulation reloaded from /path/to/scene.xml`
- From VPStreamer (final): `MuJoCo reset requested.`
- From VPStreamer (sim thread): `Performing MuJoCo reset in sim thread.`
- From VPStreamer: `Cleared joint state buffer on reset.`

Visual flow (ASCII)

VisionOS
   |
   |  reset JSON
   v
avp_stream/streamer.py
   |  -- early notify --> `callback(None, None)`
   v                      (VPStreamer sets `paused`)
(reload model/data in background)
   |
   |  -- final notify --> `callback(model, data)`
   v                      (VPStreamer sets `requested`)
VPStreamer sim timer
   |  acquires `_reset_lock` and performs swap/reset

How to verify locally
- Start `vp_streamer` (normal launch or `ros2 run teleoperation vp_streamer.py`).
- Trigger reset from VisionOS or simulate the control message in `avp_stream` logs.
- Watch for the log sequence in the previous section.

Troubleshooting tips
- If you see `Model/Data mismatch after reload` in logs:
  - Confirm the file `xml_path` is the same used when configuring the streamer.
  - Check the sizes: `len(qpos)` vs `model.nq` and `len(qvel)` vs `model.nv`.
- If segfaults persist:
  - Ensure `mujoco.MjModel`/`MjData` remain valid across threads.
  - Add temporary logging around the swap to print `id(self.model)` / `id(model)` before/after.
- If control responses fail to send, ensure WebRTC loop is running and `_control_channel.readyState == "open"`.

Quick file references
- `VPStreamer` implementation: `ros2_ws/src/teleoperation/scripts/vp_streamer.py`
- `VisionProStreamer` implementation: `avp_stream/streamer.py`

Want a test script?
- I can add a small script that simulates streamer control messages to demonstrate the pause/reload/finalize sequence and capture logs.
