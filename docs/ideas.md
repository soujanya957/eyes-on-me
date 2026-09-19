# Ideas

What else head tracking + Spot can do.

Everything below builds on the same `HeadSample` → `GazeMapper` → command
pipeline; most are a new mode in `spot_gaze.py` plus a mapper function.

**Perception**
- **Spot CAM PTZ** — same idea as `--camera auto` / `--mode arm`, but driving
  the Spot CAM's pan/tilt 1:1 with your head (`bosdyn.client.spot_cam.ptz`).
- **Look-then-walk** — in `turn` mode, hold your gaze for ~1.5 s and Spot
  walks a step toward what you're looking at (`synchro_trajectory_command_in_body_frame`).
- **Attention logging** — record where you looked and what Spot's camera saw
  there; a cheap way to collect "human-salient" frames for a dataset.

**Teleop / control**
- **Head as joystick** — pitch forward/back → v_x, roll → v_y, yaw → v_rot.
  Hands-free driving; a WASD-free `wasd.py`.
- **Nod / shake gestures** — a quick pitch oscillation = "yes" (confirm, sit),
  yaw oscillation = "no" (cancel). The gyroscope field in the JSON stream is
  perfect for this — threshold on angular velocity, not angle.
- **Arm + walk** — in `arm` mode, hold your gaze for a moment and Spot walks
  toward what the gripper is pointing at; nod to open/close the gripper.

**Interaction**
- **Mutual gaze** — combine with a face detector on Spot's front camera: when
  Spot sees you, it turns to face you; you turn your head, it mirrors. Two
  agents doing active perception at each other.
- **Spatial audio loop** — the XM5 already does head-tracked audio. Feed Spot's
  microphone (or a beamformed direction) back so sounds from Spot's side stay
  spatially fixed as you turn — a telepresence "ears on the robot".
- **Follow-me with anticipation** — normal follow uses your body position; add
  head yaw as a lead term so Spot starts turning before you do.

**Research angles**
- Compare fixed vs. head-driven camera selection for object-search tasks.
- Use head yaw as a prior in a visual SLAM / semantic mapping loop (attention
  weighting for keyframe selection).
- Latency study: BT HID (~40 ms) + gRPC + Spot's body controller — measure the
  end-to-end lag and whether smoothing or prediction (extrapolating with the
  gyroscope) helps.
