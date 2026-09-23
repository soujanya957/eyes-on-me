"""Find (or fetch once) the Spot-with-arm model from MuJoCo Menagerie, and dress the scene.

The model is Google DeepMind's ``boston_dynamics_spot`` from
https://github.com/google-deepmind/mujoco_menagerie (BSD-3). It is not
vendored: the first run clones just that directory into
``~/.cache/eyes-on-me/menagerie``. ``$EYES_ON_ME_MENAGERIE`` points at an
existing checkout instead.
"""

from __future__ import annotations

import logging
import math
import os
import subprocess
from pathlib import Path

import mujoco

log = logging.getLogger("eyes_on_me.sim")

MENAGERIE_URL = "https://github.com/google-deepmind/mujoco_menagerie"
DEFAULT_CACHE = Path.home() / ".cache" / "eyes-on-me" / "menagerie"

#: Landmarks on a ring around the start pose, so turning and looking have
#: something to point at. Colour order goes round anticlockwise from ahead.
LANDMARK_RADIUS_M = 3.0
LANDMARK_COLOURS = (
    (0.90, 0.20, 0.20),  # ahead: red
    (0.95, 0.60, 0.10),
    (0.95, 0.90, 0.20),  # left: yellow
    (0.30, 0.80, 0.30),
    (0.20, 0.60, 0.95),  # behind: blue
    (0.55, 0.35, 0.90),
    (0.90, 0.40, 0.75),  # right: pink
    (0.85, 0.85, 0.85),
)


def menagerie_dir() -> Path:
    root = Path(os.environ.get("EYES_ON_ME_MENAGERIE", DEFAULT_CACHE))
    spot = root / "boston_dynamics_spot"
    if (spot / "scene_arm.xml").is_file():
        return spot
    if "EYES_ON_ME_MENAGERIE" in os.environ:
        raise SystemExit(f"error: no boston_dynamics_spot/scene_arm.xml under $EYES_ON_ME_MENAGERIE={root}")
    log.info("fetching the Spot model from MuJoCo Menagerie into %s (one time)", root)
    root.parent.mkdir(parents=True, exist_ok=True)
    try:
        if not root.exists():
            subprocess.run(["git", "clone", "--quiet", "--depth", "1", "--filter=blob:none", "--sparse",
                            MENAGERIE_URL, str(root)], check=True)
        subprocess.run(["git", "-C", str(root), "sparse-checkout", "set", "boston_dynamics_spot"], check=True)
    except (OSError, subprocess.CalledProcessError) as e:
        raise SystemExit(f"error: could not fetch the Spot model ({e}); check the network, or clone "
                         f"{MENAGERIE_URL} yourself and set EYES_ON_ME_MENAGERIE") from e
    return spot


def load_model() -> mujoco.MjModel:
    """Spot with arm, a checker floor, and a ring of coloured posts."""
    spec = mujoco.MjSpec.from_file(str(menagerie_dir() / "scene_arm.xml"))
    for i, rgb in enumerate(LANDMARK_COLOURS):
        a = 2 * math.pi * i / len(LANDMARK_COLOURS)
        x, y = LANDMARK_RADIUS_M * math.cos(a), LANDMARK_RADIUS_M * math.sin(a)
        post = spec.worldbody.add_geom()
        post.name = f"landmark{i}"
        post.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        post.size = [0.12, 0.6, 0]
        post.pos = [x, y, 0.6]
        post.rgba = [*rgb, 1]
        post.contype = post.conaffinity = 0
    model = spec.compile()
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, 1280)
    model.vis.global_.offheight = max(model.vis.global_.offheight, 960)
    return model
