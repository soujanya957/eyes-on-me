"""MuJoCo stand-in for Spot: ``eyes-on-me --sim`` drives it instead of a robot.

``spot_sim.SimSpot`` takes the same SDK commands as the real robot and poses
the Menagerie Spot model kinematically; ``window.SimWindow`` renders it.
"""

from .spot_sim import SimLimits, SimSpot

__all__ = ["SimLimits", "SimSpot"]
