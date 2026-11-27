"""DNS Webots simulation package.

Phase 1 deliverable for the Drone Navigation System research project
(Taylor's University).

Owner of this component: Khaled - Simulation Environment.

The package provides a Webots-backed implementation of the ``Backend`` protocol
defined in Spencer's ``navigation.py``, so a trained PPO policy can drive a
physically simulated quadcopter without any change to the learning code.
"""

__all__ = ["scenario", "frames", "autopilot", "sensing", "backend", "launcher", "worldgen"]
__version__ = "0.1.0"
