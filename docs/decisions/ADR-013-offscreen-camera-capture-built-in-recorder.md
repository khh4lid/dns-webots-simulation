# ADR-013 — Offscreen Camera capture instead of the built-in recorder

**Status:** Accepted

## Context
The project needs real footage for the report and the README. Webots ships a
movie recorder and an `exportImage` call.

## Options considered
1. `Supervisor.movieStartRecording`.
2. `Supervisor.exportImage`.
3. A `Camera` node child of the robot, rendered to an offscreen texture.

## Decision
**Camera node.** Options 1 and 2 fail silently in this environment: the movie
recorder produced a structurally valid MP4 whose 148 frames averaged 47 bytes,
and `exportImage` returned a perfectly uniform fill of the background colour
(1 unique colour, std 0.0) at three different drone poses. The Camera node
renders real pixels (520-564 unique colours, changing with the drone).

## Evidence
Both failure modes are silent — every API call "succeeds" and a valid file
appears — so only inspecting the pixels catches them. MP4s are verified by
`scripts/verify_mp4.py`, which parses the ISO-BMFF container directly and counts
per-frame sizes, because a static scene collapses to one or two distinct sizes.
Webots' bundled `ffmpeg.exe` also does not run here (exit 53), so the binary
from `imageio-ffmpeg` is used instead.

## Consequences
+ Every frame in the repository is a genuine render, labelled WEBOTS CAPTURE.
− Supervisor labels and the GUI's optional rendering do not appear in a Camera
  image, so all overlays are composited in post-processing.
− The camera is world-locked by writing its mount pose each step; it carries no
  mass or collider, so it changes nothing that is measured.

---

**Related:** ADR-001 · [ADR index](README.md) ·
[decision log](DECISIONS_LOG.md)
