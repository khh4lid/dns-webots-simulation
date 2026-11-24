"""Generate the architecture decision records from authored content.

Fourteen ADRs, one file each, in a single shape: Context -> Options considered
-> Decision -> Evidence -> Consequences -> Status. Generated rather than
hand-maintained so the structure, the cross-links and the index never drift
apart, and so numbers quoted inside them are pulled from the result files.

Run::

    python scripts/make_adrs.py
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "decisions"
RESULTS = ROOT / "results" / "v2_S6"


def fact() -> dict:
    """Numbers pulled from the result files, so an ADR cannot quote a stale one."""
    values: dict = {}
    table = RESULTS / "scenario_table.csv"
    if table.is_file():
        with table.open(encoding="utf8") as handle:
            rows = [r for r in csv.DictReader(handle) if r["mode"] == "C"]
        values["corridors"] = " / ".join(
            f"{float(r['min_corridor_width_enroute_m']):.2f}" for r in rows)
        values["l02_iou"] = next(
            (f"{float(r['iou_to_L00']):.3f}" for r in rows
             if r["layout_id"] == "L02"), "TBC")
        values["l02_displacement"] = next(
            (f"{float(r['mean_obstacle_displacement_m']):.2f}" for r in rows
             if r["layout_id"] == "L02"), "TBC")
    consistency = RESULTS / "lidar_cart_consistency.json"
    if consistency.is_file():
        pooled = json.loads(consistency.read_text(encoding="utf8"))["cart_sector_pooled"]
        values["cart_mae"] = f"{pooled['mae_m']:.4f}"
        values["cart_n"] = str(pooled["comparisons"])
    reactive = RESULTS / "reactive_gnron_analysis.json"
    if reactive.is_file():
        crate = json.loads(reactive.read_text(encoding="utf8"))[
            "layouts"]["L00"]["surface_vs_centre_to_goal"]["crate_stack"]
        values["crate_surface"] = f"{crate['surface_to_goal_m']:.4f}"
        values["crate_centre"] = f"{crate['centre_to_goal_m']:.2f}"
    return values


F = fact()

ADRS = [
    dict(
        number=1, title="Webots R2025a as the simulator",
        status="Accepted",
        context="""
The environment had to provide a planar LiDAR, ground-truth obstacle poses for
scoring, physics-based quadrotor flight, a fixed and exactly divisible time
step, and had to run on the Windows laptop this project is developed on. It
also had to be driveable from an external Python process, because the policy
side is Spencer's code and must not be edited to fit the simulator.
""",
        options="""
| Simulator | LiDAR model | Ground-truth / scene control | Quadrotor physics | Windows | Licence |
|---|---|---|---|---|---|
| **Webots R2025a** | built-in `Lidar` node, configurable beams/FOV/range | `Supervisor` API reads and writes any node pose | built-in `Propeller` thrust model; samples ship with the install | first-class native build | Apache 2.0 |
| Gazebo (Ignition/Harmonic) | built-in ray sensors | services and plugins | via plugins such as PX4 SITL | ROS-centric; Windows support is partial | Apache 2.0 |
| AirSim | built-in LiDAR | API over RPC | Unreal-based, high fidelity | yes | MIT; **archived by Microsoft, succeeded by Project AirSim** |
| gym-pybullet-drones | no built-in LiDAR (ray casts must be written) | direct PyBullet state access | quadrotor-specific, RL-first | yes | MIT |
| Isaac Sim | built-in RTX LiDAR | USD + Python API | yes | yes | proprietary; **requires an RTX GPU and a large install** |

Sources are listed in the repository references; each was checked against the
project's own documentation or repository rather than from memory.
""",
        decision="""
**Webots R2025a.** It is the only option in the table that supplied all four of
the things this contribution needs — a built-in configurable LiDAR, a
supervisor API for both scene control and ground truth, physics-based rotor
flight, and a native Windows build — without additional middleware.

The deciding criterion was the *supervisor API*. Every validator, the analytic
ray-caster comparison, the swept collision test and the world-vs-scenario check
all need to read and write exact body poses from outside the controller. That
is a first-class Webots feature, where in the alternatives it is either a
plugin layer or hand-written.
""",
        evidence="""
Determinism, which the whole experiment rests on, was measured rather than
assumed: re-running the v1 protocol on the regenerated world reproduced all ten
cells with a **maximum per-episode path difference of 0 m**
(`results/v2_S6/v1_reproduction.csv`, figure `E9_l00_reproduction.png`).

This is a **selection rationale, not a benchmark**: no competing simulator was
installed, instrumented or timed for this project, and no performance
comparison is claimed.
""",
        consequences="""
+ One dependency, no ROS, no GPU requirement; the world opens offline.
+ `Supervisor` gives exact ground truth for scoring without touching the sensor path.
− Webots' own movie recorder does not work in this environment (see ADR-013).
− Tied to Webots' rotor model; a different flight model would need re-tuning.
""",
        image=None, links="ADR-008, ADR-012, ADR-013"),

    dict(
        number=2, title="Keep the 10 x 6 m arena",
        status="Accepted",
        context="""
A larger hall would make the navigation problem more interesting. But Spencer's
PPO checkpoints were fitted in `planar.py` on exactly `LOW=[0,-3]`,
`HIGH=[10,3]`, and `navigation.observation` divides the relative goal vector by
10 while capping LiDAR range at 5 m.
""",
        options="""
1. Enlarge to 12 x 8 m for a harder task.
2. Keep 10 x 6 m and vary only obstacle placement.
3. Randomise the arena size per layout.
""",
        decision="""
**Keep 10 x 6 m in every layout L00-L05.** Enlarging the room would move the
goal feature and the free-space statistics outside the distribution the
checkpoints saw, so any difference between layouts would be confounded by the
arena change rather than caused by the obstacle layout.
""",
        evidence="""
Asserted for every layout by `tests/test_generator.py::every layout keeps the
frozen L00 envelope`, which compares arena bounds, start, goal, tolerance,
altitude, time step, speed cap and the full LiDAR configuration against L00.
""",
        consequences="""
+ Layout is the single independent variable across the ladder.
− A larger-hall study is a separate, legitimate experiment that this release
  does not run, and would have to be reported as out-of-distribution.
""",
        image=None, links="ADR-009"),

    dict(
        number=3, title="Fixed 1.0 m altitude and a single-layer 360 deg LiDAR",
        status="Accepted",
        context="""
Spencer's observation contract is planar: 12 sectors, fixed yaw, no altitude
term. The environment has to match that or the policy is given information it
cannot represent.
""",
        options="""
1. Full 3-D navigation with a multi-layer LiDAR.
2. Planar navigation at a fixed altitude with one scan layer.
""",
        decision="""
**Planar, cruise altitude 1.0 m, one scan layer at 1.14 m** (1.0 m cruise plus
a 0.14 m mount). Every obstacle must rise above the scan plane with margin, so
no body can be solid to the collision test and invisible to the sensor — the
worst possible combination, because the policy would be punished for hitting
something it was never shown.
""",
        evidence="""
Enforced twice: `Scenario._check_obstacle_visibility` rejects any obstacle at
or below the scan plane plus margin, and the generator's `scan_plane_height`
validator rejects the same case at draw time
(`tests/test_generator.py::validator rejects an obstacle below the scan plane`).
""",
        consequences="""
+ The sensor sees everything the collision test can hit.
− No vertical avoidance; flying over an obstacle is not a strategy the
  environment can express. Listed as a limitation.
""",
        image=None, links="ADR-007"),

    dict(
        number=4, title="Exact box footprints instead of bounding circles",
        status="Accepted",
        context="""
v1 modelled the rectangular storage racks as circles for both the collision test
and the ray cast. A 0.8 x 2.4 m rack has a bounding radius of 1.265 m, so the
circle covers large volumes of empty aisle.
""",
        options="""
1. Keep circles — simpler maths, one closed-form segment test.
2. Exact convex footprints: box distance, slab ray intersection, swept test.
""",
        decision="""
**Exact footprints.** `PlacedObstacle.surface_distance` uses the exact box
distance, `ray_distance` uses the slab method in the box's own frame, and the
collision test sweeps the drone's motion with a ternary search on the convex
distance function.
""",
        evidence="""
The defect this fixes is reproduced as a regression test: a drone at (0.9, 1.1)
beside a rack centred at the origin has a **true surface gap of 0.50 m**, while
the bounding circle of radius 1.265 m calls it a collision
(`tests/test_generator.py::box surface distance is exact, not a bounding
circle`; panel (d) of `F4_verification_panel.png`).
""",
        consequences="""
+ Collision statistics mean what they say.
+ The same geometry code serves the collision test, the ray caster and the
  feasibility checker, so a layout cannot be feasible under one model and
  collided under another (ADR-012).
− Each collision test costs more than a closed-form circle check.
""",
        image=None, links="ADR-012"),

    dict(
        number=5, title="The racks deliberately cross the straight start-goal line",
        status="Accepted",
        context="""
An earlier v1 layout left the straight line from start to goal open by 0.30 m.
The goal-direct baseline then threaded the gap and scored 3/3, so the static
condition measured nothing at all.
""",
        options="""
1. Leave the line open and rely on the moving cart for difficulty.
2. Make each rack cross the centre line by a controlled depth.
""",
        decision="""
**Every layout blocks the straight line.** In L00 each rack crosses y = 0 by
0.20 m; the generator enforces a minimum crossing depth of 0.12 m in every
generated layout and records the realised depth as a descriptor.
""",
        evidence="""
`tests/test_generator.py::every layout blocks the straight start-goal line`
checks all six. Realised intrusion depths are in
`results/v2_S6/scenario_table.csv`. `direct` scores 0 in Mode B on every
layout, which is the behaviour this decision is meant to produce.
""",
        consequences="""
+ The static condition is discriminative on its own, without the cart.
− `direct` can never succeed, so it is a floor reference rather than a baseline.
""",
        image="ADR-005_line_block.png", links="ADR-009"),

    dict(
        number=6, title="The cart is kinematic",
        status="Accepted with a stated limitation",
        context="""
The dynamic hazard has to be exactly reproducible for a given seed, so that two
runs of the same configuration are comparable.
""",
        options="""
1. Kinematic: write the cart's pose from a closed-form function of time.
2. Physical: give the cart a `Physics` node and drive it with a motor.
""",
        decision="""
**Kinematic.** `Obstacle.centre_at(t)` is a pure function of simulated time, so
the cart's trajectory is identical in every run with the same phase.
""",
        evidence="""
The t = 0 pose is the −1.9 m extreme of the sweep, **not** the sweep centre —
a v1 world-file defect that put the cart 1.9 m from where the model said it was
and corrupted a calibration reference by 1.34 m. It is now asserted at build
time for every body in every world (`scripts/build_worlds.py`), at run time by
`dns/worldcheck.check`, and as a unit test.
""",
        consequences="""
+ Exact reproducibility; the period is 16.8889 s at 0.45 m/s over +-1.9 m.
− **Webots reports no rigid-body velocity for the cart, so physical contact
  response with it is not modelled.** A `collision_cart` outcome means the swept
  footprints intersected, not that an impact was simulated. A real cart would
  push the drone. Stated wherever cart collisions are reported.
− "Cart pose vs its own motion model = 0" is true by construction and is
  therefore an implementation-consistency check, never evidence (see C2).
""",
        image="ADR-006_cart_spacetime.png", links="ADR-012"),

    dict(
        number=7, title="Gimbal-stabilised LiDAR mount",
        status="Accepted with a stated limitation",
        context="""
Spencer's 12-sector contract assumes a horizontal scan plane. A body-fixed
scanner tilts with the airframe, so the plane dips and the sector ranges stop
meaning what the contract says.
""",
        options="""
1. Body-fixed mount — what a cheap fixed scanner does.
2. Ideal gimbal — counter-rotate the mount every control tick.
""",
        decision="""
**Ideal gimbal (`lidar_mount_mode = "stabilised"`).** The mount is
counter-rotated each tick so the scan plane stays horizontal whatever the
airframe is doing.
""",
        evidence="""
Measured in v1 (`docs/SENSOR_ATTITUDE.md`): a body-fixed plane lets a wall only
slightly taller than the sensor drop out of the scan, and Webots' out-of-range
`inf` is then encoded as "observed clear to 5 m". The mount height is also
constrained — at 0.075 m the levelled plane clipped the rear rotor and two
sectors read a constant 0.177 m, exactly the rotor tip radius; the clearance is
now asserted in `Scenario._check_lidar_clearances`.
""",
        consequences="""
+ The sector contract is actually true, matching the 2-D world the policies
  were trained in.
− **An ideal gimbal is a best case, not a real sensor.** A real stabilised mount
  has finite bandwidth and residual error. Ideal-ON and body-fixed-OFF bracket a
  real gimbal; the bracket experiment is listed as not yet run.
""",
        image=None, links="ADR-003"),

    dict(
        number=8, title="5 Hz velocity commands over a 125 Hz loop and an 8 ms step",
        status="Accepted",
        context="""
Three clocks have to nest exactly: Spencer's contract fixes one observation
every `DT = 0.2 s`; a quadrotor cannot track a velocity step instantly, so it
needs a fast inner attitude loop; and Webots advances in fixed `basicTimeStep`
increments.
""",
        options="""
1. Command the rotors directly at the policy rate.
2. A cascaded autopilot: velocity -> tilt -> rotor thrust, with the policy
   setting only the outer velocity setpoint.
""",
        decision="""
**Cascaded, with an 8 ms physics step.** 0.2 s is exactly 25 steps of 8 ms, so
the observation interval is an integer number of physics steps and the
simulation clock cannot drift away from Spencer's `DT` check. The autopilot
inner loop runs every physics step (125 Hz).
""",
        evidence="""
`Scenario.steps_per_command` raises if the period is not a whole multiple, the
contract test measures the realised interval as simulated time, and v1 recorded
an interval error below 3e-16 s.
""",
        consequences="""
+ Timing is exact by construction, not by tolerance.
− The autopilot's tilt authority (0.45 rad) bounds how aggressively a velocity
  command can be tracked; a tracker that ignores this can tumble the airframe
  (see ADR-014).
""",
        image="ADR-008_timing_ladder.png", links="ADR-001, ADR-014"),

    dict(
        number=9, title="A seeded generator and a graded shift ladder",
        status="Accepted",
        context="""
v1 measured the policies on one layout, so it could not distinguish navigation
competence from layout-specific memorisation. Testing on new layouts requires
those layouts to be reproducible, validated and frozen before any controller
sees them.
""",
        options="""
1. Hand-author a few new worlds in the Webots editor.
2. A config-plus-seed generator emitting canonical scenario JSON, with the
   world written from the JSON.
""",
        decision="""
**Generator.** `config/scenarios/generator_config.yaml` plus a seed produces a
byte-identical scenario JSON, hashed, from which the world is written. Six
layouts form a graded ladder: L00 baseline, L01 perturbed, L02 mirrored,
L03 aisle shift, L04 clutter, L05 hard.
""",
        evidence=f"""
Byte-identity is tested **across separate processes**, which specifically guards
against seeding from Python's per-process-randomised `hash()`
(`tests/test_generator.py`). Measured en-route corridor widths across the
ladder: **{F.get('corridors', 'TBC')} m**. All six are A*-feasible and all block
the straight line.
""",
        consequences="""
+ Any layout can be regenerated from the config and checked against its hash.
+ L01-L05 are flagged TEST-ONLY; training on them would require a fresh
  held-out set.
− Six layouts with one seed per family is a pilot, not a distribution: it is far
  too few to fit a trend, and no trend is fitted.
""",
        image=None, links="ADR-010, ADR-011, ADR-012"),

    dict(
        number=10, title="The mirror test as a memorisation probe",
        status="Accepted",
        context="""
A success-rate drop on a new layout has two possible causes that a single
comparison cannot separate: the layout is harder, or the policy was keyed to the
old geometry. Separating them needs a layout that is maximally *different* while
being exactly as *hard*.
""",
        options="""
1. Compare against a randomly perturbed layout and reason about difficulty.
2. Construct an exact reflection, whose difficulty is identical by construction.
""",
        decision="""
**L02 is L00 reflected about y = 0**: y -> -y, yaw -> -yaw, and the cart's
crossing phase shifted by pi. The phase shift is used rather than a negative
amplitude because the triangle-wave offset satisfies f(u + 1/2) = -f(u)
exactly, so a half-period shift reflects the whole trajectory, whereas a
negative amplitude would corrupt the period formula (period = 4A/v).
""",
        evidence=f"""
L02 has IoU **{F.get('l02_iou', 'TBC')}** with L00 and a mean obstacle
displacement of **{F.get('l02_displacement', 'TBC')} m**, yet identical A* length
(9.004 m), tortuosity (1.085), en-route corridor width (2.080 m) and intrusion
depth (0.200 m) — asserted by `tests/test_generator.py::L02 matches L00 on every
difficulty descriptor`. Visual proof that it was flown mirrored:
`assets/figures/E15_mirror_overlay.png` and `assets/gifs/mirror_l00_vs_l02.gif`.
""",
        consequences="""
+ An L00-vs-L02 difference cannot be a difficulty effect.
− The test probes **left/right layout identity only**. It is not a general
  generalisation result, and is always reported alongside L05, where the same
  checkpoints score 0/45 on a layout whose traversability was confirmed.
""",
        image=None, links="ADR-009, ADR-011"),

    dict(
        number=11, title="Distinct configurations, Wilson intervals, no seed ranking",
        status="Accepted",
        context="""
v1 reported Mode B as "3/3" per controller. Those three episodes are the same
configuration run three times: the episode seed is accepted by
`WebotsBackend.reset` and never used, there is no start jitter, and
`PPO.predict` is `tanh(actor(obs))` with no sampling.
""",
        options="""
1. Report episode counts as the sample size.
2. Count distinct documented configurations, and treat identical deterministic
   repeats as one.
3. Add start jitter so the three Mode B episodes become genuinely distinct.
""",
        decision="""
**Count distinct configurations.** Mode B is n = 1 per controller; Mode C is
n = 15 (fifteen distinct cart phases). Option 3 was rejected because the
contract freezes the start point, and breaking a frozen contract to improve a
sample size is the wrong trade.

Rates are pooled over the three checkpoints with Wilson 95% intervals. Per-seed
values are plotted but never ranked, and no significance test is reported.
""",
        evidence="""
`results/v2_S6/v1_distinctness.json` lists, per cell, how many distinct
configurations the episodes represent and the path-length spread across the
repeats (<= 0.0065 m for PPO). Pooled rates and intervals:
`results/v2_S6/pooled_rates.csv`.
""",
        consequences="""
+ Every reported rate states the n it is over, and 540 episodes are reported as
  480 distinct configurations.
− Mode B claims become much weaker than v1 implied: L00 Mode B pooled is 3/3
  with a Wilson interval of [0.439, 1.000].
""",
        image=None, links="ADR-009"),

    dict(
        number=12, title="One source of geometry, checked against the loaded world",
        status="Accepted",
        context="""
Two v1 defects shared a root cause. The calibration tool loaded a fixed scenario
and so compared one world's measurements against another world's geometry; and
the world writer placed the moving cart at its motion centre rather than its
t = 0 pose. Both were silent: every API call succeeded and plausible numbers
came out.
""",
        options="""
1. Careful review and a naming convention.
2. Make the scenario JSON the single source of geometry, and verify at run time
   that the loaded world implements it.
""",
        decision="""
**Single source plus a run-time check.** Every world, validator, ray caster,
figure and run reads geometry from the scenario JSON by id. `dns/worldcheck.py`
then asks the running simulator where its bodies are and **refuses to produce
results** if any obstacle's t = 0 pose, the time step, the observation interval
or the LiDAR beams/FOV/cap disagree with the scenario.
""",
        evidence="""
Maximum obstacle t = 0 pose error over all six layouts: **<= 4.0e-06 m**
(`results/layout_gate/SUMMARY.json`). Each world also carries the SHA-256 of the
scenario that produced it, and the experiment client asserts it before the first
episode.
""",
        consequences="""
+ A world that has drifted from its definition fails loudly instead of quietly
  producing misattributed results.
− Every geometry change means regenerating worlds; hand-editing a `.wbt` is no
  longer supported.
""",
        image=None, links="ADR-004, ADR-006"),

    dict(
        number=13, title="Offscreen Camera capture instead of the built-in recorder",
        status="Accepted",
        context="""
The project needs real footage for the report and the README. Webots ships a
movie recorder and an `exportImage` call.
""",
        options="""
1. `Supervisor.movieStartRecording`.
2. `Supervisor.exportImage`.
3. A `Camera` node child of the robot, rendered to an offscreen texture.
""",
        decision="""
**Camera node.** Options 1 and 2 fail silently in this environment: the movie
recorder produced a structurally valid MP4 whose 148 frames averaged 47 bytes,
and `exportImage` returned a perfectly uniform fill of the background colour
(1 unique colour, std 0.0) at three different drone poses. The Camera node
renders real pixels (520-564 unique colours, changing with the drone).
""",
        evidence="""
Both failure modes are silent — every API call "succeeds" and a valid file
appears — so only inspecting the pixels catches them. MP4s are verified by
`scripts/verify_mp4.py`, which parses the ISO-BMFF container directly and counts
per-frame sizes, because a static scene collapses to one or two distinct sizes.
Webots' bundled `ffmpeg.exe` also does not run here (exit 53), so the binary
from `imageio-ffmpeg` is used instead.
""",
        consequences="""
+ Every frame in the repository is a genuine render, labelled WEBOTS CAPTURE.
− Supervisor labels and the GUI's optional rendering do not appear in a Camera
  image, so all overlays are composited in post-processing.
− The camera is world-locked by writing its mount pose each step; it carries no
  mass or collider, so it changes nothing that is measured.
""",
        image=None, links="ADR-001"),

    dict(
        number=14, title="L04 excluded from the generalisation conclusion",
        status="Proposed — awaiting the supervisor's decision",
        context="""
All five controllers score 0/90 on L04. Before that can be read as a
generalisation result, the layout has to be shown flyable by this airframe
within the 90-step horizon.
""",
        options="""
1. Report 0/90 as a generalisation result.
2. Regenerate L04 until it behaves.
3. Report the data in full, mark traversability unconfirmed, and exclude it from
   the conclusion.
""",
        decision="""
**Option 3.** Option 2 would alter a layout after seeing its outcomes, which
breaks the pre-registration. Option 1 would claim something the evidence does
not support.
""",
        evidence="""
L04 passes every generator validator and is A*-feasible with an en-route
corridor of 0.850 m. The flown reference traversal reaches the goal at no tested
setting: at a 0.22 m planning margin and <= 0.48 m/s the tracker is stable
(peak tilt 0.105 rad) and collision-free but **times out at 90 steps**; at
higher speeds it collides near x = 3.1 m, which is where all three PPO
checkpoints also collide. At a 0.12 m margin the airframe tumbled to 1.5905 rad
against 0.45 rad of tilt authority. The controllers themselves do **not**
destabilise: 0 of 90 L04 episodes exceeded 0.50 rad.
""",
        consequences="""
+ The generalisation claim rests only on layouts whose traversability was
  confirmed. L05, where the reference traversal reached the goal in 56 steps, is
  reported as a **valid** 0/45 failure.
− One rung of the ladder yields no conclusion, and the question of whether L04
  is flyable at all is still open.

**Decision needed from Dr. Mehdi:** keep L04 excluded, or authorise a
diagnostic-only rerun with a longer horizon (which would sit outside the frozen
contract and so could not feed the main result)?
""",
        image=None, links="ADR-009, ADR-011"),
]


TEMPLATE = """# ADR-{number:03d} — {title}

**Status:** {status}
{image_block}
## Context
{context}
## Options considered
{options}
## Decision
{decision}
## Evidence
{evidence}
## Consequences
{consequences}
---

**Related:** {links} · [ADR index](README.md) ·
[decision log](DECISIONS_LOG.md)
"""


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    index = ["# Architecture decision records",
             "",
             "Why the simulation environment is built the way it is. Each record",
             "states the context, the options that were on the table, what was",
             "decided, the evidence behind it and what it costs.",
             "",
             "| ADR | Decision | Status |", "|---|---|---|"]
    for adr in ADRS:
        image_block = ""
        if adr["image"]:
            image_block = (
                f"\n<img src=\"../../assets/decisions/{adr['image']}\" "
                f"width=\"760\" alt=\"Evidence for ADR-{adr['number']:03d}\">\n\n"
                f"<sub>Figure — evidence for ADR-{adr['number']:03d} · "
                f"LOGGED-DATA RENDER · source: "
                f"`assets/decisions/{adr['image']}`</sub>\n")
        text = TEMPLATE.format(
            number=adr["number"], title=adr["title"], status=adr["status"],
            image_block=image_block,
            context=adr["context"].strip() + "\n",
            options=adr["options"].strip() + "\n",
            decision=adr["decision"].strip() + "\n",
            evidence=adr["evidence"].strip() + "\n",
            consequences=adr["consequences"].strip() + "\n",
            links=adr["links"])
        slug = "-".join(
            w.strip(",.()").lower() for w in adr["title"].split()
            if w.lower() not in ("the", "a", "an", "and", "as", "of", "for",
                                 "from", "instead", "with", "over", "into"))[:48]
        name = f"ADR-{adr['number']:03d}-{slug.strip('-')}.md"
        (OUT / name).write_text(text, encoding="utf8")
        index.append(f"| [ADR-{adr['number']:03d}]({name}) | {adr['title']} | "
                     f"{adr['status']} |")
        print(f"wrote docs/decisions/{name}")

    index += ["",
              "The running decision log with the smaller day-to-day choices is",
              "[DECISIONS_LOG.md](DECISIONS_LOG.md); the assumption register is",
              "[../verification/ASSUMPTIONS.md](../verification/ASSUMPTIONS.md)."]
    (OUT / "README.md").write_text("\n".join(index) + "\n", encoding="utf8")
    print(f"wrote docs/decisions/README.md ({len(ADRS)} records)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
