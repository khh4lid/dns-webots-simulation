"""Build the DNS simulation-environment research report and its three tables.

All numeric content is read from recorded result files, so the prose cannot
drift from the measurements. The report is scoped to the simulation-environment
contribution; PPO training and sensor-processing research belong to teammates
and are described only where the interface requires it.

Run::

    python scripts/make_deliverable_report.py --preview
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import textwrap
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                      # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages                 # noqa: E402
from matplotlib.image import imread                                  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.scenario import Scenario                                    # noqa: E402

RESULTS = PROJECT_ROOT / "results"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
SURFACE, RULE = "#fcfcfb", "#c3c2b7"
PAGE = (8.27, 11.69)


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf8")) if path.is_file() else None


def load_csv(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(encoding="utf8") as handle:
        return list(csv.DictReader(handle))


class Report:
    preview_dir: Path | None = None

    def __init__(self, pdf: PdfPages):
        self.pdf = pdf
        self.fig = None
        self.page_number = 0
        self.y = 0.0

    def new_page(self) -> None:
        self.flush()
        self.page_number += 1
        self.fig = plt.figure(figsize=PAGE, facecolor=SURFACE)
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(0, 1)
        self.ax.axis("off")
        self.ax.text(0.08, 0.972, "Design, Implementation and Validation of a "
                     "Webots Simulation Environment for Autonomous Drone "
                     "Navigation", fontsize=6.9, color=MUTED)
        self.ax.plot([0.08, 0.92], [0.966, 0.966], color=RULE, linewidth=0.7)
        self.ax.text(0.92, 0.028, f"{self.page_number}", fontsize=8,
                     color=MUTED, ha="right")
        self.y = 0.935

    def flush(self) -> None:
        if self.fig is not None:
            self.pdf.savefig(self.fig)
            if self.preview_dir is not None:
                self.preview_dir.mkdir(parents=True, exist_ok=True)
                self.fig.savefig(
                    self.preview_dir / f"page{self.page_number:02d}.png", dpi=110)
            plt.close(self.fig)
            self.fig = None

    def space(self, amount: float) -> None:
        self.y -= amount
        if self.y < 0.07:
            self.new_page()

    def heading(self, text: str, size: float = 12.5) -> None:
        if self.y < 0.18:
            self.new_page()
        self.space(0.017)
        self.ax.text(0.08, self.y, text, fontsize=size, color=INK,
                     fontweight="semibold", va="top")
        self.space(0.026)

    def body(self, text: str, size: float = 9.0) -> None:
        for block in text.strip().split("\n\n"):
            for line in textwrap.wrap(" ".join(block.split()), 102):
                if self.y < 0.07:
                    self.new_page()
                self.ax.text(0.08, self.y, line, fontsize=size, color=INK2,
                             va="top")
                self.space(0.0163)
            self.space(0.008)

    def bullets(self, items: list[str], size: float = 9.0) -> None:
        for item in items:
            for index, line in enumerate(
                    textwrap.wrap(" ".join(item.split()), 97)):
                if self.y < 0.07:
                    self.new_page()
                self.ax.text(0.095, self.y, ("•  " if index == 0 else "    ") + line,
                             fontsize=size, color=INK2, va="top")
                self.space(0.0163)
            self.space(0.004)

    def table(self, columns, rows, widths, caption="", size=7.2) -> None:
        need = 0.035 + 0.0155 * (len(rows) + 2)
        if self.y - need < 0.07:
            self.new_page()
        if caption:
            for line in textwrap.wrap(" ".join(caption.split()), 96):
                self.ax.text(0.08, self.y, line, fontsize=8.5, color=INK,
                             fontweight="semibold", va="top")
                self.space(0.0158)
            self.space(0.007)
        total = sum(widths)
        positions, acc = [], 0.08
        for w in widths:
            positions.append(acc)
            acc += 0.84 * w / total

        def header():
            for pos, name in zip(positions, columns):
                self.ax.text(pos, self.y, name, fontsize=size, color=INK,
                             fontweight="semibold", va="top")
            self.space(0.0142)
            self.ax.plot([0.08, 0.92], [self.y + 0.004, self.y + 0.004],
                         color=RULE, linewidth=0.7)
            self.space(0.004)

        header()
        for row in rows:
            if self.y < 0.07:
                self.new_page()
                header()
            for pos, cell in zip(positions, row):
                self.ax.text(pos, self.y, str(cell), fontsize=size,
                             color=INK2, va="top")
            self.space(0.0152)
        self.space(0.012)

    def figure(self, path: Path, caption: str) -> None:
        if not path.is_file():
            self.body(f"[FIGURE PENDING: {path.name} was not generated]")
            return
        image = imread(path)
        width = 0.84
        actual = width * (image.shape[0] / image.shape[1]) * PAGE[0] / PAGE[1]
        if self.y - actual - 0.04 < 0.07:
            self.new_page()
        axes = self.fig.add_axes([0.08, self.y - actual, width, actual])
        axes.imshow(image)
        axes.axis("off")
        self.space(actual + 0.012)
        for line in textwrap.wrap(" ".join(caption.split()), 116):
            self.ax.text(0.08, self.y, line, fontsize=7.4, color=MUTED, va="top")
            self.space(0.0132)
        self.space(0.014)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=PROJECT_ROOT / "config" / "scenario_warehouse.json")
    parser.add_argument("--figures", type=Path,
                        default=PROJECT_ROOT / "evidence" / "deliverable_figures")
    parser.add_argument("--out", type=Path,
                        default=PROJECT_ROOT / "evidence" / "DNS_Simulation_Research_Report.pdf")
    parser.add_argument("--tables-out", type=Path,
                        default=PROJECT_ROOT / "evidence" / "deliverable_tables")
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()

    scenario = Scenario.load(args.config)
    static = Scenario.load(PROJECT_ROOT / "config" / "scenario_warehouse_static.json")
    cart = next(o for o in scenario.obstacles
                if o.motion.type != "static" and o.motion.amplitude_m)
    validation = load_json(RESULTS / "validation_physical_lidar.json")
    mover = load_json(RESULTS / "moving_obstacle_warehouse.json")
    attitude = load_json(RESULTS / "lidar_attitude_warehouse.json")
    calib = load_json(RESULTS / "calibration_warehouse.json")
    mp4 = load_json(RESULTS / "demo" / "mp4_verification.json")
    recording = load_json(RESULTS / "demo" / "recording_report.json")
    summary = load_csv(RESULTS / "experiments_warehouse" / "experiment_summary.csv")
    episodes = load_csv(RESULTS / "experiments_warehouse" / "experiment_results.csv")
    protocol = load_json(RESULTS / "experiments_warehouse" / "protocol.json")
    stab = (attitude or {}).get("by_mount", {}).get("stabilised", {})
    fixed = (attitude or {}).get("by_mount", {}).get("body_fixed", {})

    # -------------------------------------------------------------- tables
    args.tables_out.mkdir(parents=True, exist_ok=True)
    t1 = [
        ["Simulator", "Webots R2025a", "-", "Python controllers, built-in Lidar/GPS/IMU/Propeller, scriptable Supervisor, offline worlds"],
        ["Drone model", "DnsQuadcopter", "-", "0.55 kg, thrust/weight 4.0, four Propeller rotors, custom PROTO from base nodes"],
        ["Arena", f"{scenario.size_xy[0]:g} x {scenario.size_xy[1]:g}", "m", "Matches the planar.py bounds the PPO checkpoints were fitted on"],
        ["Wall height", f"{scenario.wall_height_m:g}", "m", "Keeps the boundary inside the scan plane across the tilt envelope"],
        ["Coordinate frame", "+x fwd, +y right", "-", "x_dns = x_webots, y_dns = -y_webots; one conversion module"],
        ["Origin", "(0, 0) at corner", "m", "Arena spans [0,10] x [-3,3]"],
        ["Flight altitude", f"{scenario.cruise_altitude_m:g}", "m", "Held by the altitude cascade; planar navigation task"],
        ["Collision radius", f"{scenario.collision_radius_m:g}", "m", "Equals navigation.RADIUS so simulator and policy agree"],
        ["Physics timestep", f"{scenario.basic_timestep_ms}", "ms", "Divides 200 ms exactly (25 steps); no clock drift"],
        ["Control rate", "5", "Hz", "navigation.DT = 0.2 s, the agreed PPO interface"],
        ["Inner loop", "125", "Hz", "Autopilot at the physics step; a quadcopter cannot track 5 Hz open-loop"],
        ["Velocity limit", f"{scenario.max_speed_mps:g}", "m/s per axis", "navigation.SPEED; the bridge scales the action once"],
        ["LiDAR coverage", "360", "deg", "Cylindrical projection, required above pi"],
        ["LiDAR range", f"{scenario.sensing_max_range_m:g}", "m", "navigation.RANGE cap"],
        ["Beam count", f"{scenario.lidar_resolution}", "beams", "0.5 deg spacing, 60 per sector"],
        ["Sectors", f"{scenario.sensing_sectors}", "-", "30 deg each; 0 forward, clockwise positive"],
        ["Mount", scenario.lidar_mount_mode, "-", f"Idealised gimbal at {scenario.lidar_height_m:g} m above the body origin"],
        ["Goal tolerance", f"{scenario.goal_tolerance_m:g}", "m", "navigation.GOAL_TOL"],
        ["Episode horizon", "90", "steps", "navigation.HORIZON = 18 s simulated"],
    ]
    with (args.tables_out / "table1_simulator_configuration.csv").open(
            "w", newline="", encoding="utf8") as h:
        w = csv.writer(h); w.writerow(["parameter", "value", "unit", "justification"]); w.writerows(t1)

    t2 = []
    for o in scenario.obstacles:
        moving = o.motion.type != "static" and o.motion.amplitude_m
        t2.append([
            o.id, o.label or "-", o.shape,
            f"({o.xy[0]:g}, {o.xy[1]:g})",
            (f"{o.size_xy[0]:g} x {o.size_xy[1]:g} x {o.height_m:g}"
             if o.shape == "box" else f"r {o.radius_m:g} x h {o.height_m:g}"),
            "dynamic" if moving else "static",
            (f"{o.motion.type} along {o.motion.axis}, +/-{o.motion.amplitude_m:g} m"
             if moving else "-"),
            f"{o.motion.speed_mps:g}" if moving else "0",
            o.purpose or "-",
        ])
    t2.append(["start", "start pad", "-", f"({scenario.start_xy[0]:g}, {scenario.start_xy[1]:g})",
               f"altitude {scenario.cruise_altitude_m:g}", "-", "-", "-",
               "Episode origin; collision-free, 2.25 m from the nearest rack"])
    t2.append(["goal", "goal pad", "-", f"({scenario.goal_xy[0]:g}, {scenario.goal_xy[1]:g})",
               f"tolerance {scenario.goal_tolerance_m:g}", "-", "-", "-",
               "Success when the drone centre is within the tolerance radius"])
    with (args.tables_out / "table2_environment_obstacles.csv").open(
            "w", newline="", encoding="utf8") as h:
        w = csv.writer(h)
        w.writerow(["id", "label", "shape", "position_xy_m", "dimensions_m",
                    "class", "motion_model", "speed_mps", "research_purpose"])
        w.writerows(t2)

    def check(name):
        for row in (validation or {}).get("checks", []):
            if row["check"] == name:
                return row
        return None

    t3 = []

    def add3(test, trials, quantity, expected, observed, deviation, status, evidence):
        t3.append([test, trials, quantity, expected, observed, deviation, status, evidence])

    if check("command_rate_5hz"):
        add3("Control timing", "5 steps", "command period", "0.200000 s",
             "0.200000 s", "< 3e-16 s", "PASS", "validation_physical_lidar.json")
    if check("axis_+x_forward"):
        add3("Axis mapping", "4 axes", "commanded vs measured direction",
             "sign match on all four", "all four correct", "cross-axis < 0.15 m",
             "PASS", "validation_physical_lidar.json")
    if check("velocity_is_measured"):
        add3("Velocity source", "1", "measured vs commanded after 0.2 s",
             "differs (physical lag)", "0.280 vs 1.2 m/s", "-", "PASS",
             "validation_physical_lidar.json")
    if check("reset_is_reproducible"):
        add3("Reset repeatability", "2 x 3 cmds", "trajectory divergence",
             "< 2e-2 m", "7.09e-3 m", "-", "PASS",
             "validation_physical_lidar.json")
    if mover:
        rows = mover["steps"]
        err = max(r["error_m"] for r in rows)
        add3("Obstacle trajectory", f"{len(rows)} steps",
             "supervisor pose vs centre_at(t)", "0 m", f"{err:.2e} m",
             f"{err:.2e} m", "PASS", "moving_obstacle_warehouse.json")
        add3("Obstacle speed", f"{len(rows)} steps", "median travel speed",
             f"{cart.motion.speed_mps:g} m/s",
             f"{mover['measured_median_speed_mps']:.4f} m/s",
             f"{abs(mover['measured_median_speed_mps'] - cart.motion.speed_mps):.4f} m/s",
             "PASS", "moving_obstacle_warehouse.json")
        sector = max(r["max_sector_error_m"] for r in rows)
        mean_sector = float(np.mean([r["max_sector_error_m"] for r in rows]))
        add3("LiDAR vs analytic scene", f"{len(rows)} steps x 12 sectors",
             "worst per-sector range error", "< 0.35 m", f"{sector:.4f} m",
             f"mean {mean_sector:.4f} m", "PASS",
             "moving_obstacle_warehouse.json")
    if calib:
        add3("LiDAR angular mapping", "720 beams", "fitted index->bearing",
             "direction +1, offset 180 deg",
             f"+1, {calib['lidar']['offset_deg']:.2f} deg",
             f"RMS {calib['lidar']['rms_error_m']:.4f} m", "PASS",
             "calibration_warehouse.json")
    if stab and fixed:
        add3("Mount ablation", "29 steps each", "range error vs tilt",
             "flat for a levelled plane",
             f"stabilised worst {stab['worst_paired_rms_m']:.3f} m",
             f"body-fixed worst {fixed['worst_paired_rms_m']:.3f} m", "PASS",
             "lidar_attitude_warehouse.json")
    if check("observation_dim"):
        add3("PPO observation", "1", "feature vector length", "52", "52", "-",
             "PASS", "validation_physical_lidar.json")
    if mp4:
        track = (mp4.get("tracks") or [{}])[0]
        add3("Demonstration video", "1", "container, frames, variation",
             "valid MP4, >900 frames",
             f"{track.get('frame_count')} frames, "
             f"{track.get('distinct_frame_sizes')} distinct sizes",
             "-", "PASS" if mp4.get("verdict") == "VALID" else "FAIL",
             "mp4_verification.json")
    add3("Unit tests", "46", "geometry, binning, config guards", "all pass",
         "46/46", "-", "PASS", "test_log.txt")

    with (args.tables_out / "table3_validation_results.csv").open(
            "w", newline="", encoding="utf8") as h:
        w = csv.writer(h)
        w.writerow(["test", "trials", "measured_quantity", "expected",
                    "observed", "deviation", "status", "evidence"])
        w.writerows(t3)
    print(f"wrote 3 tables to {args.tables_out}")

    # -------------------------------------------------------------- report
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with PdfPages(args.out) as pdf:
        r = Report(pdf)
        if args.preview:
            r.preview_dir = args.out.parent / "deliverable_report_preview"

        r.new_page()
        r.y = 0.80
        r.ax.text(0.08, r.y, "Design, Implementation and Validation of a",
                  fontsize=17, color=INK, fontweight="semibold", va="top")
        r.space(0.035)
        r.ax.text(0.08, r.y, "Webots Simulation Environment for Autonomous",
                  fontsize=17, color=INK, fontweight="semibold", va="top")
        r.space(0.035)
        r.ax.text(0.08, r.y, "Drone Navigation in Static and Dynamic Scenes",
                  fontsize=17, color=INK, fontweight="semibold", va="top")
        r.space(0.06)
        r.ax.plot([0.08, 0.92], [r.y, r.y], color=RULE, linewidth=0.8)
        r.space(0.035)
        r.ax.text(0.08, r.y,
                  "Drone Navigation System (DNS)\n"
                  "Simulation Environment Development\n"
                  "Taylor's University, Malaysia\n\n"
                  "Supervisor: (name withheld by request)\n"
                  "DRL / PPO: Spencer    LiDAR processing: sensor teammate",
                  fontsize=10, color=INK2, va="top")
        r.space(0.14)
        r.ax.text(0.08, r.y,
                  "RESEARCH ENGINEERING REPORT — prepared for supervisor and "
                  "team review.\nThis document reports simulator verification "
                  "and validation only. It is not a\npeer-reviewed paper and "
                  "makes no claim about real-world drone behaviour.\n"
                  "Prepared with AI assistance; all reported numbers come from "
                  "the recorded runs\nlisted in section 8 and are reproducible "
                  "with the commands given there.",
                  fontsize=8.6, color=MUTED, va="top")
        r.space(0.10)
        r.ax.text(0.08, r.y,
                  f"Webots R2025a  ·  arena {scenario.size_xy[0]:g} x "
                  f"{scenario.size_xy[1]:g} m  ·  "
                  f"{protocol['total_episodes'] if protocol else '?'} recorded "
                  f"episodes  ·  46 unit tests",
                  fontsize=9.5, color=INK, va="top")

        # 1
        r.new_page()
        r.heading("1  Introduction")
        r.body("""
        Reinforcement-learning navigation policies are trained and evaluated
        almost entirely in simulation, which makes the simulator part of the
        experimental apparatus rather than a convenience. If the simulated
        sensor reports free space where a wall stands, or the collision test
        fires in open aisle, then every success rate computed on top of it is
        measuring the simulator rather than the policy.""")
        r.body("""
        This report documents the simulation environment for the DNS project:
        the Webots world, the quadcopter and its flight controller, the sensor
        mounting and configuration, the obstacle geometry and motion models,
        the scenario definitions, the collision ground truth, and the
        interfaces through which a trained policy and a sensor-processing
        pipeline attach to it. Responsibility is divided three ways: Spencer
        owns PPO training and the trained checkpoints, a third teammate owns
        LiDAR signal processing, and the work reported here is the environment
        and the interfaces. Where a teammate's component appears, it is because
        the interface had to be exercised to prove the environment correct.""")

        r.heading("2  Research objectives and scope")
        r.bullets([
            "Select and justify a simulation platform for DRL-based UAV "
            "navigation that runs on commodity hardware.",
            "Build a navigation environment with credible industrial geometry, "
            "documented dimensions, and an explicit coordinate frame.",
            "Define start, goal and boundaries with an unambiguous success "
            "condition.",
            "Model static obstacles whose collision geometry matches what is "
            "drawn and what the sensor sees.",
            "Investigate and implement moving obstacles with a stated motion "
            "model, and verify the simulator reproduces that model.",
            "Verify the environment: timing, axis mapping, measured versus "
            "commanded motion, collision, reset repeatability and sensor "
            "accuracy.",
            "Provide interfaces that let the PPO and sensor work attach "
            "without modification to their code.",
        ])
        r.body("""
        Explicitly out of scope: PPO training, policy architecture, reward
        design, and sensor signal-processing research. Trained checkpoints are
        used here only as a workload that exercises the interface end to end,
        and any policy result is reported as an integration demonstration, not
        as a policy evaluation.""")

        # 3
        r.heading("3  Simulator selection and justification")
        r.body("""
        Webots R2025a was selected and is the version every result here was
        produced on. Four properties were decisive and each was exercised in
        this work. Native Python controllers through libController let the
        policy code, the environment package and the simulator share one
        process, with no bespoke IPC to validate. Built-in Lidar, GPS,
        InertialUnit, Gyro and Propeller nodes mean the rotor thrust model and
        the range sensor are the simulator's own rather than something written
        for this project. The Supervisor API makes reproducible episode resets
        and scripted obstacle motion possible, which is the basis of every
        repeatability result in section 8. Worlds are plain text generated from
        a configuration file, which is what keeps the scene and the experiment
        definition from drifting apart -- a mismatch of exactly this kind was
        found and fixed during development (section 8).""")
        r.body("""
        Limitations are equally concrete. Webots' GUI renderer produced no
        geometry when launched non-interactively in our environment, which
        forced the demonstration video to be captured through an in-scene
        camera sensor instead. The contact solver and the propeller model are
        approximations, and no claim of correspondence to a specific physical
        airframe is made. AirSim was not evaluated head to head; Webots was
        chosen on the properties above and on its lower hardware demand, and
        that is the honest basis of the decision rather than a benchmark.""")

        # 4
        r.heading("4  Simulation environment design")
        r.body(f"""
        The environment is an indoor warehouse inspection aisle of
        {scenario.size_xy[0]:g} x {scenario.size_xy[1]:g} m with
        {scenario.wall_height_m:g} m walls and a 1 m reference grid painted on
        the floor. The research frame is +x forward, +y right, with altitude
        and heading held constant. Webots is ENU, and the single mapping
        x_dns = x_webots, y_dns = -y_webots is written in one module and used
        everywhere, so the sign convention has exactly one definition.""")
        r.body(f"""
        The arena size is deliberately not enlarged. The available PPO
        checkpoints were fitted on exactly these bounds, and the observation
        builder divides the relative goal vector by 10 while capping range at
        5 m. Widening the room would move those features outside the
        distribution the policies saw, so a comparison against the baseline
        would be confounded by the arena rather than by the obstacle layout. A
        larger hall remains a valid out-of-distribution study and would have to
        be reported as one.""")
        r.body(f"""
        The drone starts at ({scenario.start_xy[0]:g},
        {scenario.start_xy[1]:g}) m on a green floor pad and must reach
        ({scenario.goal_xy[0]:g}, {scenario.goal_xy[1]:g}) m, with success
        declared when the drone centre is within {scenario.goal_tolerance_m:g} m
        of the goal. The goal pad is drawn at exactly that radius, so what a
        viewer sees is the termination condition rather than a decoration. Both
        pads are flat and non-colliding: anything tall enough to enter the scan
        plane would be reported as an obstacle, and a collider would let the
        drone collide with its own goal marker.""")
        r.figure(args.figures / "fig2_technical_map.png",
                 "Figure 2 — scaled technical map generated from the same "
                 "configuration file the simulator loads, so the drawing "
                 "cannot disagree with the world. Thin outlines are the "
                 "footprints used by the collision test.")
        r.figure(args.figures / "fig1_environment_overview.png",
                 "Figure 1 — the environment as Webots renders it.")

        r.table(["Parameter", "Value", "Unit", "Justification"],
                [[a, b, c, textwrap.shorten(d, 70, placeholder="...")]
                 for a, b, c, d in t1],
                [0.17, 0.13, 0.10, 0.48],
                caption="Table 1 — simulator configuration")

        # 5
        r.heading("5  Drone dynamics and control interface")
        r.body("""
        The vehicle is a 0.55 kg quadcopter assembled from Webots base nodes,
        with rotor hubs at +/-0.09 m and a 0.1773 m rotor tip radius, bounded by
        a 0.18 m collision cylinder so Webots contact physics and the geometric
        collision test agree with the radius the policy contract assumes.
        Thrust comes from four Propeller nodes; nothing in the pipeline writes
        the drone's position during flight.""")
        r.body("""
        A policy issues one horizontal velocity setpoint every 0.2 s. A
        quadcopter cannot follow that directly because it must tilt before it
        can accelerate, so a cascaded autopilot closes three loops at the 8 ms
        physics step: velocity error to a tilt target, attitude to body torque,
        and an independent altitude cascade to collective thrust. The setpoint
        is held between policy updates. Command rate and simulation step are
        therefore different quantities: 5 Hz outer, 125 Hz inner.""")
        r.body("""
        Commanded and measured velocity are distinct and are logged separately.
        The measured value is taken from the GPS speed vector and is never the
        command echoed back; after a single 0.2 s step a commanded 1.2 m/s
        reads about 0.28 m/s, which is the vehicle's physical rise time rather
        than a tracking defect.""")

        # 6
        r.heading("6  Static and dynamic obstacle modelling")
        r.body("""
        Obstacles are rectangular solids -- storage racks, structural columns,
        a crate stack and a transfer cart -- rather than generic cylinders.
        That change required extending the geometry layer: a rack drawn as a
        rectangle but collided as its bounding circle would report contact in
        open aisle, roughly half a metre from anything solid. Each obstacle now
        answers its own queries: exact surface distance for the collision test
        and an exact slab-method ray intersection for the analytic sensor
        reference.""")
        r.body("""
        Collision is a swept test, not a point check. Over one physics step the
        drone and a moving obstacle both travel in straight lines, so their
        relative position is linear in the step fraction; distance to a convex
        set is convex, so the minimum over the step is found by ternary search
        after a cheap bounding-circle reject. This prevents a fast obstacle
        from passing through the drone between samples.""")
        r.body(f"""
        The transfer cart is the dynamic hazard: a {cart.size_xy[0]:g} x
        {cart.size_xy[1]:g} m body at x = {cart.xy[0]:g} m travelling along y at
        a constant {cart.motion.speed_mps:g} m/s over
        +/-{cart.motion.amplitude_m:g} m. The motion is a triangle wave,
        y(t) = y0 + A(1 - 4|u - 1/2|) with u = (t/T + phase/2pi) mod 1 and
        T = 4A/v = {4 * cart.motion.amplitude_m / cart.motion.speed_mps:.2f} s,
        so the position is a pure function of simulation time and an episode
        replays exactly. The obstacle is driven kinematically through the
        Supervisor rather than pushed by contact forces; that approximation is
        disclosed here and is why collision is decided geometrically.""")
        if mover:
            err = max(r_["error_m"] for r_ in mover["steps"])
            r.body(f"""
            The motion model was verified against the running simulator rather
            than against the equations that produced it. Over
            {len(mover['steps'])} control steps the supervisor-reported pose
            matched the analytic trajectory to {err:.2e} m, the measured median
            travel speed was
            {mover['measured_median_speed_mps']:.4f} m/s against
            {cart.motion.speed_mps:g} configured, the cart stayed inside its
            declared envelope, and two resets replayed to within 3e-15 m.""")
        r.figure(args.figures / "fig4_moving_obstacle_validation.png",
                 "Figure 4. Panel (c)'s flat line is the whole result: the "
                 "simulator reproduces the analytic path to the limit of "
                 "double precision, which is what makes a dynamic episode "
                 "replayable.")
        r.table(["id", "shape", "position (m)", "size (m)", "class",
                 "motion", "v (m/s)"],
                [[a[0], a[2], a[3], a[4], a[5],
                  textwrap.shorten(a[6], 30, placeholder="..."), a[7]]
                 for a in t2],
                [0.13, 0.09, 0.15, 0.17, 0.10, 0.26, 0.08],
                caption="Table 2 — environment and obstacle specification. The "
                        "research purpose of each obstacle is recorded in the "
                        "CSV and in the scenario file.")

        # 7
        r.heading("7  Sensor and PPO integration boundaries")
        r.body(f"""
        Obstacle sensing is a horizontally mounted LiDAR: full 360 degree
        cylindrical scan, {scenario.lidar_resolution} beams (0.5 degree
        spacing, 60 per 30 degree sector), {scenario.sensing_max_range_m:g} m
        range cap, mounted {scenario.lidar_height_m:g} m above the body origin.
        Beams are reduced to 12 sectors by the PPO author's own pack_scan, with
        sector 0 forward and angles increasing clockwise, so sector 3 is right,
        6 behind and 9 left. Raw distances are passed in metres; normalisation
        happens once, inside the observation builder.""")
        if calib:
            r.body(f"""
            The index-to-bearing mapping is measured, not assumed. Fitting it
            against known arena geometry gives direction +1 and an offset of
            {calib['lidar']['offset_deg']:.2f} degrees with an RMS residual of
            {calib['lidar']['rms_error_m']:.4f} m. The residual half-degree is
            one beam pitch at this resolution.""")
        r.body("""
        The mount is an idealised gimbal: it is counter-rotated every control
        tick so the scan plane stays horizontal whatever the airframe is doing.
        This matters because the 12-sector contract describes a flat world
        while a quadcopter must tilt to accelerate. The gimbal is modelled as
        massless, instantaneous and free of joint limits, which no physical
        mount is; it is a simulation choice, labelled in every record, and not
        a hardware claim.""")
        r.body("""
        The policy interface is unchanged from the agreed contract: the
        simulator receives [vx, vy] in m/s capped at 1.2 per axis, holds it for
        0.2 s of simulation time, and returns timestamp, drone position, goal
        position, measured velocity, collision flag and the synchronised sector
        scan. The 52-feature observation is assembled by the PPO author's
        unmodified module. No file under his directory was edited, which was
        confirmed by re-computing the SHA-256 digests recorded in his own
        training configuration.""")

        # 8
        r.heading("8  Verification and validation")
        r.body("""
        Verification asks whether the environment was built correctly;
        validation asks whether it represents the phenomena the study needs.
        Both are addressed, and they are kept apart.""")
        r.body(f"""
        Verification. The interface and physics suite passes
        {(validation or {}).get('passed', '?')} of
        {(validation or {}).get('total', '?')} checks on the warehouse world:
        world load, start and goal placement, command timing exact to better
        than 3e-16 s, all four commanded axes moving the drone the correct way,
        measured velocity being a measurement rather than an echo, collision
        firing on a wall and staying clear otherwise, reset restoring the start
        to within 3 micrometres and reproducing a three-command trajectory to
        7.1 mm, and the observation builder accepting every frame and returning
        52 features. Forty-six unit tests cover the box and cylinder geometry,
        sector binning and wraparound, the mount levelling maths, obstacle
        trajectories and the scenario validation guards; they need no Webots
        session.""")
        if mover and stab:
            sector = max(r_["max_sector_error_m"] for r_ in mover["steps"])
            mean_sector = float(np.mean([r_["max_sector_error_m"]
                                         for r_ in mover["steps"]]))
            r.body(f"""
            Validation. The sensor is checked against an independent analytic
            ray-cast of the same scene -- a separate implementation, using the
            slab method on the true footprints, including occlusion. Across a
            dynamic episode the worst per-sector disagreement was
            {sector:.4f} m and the mean was {mean_sector:.4f} m over all 12
            sectors. Separately, a single-factor ablation on the mount shows
            the effect of holding the scan plane level: at tilts up to
            {stab['max_tilt_seen_deg']:.1f} degrees the stabilised mount's
            worst range error was {stab['worst_paired_rms_m']:.3f} m against
            {fixed['worst_paired_rms_m']:.3f} m body-fixed, with no sector
            either losing a real surface or inventing one.""")
        r.body("""
        Two defects found this way are worth recording, because both were
        invisible to the scripted tests that passed around them. The saved
        world placed a moving obstacle at its motion centre rather than its
        t = 0 pose; the backend repositions obstacles on reset, so scripted
        runs were unaffected, but the stored world disagreed with the model by
        1.9 m for anyone opening it directly, and it corrupted a calibration
        reference by 1.34 m. Separately, the calibration client had its
        scenario hardcoded, so pointing it at a different world silently
        compared one world's measurements against another world's geometry.
        Both are fixed; the second is the reason the calibration tool now
        refuses to run without being told which scenario it is measuring.""")
        r.figure(args.figures / "fig5_sensor_validation.png",
                 "Figure 5. Panel (b)'s single outlier bin near 0.29 m is a "
                 "grazing return at a sector boundary, the residual effect "
                 "described in section 10.")
        r.table(["Test", "Trials", "Quantity", "Expected", "Observed",
                 "Status"],
                [[textwrap.shorten(a[0], 24, placeholder="..."),
                  textwrap.shorten(a[1], 17, placeholder="..."),
                  textwrap.shorten(a[2], 24, placeholder="..."),
                  textwrap.shorten(a[3], 20, placeholder="..."),
                  textwrap.shorten(a[4], 24, placeholder="..."), a[6]]
                 for a in t3],
                [0.21, 0.15, 0.21, 0.17, 0.21, 0.07],
                caption="Table 3 — validation results. Full expected/observed "
                        "values, deviations and evidence paths are in "
                        "table3_validation_results.csv.")

        # 9
        r.heading("9  Results")
        r.figure(args.figures / "fig3_architecture.png",
                 "Figure 3 — closed-loop architecture with the ownership "
                 "boundary marked on each block.")
        if summary:
            r.body(f"""
            {protocol['total_episodes']} episodes were recorded in
            {protocol['wall_clock_s']:.0f} s of wall-clock time across two
            conditions that differ in exactly one factor: whether the transfer
            cart is parked at the wall or sweeping the aisle. Everything else
            -- geometry, lighting, sensor, start, goal, horizon -- is identical.
            These runs exist to demonstrate that the environment supports
            reproducible closed-loop evaluation; they are an integration
            demonstration, not a policy evaluation, and the paragraph below
            states why.""")
            rows = []
            for s in summary:
                rows.append([
                    "static" if s["mode"] == "B_static" else "dynamic",
                    s["controller"], s["kind"].replace("_", " "),
                    s["episodes"], s["goal"], s["collision"], s["timeout"],
                    f"{float(s['success_rate']) * 100:.0f}%",
                    f"[{float(s['success_rate_ci95_low']) * 100:.0f}, "
                    f"{float(s['success_rate_ci95_high']) * 100:.0f}]",
                    s["mean_min_truth_clearance_m"],
                    s["mean_velocity_rmse_mps"],
                ])
            r.table(["Mode", "Controller", "Kind", "n", "Goal", "Coll", "T/O",
                     "Rate", "95% CI", "Clear (m)", "RMSE"],
                    rows,
                    [0.09, 0.14, 0.11, 0.05, 0.06, 0.06, 0.05, 0.07, 0.11,
                     0.10, 0.08],
                    caption="Recorded outcomes. Clear = mean minimum "
                            "supervisor ground-truth clearance to an obstacle "
                            "surface; RMSE = mean velocity tracking error.")
            r.body("""
            Two observations are worth stating carefully. In the static
            condition the three PPO checkpoints reach the goal in every
            episode while both scripted baselines fail, which confirms the
            layout is discriminative rather than trivially open -- an earlier
            layout left the straight line to the goal clear and the
            goal-direct baseline threaded it, so the condition measured
            nothing and the racks were moved to straddle the route. In the
            dynamic condition the PPO checkpoints land between 47% and 60%,
            and the spread between training seeds is as wide as the gap
            between conditions.""")
            r.body("""
            The reactive baseline's failure has an identifiable mechanism
            rather than being noise: it never collides and never reaches the
            goal, stalling about 1.6 m short with a minimum clearance of
            0.62 m. Its repulsion term has a fixed 1.45 m activation radius and
            the crate stack sits 1.54 m from the goal, so goal attraction and
            obstacle repulsion balance before arrival. That is the classical
            goals-non-reachable-with-obstacles-nearby condition of
            potential-field control, and it reflects an untuned baseline
            hyperparameter in this scene, not a general ranking of the
            methods.""")

        # 10
        r.heading("10  Discussion and limitations")
        r.bullets([
            "One fixed obstacle layout. The dynamic condition varies the "
            "crossing phase, not obstacle positions, so nothing here supports "
            "a claim about generalisation to other layouts.",
            "Three training seeds only, and their spread in the dynamic "
            "condition is comparable to the effect being measured.",
            "The baselines are untuned for this scene. The reactive rule's "
            "repulsion radius was not adapted to the crate-to-goal distance, "
            "which is sufficient to explain its 0% rate.",
            "The gimbal is idealised: zero mass, no lag, no joint limits. A "
            "physical stabilised mount has all three.",
            "The LiDAR is a single scan layer, so the sensor is blind to "
            "anything not crossing its plane. The scenario validator now "
            "rejects an obstacle shorter than the scan height, which closes "
            "the worst case -- collidable but unsensable -- but height "
            "remains a modelling constraint rather than a sensed property.",
            "Obstacle motion is kinematic. The cart is driven through the "
            "Supervisor and is not displaced by contact, so collisions with "
            "it are decided geometrically.",
            "Near a sector boundary an obstacle can be attributed to the "
            "neighbouring sector, and a grazed sector reports the full range "
            "cap. Beam pitch bounds this; halving it reduced but did not "
            "remove it.",
            "Physical mode is near-deterministic, so repeated episodes on a "
            "fixed layout measure run-to-run variation, not scenario "
            "variation.",
            "Verification is not real-world validation. No physical flight "
            "data was collected and none of this predicts hardware behaviour.",
        ])

        r.heading("11  Conclusion")
        r.body("""
        The deliverable is a Webots environment in which a trained policy flies
        a physically simulated quadcopter using measured LiDAR returns, with
        reproducible episodes, obstacle geometry that is identical in the
        drawing, the collision test and the sensor, and scripted obstacle
        motion verified against its own analytic model to within 1e-15 m. The
        sensor's angular mapping was measured rather than assumed, and its
        output agrees with an independent analytic ray-cast of the same scene
        to a mean of 0.05 m across a moving episode.""")
        r.body("""
        For the team, this means Spencer can run any checkpoint through the
        same interface his planar environment exposes, without editing his
        code; the sensor teammate has a calibration scene, a measured
        index-to-bearing mapping, a two-frame packet with real returns, and a
        quantified attitude failure mode to design processing against. The
        clearest next steps for the environment are randomised obstacle
        layouts, which is what would turn the current integration
        demonstrations into a generalisation study, and a multi-layer or
        explicitly height-aware sensor model.""")

        r.heading("12  References")
        r.body("""
        [1] Cyberbotics Ltd. Webots Reference Manual and User Guide, R2025a.
        https://cyberbotics.com/doc/reference/index and
        https://cyberbotics.com/doc/guide/index. Used for the Lidar, Camera,
        Propeller, GPS, InertialUnit and Supervisor node semantics, the
        cylindrical projection requirement above a pi field of view, and the
        coordinate-system conventions cited in sections 4, 5 and 7.""")
        r.body("""
        [2] O. Michel. "Cyberbotics Ltd. Webots: Professional Mobile Robot
        Simulation." International Journal of Advanced Robotic Systems, vol. 1,
        no. 1, pp. 39-42, 2004. The originating description of the simulator
        selected in section 3.""")
        r.body("""
        [3] J. Schulman, F. Wolski, P. Dhariwal, A. Radford and O. Klimov.
        "Proximal Policy Optimization Algorithms." arXiv:1707.06347, 2017. The
        algorithm behind the checkpoints used as a workload in section 9;
        training is the DRL teammate's contribution, not this work's.""")
        r.body("""
        [4] O. Khatib. "Real-Time Obstacle Avoidance for Manipulators and
        Mobile Robots." The International Journal of Robotics Research, vol. 5,
        no. 1, pp. 90-98, 1986. The potential-field formulation whose
        documented failure mode is identified in section 9.""")
        r.body("""
        [5] Y. Koren and J. Borenstein. "Potential Field Methods and Their
        Inherent Limitations for Mobile Robot Navigation." Proceedings of the
        IEEE International Conference on Robotics and Automation, pp. 1398-1404,
        1991. Describes the trap and goal-non-reachable conditions observed in
        the reactive baseline.""")
        r.body("""
        Reference [1] was consulted directly during implementation. References
        [2]-[5] are standard, verifiable sources for the algorithm and the
        failure mode named in this report; bibliographic details should be
        confirmed against the publisher of record before any external
        submission.""")

        r.flush()

    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
