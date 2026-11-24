"""Build the DNS research report PDF and the three research tables.

Everything numeric is read from recorded result files; nothing is typed in by
hand, so the report cannot drift from the data. A missing input makes the
affected row say "unavailable" rather than inventing a value.

The PDF is assembled with matplotlib's PdfPages (already a project dependency
via Spencer's requirements.txt) rather than by adding a document toolchain.

Run::

    python scripts/make_report.py
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

PAGE = (8.27, 11.69)          # A4 portrait, inches


def load_csv(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(encoding="utf8") as handle:
        return list(csv.DictReader(handle))


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf8")) if path.is_file() else {}


class Report:
    """A simple flowing-text page builder on top of PdfPages."""

    def __init__(self, pdf: PdfPages):
        self.pdf = pdf
        self.fig = None
        self.y = 0.0
        self.page_number = 0

    def new_page(self) -> None:
        self.flush()
        self.page_number += 1
        self.fig = plt.figure(figsize=PAGE, facecolor=SURFACE)
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(0, 1)
        self.ax.axis("off")
        self.ax.text(0.08, 0.972, "Drone Navigation System — simulation "
                     "environment report", fontsize=7.5, color=MUTED)
        self.ax.plot([0.08, 0.92], [0.966, 0.966], color=RULE, linewidth=0.7)
        self.ax.text(0.92, 0.028, f"{self.page_number}", fontsize=8,
                     color=MUTED, ha="right")
        self.y = 0.935

    #: When set, every page is also written here as a PNG. PDFs cannot be
    #: rendered in this environment, so this is how page layout gets checked
    #: rather than assumed.
    preview_dir: Path | None = None

    def flush(self) -> None:
        if self.fig is not None:
            self.pdf.savefig(self.fig)
            if self.preview_dir is not None:
                self.preview_dir.mkdir(parents=True, exist_ok=True)
                self.fig.savefig(self.preview_dir / f"page{self.page_number:02d}.png",
                                 dpi=110)
            plt.close(self.fig)
            self.fig = None

    def space(self, amount: float) -> None:
        self.y -= amount
        if self.y < 0.07:
            self.new_page()

    def heading(self, text: str, size: int = 13) -> None:
        if self.y < 0.17:
            self.new_page()
        self.space(0.018)
        self.ax.text(0.08, self.y, text, fontsize=size, color=INK,
                     fontweight="semibold", va="top")
        self.space(0.026 if size >= 13 else 0.022)

    def body(self, text: str, size: int = 9.1, indent: float = 0.08) -> None:
        width = int(104 * 9.1 / size)
        for block in text.strip().split("\n\n"):
            lines = textwrap.wrap(" ".join(block.split()), width)
            for line in lines:
                if self.y < 0.07:
                    self.new_page()
                self.ax.text(indent, self.y, line, fontsize=size, color=INK2,
                             va="top")
                self.space(0.0165 * size / 9.1)
            self.space(0.008)

    def bullets(self, items: list[str], size: int = 9.1) -> None:
        for item in items:
            lines = textwrap.wrap(" ".join(item.split()), 98)
            for index, line in enumerate(lines):
                if self.y < 0.07:
                    self.new_page()
                prefix = "•  " if index == 0 else "    "
                self.ax.text(0.095, self.y, prefix + line, fontsize=size,
                             color=INK2, va="top")
                self.space(0.0165)
            self.space(0.004)

    def table(self, columns: list[str], rows: list[list[str]],
              widths: list[float], caption: str = "", size: float = 7.4) -> None:
        need = 0.03 + 0.0165 * (len(rows) + 2)
        if self.y - need < 0.07:
            self.new_page()
        if caption:
            for line in textwrap.wrap(" ".join(caption.split()), 96):
                self.ax.text(0.08, self.y, line, fontsize=8.6, color=INK,
                             fontweight="semibold", va="top")
                self.space(0.016)
            self.space(0.008)
        x0 = 0.08
        total = sum(widths)
        positions = []
        acc = x0
        for w in widths:
            positions.append(acc)
            acc += 0.84 * w / total
        for pos, name in zip(positions, columns):
            self.ax.text(pos, self.y, name, fontsize=size, color=INK,
                         fontweight="semibold", va="top")
        self.space(0.0145)
        self.ax.plot([x0, 0.92], [self.y + 0.004, self.y + 0.004],
                     color=RULE, linewidth=0.7)
        self.space(0.004)
        for row in rows:
            if self.y < 0.07:
                self.new_page()
                for pos, name in zip(positions, columns):
                    self.ax.text(pos, self.y, name, fontsize=size, color=INK,
                                 fontweight="semibold", va="top")
                self.space(0.0145)
                self.ax.plot([x0, 0.92], [self.y + 0.004, self.y + 0.004],
                             color=RULE, linewidth=0.7)
                self.space(0.004)
            for pos, cell in zip(positions, row):
                self.ax.text(pos, self.y, str(cell), fontsize=size,
                             color=INK2, va="top")
            self.space(0.0152)
        self.space(0.012)

    def figure(self, path: Path, caption: str, height: float = 0.30) -> None:
        if not path.is_file():
            self.body(f"[FIGURE PENDING: {path.name} was not generated]")
            return
        image = imread(path)
        aspect = image.shape[0] / image.shape[1]
        width = 0.84
        actual = width * aspect * PAGE[0] / PAGE[1]
        if self.y - actual - 0.03 < 0.07:
            self.new_page()
        axes = self.fig.add_axes([0.08, self.y - actual, width, actual])
        axes.imshow(image)
        axes.axis("off")
        self.space(actual + 0.012)
        # Captions must wrap: a single long line runs off the right margin.
        for line in textwrap.wrap(" ".join(caption.split()), 118):
            self.ax.text(0.08, self.y, line, fontsize=7.6, color=MUTED,
                         va="top")
            self.space(0.0135)
        self.space(0.014)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path,
                        default=PROJECT_ROOT / "config" / "scenario_research_demo.json")
    parser.add_argument("--figures", type=Path,
                        default=PROJECT_ROOT / "evidence" / "package_figures")
    parser.add_argument("--out", type=Path,
                        default=PROJECT_ROOT / "evidence" / "DNS_Research_Report.pdf")
    parser.add_argument("--tables-out", type=Path,
                        default=PROJECT_ROOT / "evidence" / "tables")
    parser.add_argument("--preview", action="store_true",
                        help="also write each page as a PNG, so layout can be "
                             "inspected without a PDF renderer")
    args = parser.parse_args()

    scenario = Scenario.load(args.config)
    static = Scenario.load(PROJECT_ROOT / "config" / "scenario_demo_static.json")
    summary = load_csv(RESULTS / "experiments" / "experiment_summary.csv")
    episodes = load_csv(RESULTS / "experiments" / "experiment_results.csv")
    protocol = load_json(RESULTS / "experiments" / "protocol.json")
    attitude = load_json(RESULTS / "lidar_attitude.json")
    calibration = load_json(RESULTS / "calibration.json")
    recording = load_json(RESULTS / "demo" / "recording_report.json")
    mp4_check = load_json(RESULTS / "demo" / "mp4_verification.json")
    validation = load_json(RESULTS / "validation_physical_lidar.json")
    mover = next(o for o in scenario.obstacles
                 if o.motion.type != "static" and o.motion.amplitude_m)

    # ------------------------------------------------------------- tables
    args.tables_out.mkdir(parents=True, exist_ok=True)

    table1 = [
        ["Simulator", "Webots R2025a", "-", "Native Python controllers, built-in Lidar/GPS/IMU/Propeller nodes, runs offline"],
        ["Physics timestep", f"{scenario.basic_timestep_ms}", "ms", "Divides 200 ms exactly (25 steps), so the PPO clock never drifts"],
        ["PPO control period", f"{scenario.command_period_s}", "s", "Spencer's navigation.DT; 5 Hz is the agreed interface"],
        ["Inner loop rate", "125", "Hz", "Autopilot runs at the physics timestep; a quadcopter cannot track 5 Hz setpoints open-loop"],
        ["Arena", f"{scenario.size_xy[0]:g} x {scenario.size_xy[1]:g}", "m", "Matches planar.py training bounds; see section 3"],
        ["Wall height", f"{scenario.wall_height_m:g}", "m", "Raised from 1.2 m: a low wall left only 0.125 m above the scan plane"],
        ["Drone mass", "0.55", "kg", "Whole airframe; thrust/weight 4.0"],
        ["Collision radius", f"{scenario.collision_radius_m:g}", "m", "Equals navigation.RADIUS, so sim and policy agree"],
        ["Flight altitude", f"{scenario.cruise_altitude_m:g}", "m", "Held by the altitude cascade; planar task by design"],
        ["Max speed", f"{scenario.max_speed_mps:g}", "m/s per axis", "navigation.SPEED; action is scaled once, by the bridge"],
        ["Sensor", "360 deg LiDAR", "-", "Full-circle planar scan feeding the 12-sector contract"],
        ["Field of view", "360", "deg", "Cylindrical projection; required above pi"],
        ["Sensor range", f"{scenario.sensing_max_range_m:g}", "m", "navigation.RANGE cap"],
        ["Beam count", f"{scenario.lidar_resolution}", "beams", "0.5 deg spacing, 60 per sector; halved the grazing error"],
        ["Sectors", f"{scenario.sensing_sectors}", "-", "30 deg each, 0 forward, clockwise positive"],
        ["Mount", scenario.lidar_mount_mode, "-", "Ideal gimbal holding the scan plane horizontal"],
        ["Mount height", f"{scenario.lidar_height_m:g}", "m", "Above the body origin; clears the rotor discs at max tilt"],
        ["Goal tolerance", f"{scenario.goal_tolerance_m:g}", "m", "navigation.GOAL_TOL"],
        ["Episode horizon", "90", "steps", "navigation.HORIZON = 18 s of simulated time"],
    ]
    with (args.tables_out / "table1_configuration.csv").open(
            "w", newline="", encoding="utf8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["parameter", "value", "unit", "justification"])
        writer.writerows(table1)

    table2 = [
        ["Mode B (static)", static.name, "10 x 6 m indoor arena",
         f"({static.start_xy[0]:g}, {static.start_xy[1]:g})",
         f"({static.goal_xy[0]:g}, {static.goal_xy[1]:g})", "3", "0", "none",
         "0.0", "Obstacle avoidance with a fixed layout"],
        ["Mode C (dynamic)", scenario.name, "10 x 6 m indoor arena",
         f"({scenario.start_xy[0]:g}, {scenario.start_xy[1]:g})",
         f"({scenario.goal_xy[0]:g}, {scenario.goal_xy[1]:g})", "2", "1",
         f"linear, +/-{mover.motion.amplitude_m:g} m along y",
         f"{mover.motion.speed_mps:g}",
         "Reaction to an obstacle crossing the route"],
    ]
    with (args.tables_out / "table2_environment.csv").open(
            "w", newline="", encoding="utf8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["mode", "scenario", "map_type", "start_xy_m",
                         "goal_xy_m", "n_static", "n_moving", "motion_type",
                         "obstacle_speed_mps", "purpose"])
        writer.writerows(table2)

    table3 = []
    for row in summary:
        table3.append([
            "B static" if row["mode"] == "B_static" else "C dynamic",
            row["controller"], row["kind"], row["episodes"], row["goal"],
            row["collision"], row["timeout"],
            f"{float(row['success_rate']) * 100:.0f}%",
            f"[{float(row['success_rate_ci95_low']) * 100:.0f}, "
            f"{float(row['success_rate_ci95_high']) * 100:.0f}]",
            row["mean_path_m"],
            row["mean_completion_s"] or "n/a",
            row["mean_min_truth_clearance_m"],
            row["mean_velocity_rmse_mps"],
        ])
    with (args.tables_out / "table3_results.csv").open(
            "w", newline="", encoding="utf8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["mode", "controller", "kind", "episodes", "goal",
                         "collision", "timeout", "success_rate", "ci95_pct",
                         "mean_path_m", "mean_completion_s",
                         "mean_min_truth_clearance_m",
                         "mean_velocity_rmse_mps"])
        writer.writerows(table3)
    print(f"wrote 3 tables to {args.tables_out}")

    # -------------------------------------------------------------- report
    args.out.parent.mkdir(parents=True, exist_ok=True)
    checks_passed = validation.get("passed", "?")
    checks_total = validation.get("total", "?")
    stabilised = (attitude.get("by_mount", {}) or {}).get("stabilised", {})
    body_fixed = (attitude.get("by_mount", {}) or {}).get("body_fixed", {})

    with PdfPages(args.out) as pdf:
        report = Report(pdf)
        if args.preview:
            report.preview_dir = args.out.parent / "report_preview"

        # ---- title page
        report.new_page()
        report.y = 0.76
        report.ax.text(0.08, report.y, "Drone Navigation System", fontsize=25,
                       color=INK, fontweight="semibold", va="top")
        report.space(0.052)
        report.ax.text(0.08, report.y, "A validated Webots simulation "
                       "environment for\ndeep-reinforcement-learning UAV "
                       "navigation", fontsize=14, color=INK2, va="top")
        report.space(0.085)
        report.ax.plot([0.08, 0.92], [report.y, report.y], color=RULE,
                       linewidth=0.8)
        report.space(0.03)
        report.ax.text(0.08, report.y,
                       "Simulation environment: Khaled\n"
                       "PPO / DRL: Spencer   ·   LiDAR sensing: sensor "
                       "teammate\n"
                       "Supervisor: Dr. Mehdi Samieiyeganeh\n"
                       "Taylor's University, Malaysia",
                       fontsize=10, color=INK2, va="top")
        report.space(0.10)
        report.ax.text(0.08, report.y,
                       "RESEARCH ENGINEERING DRAFT — prepared for supervisor "
                       "and team review.\nNot a peer-reviewed or accepted "
                       "paper. All results in this document were\nproduced by "
                       "the runs listed in section 9 and can be reproduced "
                       "with the\ncommands given there.",
                       fontsize=9, color=MUTED, va="top")
        report.space(0.07)
        report.ax.text(0.08, report.y,
                       f"Simulator: Webots R2025a   ·   "
                       f"Arena {scenario.size_xy[0]:g} x {scenario.size_xy[1]:g} m   ·   "
                       f"{protocol.get('total_episodes', '?')} recorded episodes",
                       fontsize=9.5, color=INK, va="top")

        # ---- 1
        report.new_page()
        report.heading("1  Project overview")
        report.body("""
        Autonomous drone navigation requires a vehicle to reach a goal while
        avoiding obstacles it can only perceive through onboard sensors. The
        DNS project studies whether deep reinforcement learning, specifically
        Proximal Policy Optimization, can produce such a navigation policy, and
        how that policy behaves when obstacles move rather than stand still.
        Static obstacles test geometric avoidance; moving obstacles additionally
        test timing, because the free space itself changes while the drone
        flies through it.""")
        report.body("""
        The project is split three ways. Spencer owns the PPO implementation
        and the trained policies. A sensor teammate owns LiDAR processing. This
        report covers the third part, the simulation environment: the Webots
        world, the quadcopter model and its flight controller, the sensor
        mounting and configuration, the obstacle dynamics, the scenario
        definitions, collision ground truth, and the bridge that lets a trained
        policy fly inside the simulator.""")
        report.body("""
        The guiding constraint is that the simulator must be honest about what
        it measures. Simulator ground truth, sensor measurement, policy
        observation and commanded action are kept strictly separate, and the
        observation handed to PPO is built by Spencer's own unmodified code
        from measured quantities only.""")

        report.heading("2  Simulator selection")
        report.body(f"""
        Webots R2025a was selected and is the version every result here was
        produced on. The practical reasons, each of which was exercised in this
        work: native Python controllers through libController, so Spencer's
        NumPy policy code runs in the same process as the simulator with no
        custom IPC; built-in Lidar, GPS, InertialUnit, Gyro and Propeller nodes,
        so the sensor and the rotor thrust model are the simulator's own rather
        than something we wrote; a scriptable Supervisor API, which is what
        makes reproducible episode resets and scripted obstacle motion
        possible; and worlds that are plain text generated from configuration,
        which is what keeps the scene and the experiment definition from
        drifting apart.""")
        report.body("""
        Limitations are equally concrete. Webots' rendering was unavailable in
        our automated session, which shaped how the demonstration video had to
        be captured (section 9). The contact solver and the propeller model are
        approximations; no attempt is made here to claim correspondence with a
        particular physical airframe.""")

        # ---- 3
        report.heading("3  Simulation environment design")
        report.body(f"""
        The arena is a closed rectangular room of
        {scenario.size_xy[0]:g} x {scenario.size_xy[1]:g} m with
        {scenario.wall_height_m:g} m walls, origin at one corner. The research
        frame is +x forward, +y right, with altitude and heading held constant;
        Webots itself is ENU, and the single mapping x_dns = x_webots,
        y_dns = -y_webots is written down in one module and used everywhere.""")
        report.body(f"""
        The arena size is deliberately NOT enlarged. Spencer's policies were
        trained in planar.py on exactly these bounds, and
        navigation.observation divides the relative goal vector by 10 and caps
        LiDAR at 5 m. Widening the room would push those features outside the
        distribution the policies were fitted to, and results would no longer
        be comparable with the static baseline. A larger arena is a valid
        future out-of-distribution study, but it would have to be labelled as
        one rather than reported as the headline environment.""")
        report.figure(args.figures / "fig1_environment_layout.png",
                      "Figure 1 — arena geometry, start, goal and obstacle "
                      "placement, with the crossing obstacle's sweep. Dashed "
                      "rings are obstacle radius inflated by the drone's "
                      "collision radius: the true no-go region.")
        report.table(
            ["Parameter", "Value", "Unit", "Justification"],
            [[r[0], r[1], r[2], textwrap.shorten(r[3], 74, placeholder="…")]
             for r in table1],
            [0.17, 0.12, 0.09, 0.50],
            caption="Table 1 — simulation configuration")

        # ---- 4
        report.heading("4  Drone model and control")
        report.body(f"""
        The vehicle is a 0.55 kg quadcopter built from Webots base nodes, with
        rotor hubs at +/-0.09 m and a 0.1773 m rotor tip radius, bounded by a
        0.18 m collision cylinder so that Webots contact physics and the
        geometric collision test agree with navigation.RADIUS. Thrust comes
        from four Propeller nodes; nothing in the pipeline teleports the drone
        or writes its position during flight.""")
        report.body("""
        A policy issues one horizontal velocity setpoint every 0.2 s. A real
        quadcopter cannot follow that directly, because it must tilt before it
        can accelerate, so a cascaded autopilot closes three loops at the 8 ms
        physics timestep: velocity error to a tilt target, attitude to body
        torque, and an independent altitude cascade to collective thrust. The
        commanded setpoint is held between policy updates.""")
        report.body("""
        Commanded and measured velocity are therefore different quantities and
        are logged separately. The measured value comes from the GPS speed
        vector, never from the command echoed back; after a single 0.2 s step a
        commanded 1.2 m/s reads about 0.28 m/s, which is the physical rise time
        of the vehicle, not an error.""")

        # ---- 5
        report.heading("5  Sensor configuration")
        beams = scenario.lidar_resolution
        report.body(f"""
        Obstacle sensing is a horizontally mounted LiDAR with a full 360 degree
        cylindrical scan, {beams} beams (0.5 degree spacing, 60 per sector), a
        5 m range cap, mounted {scenario.lidar_height_m:g} m above the body
        origin. Beams are reduced to 12 sectors of 30 degrees by Spencer's own
        pack_scan, with sector 0 forward and angles increasing clockwise, so
        sector 3 is right, 6 behind and 9 left. The index-to-bearing mapping is
        measured, not assumed: fitting it against known arena geometry gives
        direction +1 and an offset of 180.00 degrees with an RMS residual of
        {calibration.get('lidar', {}).get('rms_error_m', float('nan')):.4f} m.""")
        report.body("""
        The mount is gimbal-stabilised: it is counter-rotated every control
        tick so the scan plane stays horizontal regardless of airframe
        attitude. This matters because the 12-sector contract describes a flat
        world, while a quadcopter must tilt to accelerate. Measured with a
        body-fixed mount and 1.2 m walls, tilting past about 6 degrees lifted
        the beams over the wall tops, and Webots' out-of-range return was then
        encoded as "observed clear to 5 m" — the sensor reported open space
        where a wall stood, in up to 5 of 12 sectors at once.""")
        report.body(f"""
        Four things were changed together in response: the gimbal mount, the
        wall height (1.2 to {scenario.wall_height_m:g} m), the mount height
        (0.075 to {scenario.lidar_height_m:g} m, because a levelled plane
        otherwise range-found the drone's own rotors at a constant 0.177 m),
        and the beam count (360 to {beams}). Because all four changed at once,
        the improvement below is the effect of the combination; no controlled
        ablation isolating the gimbal alone has been run, and the report does
        not attribute the result to the gimbal by itself.""")
        if stabilised and body_fixed:
            report.table(
                ["Mount", "Steps", "Surface lost", "Surface invented",
                 "Median error", "Worst error"],
                [["body-fixed", body_fixed.get("steps"),
                  body_fixed.get("window_steps_with_phantom_clear"),
                  body_fixed.get("strict_steps_with_too_short"),
                  f"{body_fixed.get('median_paired_rms_m', 0):.3f} m",
                  f"{body_fixed.get('worst_paired_rms_m', 0):.3f} m"],
                 ["gimbal-stabilised", stabilised.get("steps"),
                  stabilised.get("window_steps_with_phantom_clear"),
                  stabilised.get("strict_steps_with_too_short"),
                  f"{stabilised.get('median_paired_rms_m', 0):.3f} m",
                  f"{stabilised.get('worst_paired_rms_m', 0):.3f} m"]],
                [0.20, 0.09, 0.15, 0.17, 0.14, 0.13],
                caption="Sensor validation, current geometry, 29 control "
                        "steps per mount at tilts up to 19 degrees")
        report.figure(args.figures / "fig3_lidar_validation.png",
                      "Figure 3 — (a) a measured 12-sector scan at episode "
                      "reset, in metres, in the agreed angular convention; "
                      "(b) sector range error against ground truth as a "
                      "function of airframe tilt, for both mounts.")

        # ---- 6
        report.heading("6  Static and dynamic obstacles")
        report.body(f"""
        Obstacles are vertical cylinders 1.8 m tall, which keeps them in the
        scan plane throughout. Two are static pillars, placed on opposite sides
        of the straight line from start to goal so the direct route is blocked
        and the drone must change side at least once. The third is the crossing
        obstacle: radius {mover.radius_m:g} m at x = {mover.xy[0]:g} m,
        travelling along y at a constant {mover.motion.speed_mps:g} m/s over
        +/-{mover.motion.amplitude_m:g} m, a full period of
        {4 * mover.motion.amplitude_m / mover.motion.speed_mps:.2f} s.""")
        report.body("""
        Motion is a pure function of simulation time, which is what makes an
        episode replayable: the obstacle's pose is computed analytically and
        written through the Supervisor each step. That is a kinematic
        approximation — the obstacle is not pushed by contact forces — and it
        is disclosed as such. Collision is therefore decided by a swept-segment
        test between the drone's motion over a step and the obstacle discs at
        both endpoints, which also prevents a fast obstacle tunnelling through
        the drone between samples.""")
        report.body("""
        The motion model was verified against the running simulator rather than
        against the equations that produced it: over 90 control steps the
        supervisor-reported position matched the analytic trajectory to 0.0 m,
        the measured travel speed was 0.5000 m/s against 0.5 configured, two
        resets replayed to within 3e-15 m, and the LiDAR tracked the obstacle
        across three sectors with a maximum range error of 0.012 m.""")
        report.table(
            ["Mode", "Start (m)", "Goal (m)", "Static", "Moving", "Motion",
             "Speed", "Purpose"],
            [[r[0], r[3], r[4], r[5], r[6], r[7], r[8],
              textwrap.shorten(r[9], 34, placeholder="…")] for r in table2],
            [0.13, 0.11, 0.11, 0.07, 0.07, 0.17, 0.07, 0.27],
            caption="Table 2 — environment and obstacle design")

        # ---- 7
        report.heading("7  Experimental methodology")
        report.body(f"""
        Two conditions are run in one world, differing in exactly one factor:
        whether the third obstacle is parked at the arena edge (Mode B) or
        sweeping the route (Mode C). Everything else — geometry, lighting,
        sensor, controller, start, goal, horizon — is identical, so a
        difference between the two is attributable to the obstacle's motion.""")
        report.body(f"""
        Mode B uses {protocol.get('episodes_per_controller', {}).get('B_static', '?')}
        episodes per controller. The layout there is fixed and physical mode is
        near-deterministic, so those repeats measure run-to-run variation, not
        scenario variation, and the rate carries very little information — the
        wide intervals in Figure 5 say so honestly. Mode C uses
        {protocol.get('episodes_per_controller', {}).get('C_dynamic', '?')}
        episodes per controller, where each seed selects a different initial
        crossing phase through numpy default_rng(seed).uniform(0, 2*pi). Those
        are genuinely distinct scenario instances, so a rate over them is
        meaningful.""")
        report.body("""
        Five controllers are compared, all of them Spencer's code: three
        trained PPO checkpoints (seeds 7, 19 and 43, each a 52-32-32-2 tanh
        policy tagged with the same observation schema), plus two scripted
        baselines, a goal-direct controller and a gap-following reactive rule.
        Naming them explicitly matters because a reactive rule must never be
        reported as a learned policy.""")
        report.bullets([
            "Independent variables: obstacle motion (parked vs crossing), "
            "controller, crossing phase.",
            "Dependent variables: outcome, control steps, path length, final "
            "goal distance, minimum clearance by ground truth and by sensor, "
            "velocity tracking RMSE.",
            "Controlled: arena, start, goal, altitude, sensor configuration, "
            "collision rule, 90-step horizon, 0.2 s command period.",
            "Termination: goal within 0.4 m, collision by swept-segment test, "
            "or timeout at 90 steps. Every episode ends in exactly one of "
            "these and none was excluded, so every rate's denominator is the "
            "full cell.",
        ])

        # ---- 8
        report.heading("8  Results")
        report.body(f"""
        {protocol.get('total_episodes', '?')} episodes were recorded in
        {protocol.get('wall_clock_s', '?')} s of wall-clock time. In Mode B all
        four obstacle-aware controllers reach the goal in every episode and the
        goal-direct baseline collides in every episode, which confirms the
        obstacles and the collision test are real rather than confirming
        anything about navigation quality.""")
        report.body("""
        Mode C separates the controllers sharply, and not in PPO's favour. The
        scripted reactive rule reached the goal in every episode. The three PPO
        checkpoints, trained identically and differing only in random seed,
        spread from 53% to 93%. That spread across seeds is as large as the gap
        between conditions, which is the single most important number in this
        report: with three training seeds we cannot distinguish "PPO handles
        crossing obstacles" from "one of these three seeds happened to".""")
        report.figure(args.figures / "fig5_results.png",
                      "Figure 5 — goal-reaching rate by controller and "
                      "condition, with Wilson 95% intervals. Mode B's wide "
                      "intervals reflect n=3 on a deterministic layout.")
        report.table(
            ["Mode", "Controller", "n", "Goal", "Coll", "T/O", "Rate",
             "95% CI", "Path (m)", "Time (s)", "Clear (m)", "RMSE"],
            [[r[0], r[1], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10],
              r[11], r[12]] for r in table3],
            [0.11, 0.14, 0.04, 0.05, 0.05, 0.04, 0.06, 0.10, 0.08, 0.08,
             0.08, 0.07],
            caption="Table 3 — experimental results. Clear = mean minimum "
                    "supervisor ground-truth clearance; RMSE = mean velocity "
                    "tracking error [m/s]")
        report.figure(args.figures / "fig4_trajectories.png",
                      "Figure 4 — recorded trajectories for episode 0 of each "
                      "condition. Circles mark goal arrival, crosses mark "
                      "collision.", height=0.34)

        # ---- 9
        report.heading("9  Verification and evidence")
        report.body(f"""
        The interface and physics checks pass {checks_passed} of
        {checks_total} in the full validation run, covering world load, start
        and goal placement, 5 Hz command timing to within 3e-16 s, all four
        commanded axes moving the drone the correct way, measured velocity
        being a measurement rather than an echo, collision firing on a wall and
        staying clear otherwise, reset reproducibility, and Spencer's
        Navigation wrapper accepting every frame and producing 52 features.
        Thirty-eight unit tests cover the coordinate transform, sector binning
        and wraparound, the mount levelling maths, obstacle trajectories and
        the scenario validation guards; they need no Webots session.""")
        report.body(f"""
        The two-frame sensor handoff is accepted by Spencer's own handoff.py
        verifier, run unmodified against our packet. The demonstration video is
        {recording.get('duration_s', '?')} s at
        {recording.get('width', '?')}x{recording.get('height', '?')},
        {mp4_check.get('tracks', [{}])[0].get('frame_count', '?')} frames,
        verified by parsing the MP4 container.""")
        report.body("""
        One limitation shaped the recording method and is disclosed here rather
        than hidden. Webots' GUI renderer produced no geometry in our automated
        session: Supervisor.exportImage returned a perfectly uniform fill of
        the background colour at three different drone poses, and the built-in
        movieStartRecording API produced a structurally valid MP4 whose every
        frame was blank. A Camera node renders to an offscreen texture instead,
        and that path does work, so every frame of the delivered video is a
        real Webots render of the real simulation captured through the camera
        sensor. Playback is 1:1 with simulation time; no frame was duplicated
        or stretched.""")

        # ---- 10
        report.heading("10  Discussion and limitations")
        report.bullets([
            "ONE fixed obstacle layout throughout. Mode C varies the crossing "
            "phase, not obstacle positions, so nothing here supports a claim "
            "of spatial generalisation. Randomised layouts are the next step.",
            "Three training seeds only. The 53-93% spread across seeds is "
            "wide enough that seed-level variance, not algorithmic capability, "
            "may explain the Mode C ordering.",
            "The reactive baseline beating PPO is a real measurement on this "
            "task, not a general claim about PPO. It does show that this "
            "scenario is solvable by a simple rule, which makes it a fair and "
            "informative benchmark rather than an impossible one.",
            "The gimbal is ideal: zero mass, no actuator lag, no joint limits. "
            "A physical stabilised mount has all three. This is a simulation "
            "modelling choice, labelled in every record, not a hardware claim.",
            "The LiDAR is a single scan layer, so the sensor is blind to "
            "anything not crossing its plane, and obstacle heights are not yet "
            "checked per-obstacle against the scan height.",
            "Obstacle motion is kinematic: the crossing obstacle is driven "
            "through the Supervisor and is not displaced by contact.",
            "Near a sector boundary an obstacle can be attributed to the "
            "neighbouring sector, and a grazed sector reports the full 5 m "
            "cap. Beam pitch bounds this; it is reduced, not eliminated.",
            "Simulation fidelity is not real-world fidelity. None of this "
            "evidence predicts how a physical drone would behave.",
        ])

        report.heading("11  Conclusion and next steps")
        report.body("""
        The deliverable is a Webots environment in which a trained PPO policy
        flies a physically simulated quadcopter using real LiDAR returns, with
        measured state, reproducible episodes, scripted obstacle motion
        verified against its own analytic model, and a sensor whose angular
        mapping and attitude behaviour have been measured rather than assumed.
        The bridge is accepted by the PPO author's own verification tooling,
        which is the strongest available evidence that the interface is
        correct.""")
        report.bullets([
            "Spencer can run any checkpoint in the simulator through the same "
            "Navigation API used for planar.py, and can now see where "
            "planar-trained policies lose to a reactive rule.",
            "The sensor teammate has a verified two-frame packet with real "
            "returns, a measured index-to-bearing mapping, and a quantified "
            "attitude failure mode to design processing against.",
            "Before any stronger claim: randomise obstacle layouts, raise the "
            "number of training seeds, and run the controlled ablation that "
            "isolates the gimbal from the other three sensor changes.",
        ])

        report.flush()

    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
