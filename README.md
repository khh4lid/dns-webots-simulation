# DNS — Webots Simulation Environment

Drone-navigation simulation built in **Webots R2025a**, with the evidence behind
every number reported from it.

A simulated quadcopter must fly from a start point to a goal point across an
indoor warehouse aisle without hitting anything. It perceives the world only
through a LiDAR mounted on it — it is never told where the obstacles are. Some
obstacles are fixed; one is a powered cart that crosses its route.

---

## Scope of this repository

The Drone Navigation System (DNS) research project — Taylor's University,
supervised by Dr. Mehdi Samieiyeganeh — has three parts. **This repository is
the simulation environment**, and only that:

| Owned here | Owned by teammates |
|---|---|
| Webots world generation and the arena geometry | PPO algorithm and trained controllers (Spencer) |
| The quadcopter PROTO and its cascaded autopilot | Raw LiDAR signal *processing* |
| Sensor *mounting and configuration* | |
| Obstacle geometry and motion | |
| Scenario definitions and collision ground truth | |
| The simulator side of the PPO and sensor interfaces | |

### Teammate code

Spencer's `DroneNavPPO` package is **not vendored here** and is not modified by
anything in this repository. `dns/spencer_link.py` puts his directory on
`sys.path` at runtime so that the observation contract (`DT`, `SPEED`,
`RADIUS`, `OBS_DIM`, `pack_scan`, `validate`, …) can never silently drift out of
sync with the trained policies. His files are the authoritative interface spec.

By default the package is expected one level above this repository:

```
<parent>/
├── webots_dns/        <- this repository
└── DroneNavPPO/
    └── DroneNavPPO/   <- navigation.py lives here
```

If it lives anywhere else, point `DNS_PPO_DIR` at the directory containing
`navigation.py`:

```bash
export DNS_PPO_DIR=/path/to/DroneNavPPO/DroneNavPPO     # bash
$env:DNS_PPO_DIR = "C:\path\to\DroneNavPPO\DroneNavPPO" # PowerShell
```

Only the 46 unit tests run without it. Everything that touches the PPO bridge
needs it present.

---

## Requirements

| | |
|---|---|
| Simulator | Webots **R2025a** (set `WEBOTS_HOME` if it is not auto-detected) |
| Python | 3.11+ — last verified on 3.15.2 |
| Required | `numpy` |
| Optional | `matplotlib` (figures), `Pillow` (image probes), `imageio-ffmpeg` (video) |

```bash
pip install -r requirements.txt
```

---

## Quick start

```bash
# 1. no simulator needed -- 46 checks, a few seconds
python tests/unit_tests.py

# 2. full interface and physics check against Webots -- 22 checks
python scripts/with_webots.py --timeout 420 --world worlds/DNS_Warehouse.wbt \
    tests/client_validation.py -- --config config/scenario_warehouse.json \
    --dynamics physical --sensing lidar
```

`scripts/with_webots.py` launches headless Webots on a free IPC port, runs the
client as a child process with `libController` on its path, and tears Webots
down afterwards. Add `--gui` to watch.

The drone's controller is `<extern>`: Webots loads the scene and then **waits**
for a Python program to connect and fly it. A drone sitting still in the GUI is
expected — run a client to drive it.

> **Worlds are generated, not hand-edited.** After changing `config/*.json` or
> `protos/DnsQuadcopter.proto`, re-run `python -m dns.worldgen` (see
> [docs/REPRODUCIBILITY.md](docs/REPRODUCIBILITY.md)) or the `.wbt` still holds
> the old values.

Full step-by-step reproduction of every reported figure, table and video:
**[docs/REPRODUCIBILITY.md](docs/REPRODUCIBILITY.md)**.

---

## Repository layout

```
config/        scenario definitions -- the single source of truth for geometry
dns/           the simulation package
  scenario.py    config loading, validation, DNS <-> Webots frame transform
  worldgen.py    writes .wbt worlds from a scenario JSON
  backend.py     implements Spencer's navigation.Backend protocol
  autopilot.py   cascaded velocity -> attitude -> rotor-thrust controller
  sensing.py     12 x 30 deg sector sensing (none / lidar / geometric)
  launcher.py    starts Webots, prepares the <extern> controller environment
  spencer_link.py  locates and imports Spencer's PPO package
protos/        DnsQuadcopter.proto -- the airframe, rotors and sensor mounts
worlds/        generated Webots worlds (see note below)
controllers/   small in-simulator diagnostic controllers
tests/         client_*.py run against Webots; unit_tests.py runs standalone
scripts/       harness, figure/report/verification generators, MP4 verifier
results/       raw measured output -- the provenance behind every claim
evidence/      reports, figures, tables and the teammate interface documents
docs/          environment audit, sensor-attitude study, reproducibility guide
```

### Scenarios

| Config | World | What it is |
|---|---|---|
| `scenario_warehouse.json` | `DNS_Warehouse.wbt` | **Mode C, dynamic** — the transfer cart sweeps the aisle |
| `scenario_warehouse_static.json` | `DNS_Warehouse.wbt` | **Mode B, static** — the same cart parked against the wall |
| `scenario_default.json` | `dns_arena.wbt` | bare 10 × 6 m arena, 2 cylinders |
| `scenario_research_demo.json` | `DNS_Research_World.wbt` | earlier research-demo layout |
| `scenario_demo_static.json` | `DNS_Research_World.wbt` | its static counterpart |
| `scenario_s2_single_moving.json` | `S2_single_moving.wbt` | single-moving-obstacle isolation case |
| `scenario_empty.json` | `dns_empty.wbt` | obstacle-free flight-dynamics reference |

The two warehouse configs differ in exactly one factor, so results between them
are attributable to obstacle motion alone.

### A note on `worlds/_*.wbt` and `results/bisect_*`

Files prefixed with `_` are the **bisection ladder** from diagnosing the rotor
physics (`_shell_S0…S8e`, `_quad_Q0…Q5d`, `_bisect_*`, `_probe_*`) with their
captured logs in `results/`. They are kept deliberately: they are the record of
why the airframe, timestep and thrust configuration are what they are. They are
not part of the deliverable — the five worlds without a `_` prefix are.

---

## The environment, in numbers

| | |
|---|---|
| Arena | **10.0 × 6.0 m**, walls 2.5 m, 1 m floor grid |
| Frame | +x forward, +y right; origin at a corner, fixed yaw |
| Start | **(0.85, 0.00) m**, green pad |
| Goal | **(9.15, 0.00) m**, blue pad, success within **0.4 m** |
| Altitude | 1.0 m, held |
| Drone | 0.55 kg quadcopter, thrust/weight 4.0, 0.18 m collision radius |
| Static obstacles | 2 storage racks (0.8 × 2.4 m), 2 columns (0.4 × 0.4 m), 1 crate stack (0.7 × 0.7 m) |
| Moving obstacle | transfer cart 0.9 × 0.55 m, crossing at **0.45 m/s** over ±1.9 m |
| Sensor | 360° LiDAR, 720 beams, 5 m range, 12 sectors, gimbal-stabilised |
| Control | 5 Hz commands, max 1.2 m/s per axis, 125 Hz inner loop, 8 ms physics |

Arena dimensions mirror Spencer's `planar.py` benchmark (`LOW=[0,-3]`,
`HIGH=[10,3]`) so trained policies see a familiar workspace.

---

## What was verified

Tested and passing, not merely written — 46 unit tests, 22 simulator checks,
13 moving-obstacle checks:

- The drone flies on rotor thrust — nothing teleports it.
- Commands advance the clock by exactly 0.200000 s (error < 3e-16 s).
- All four commanded directions move it the right way.
- Reported speed is **measured**, not the command echoed back.
- Collisions fire against exact obstacle footprints; resets repeat to 2.8 µm.
- The LiDAR's angular mapping was **measured** (RMS 0.025 m), not assumed.
- The moving cart matches its mathematical path to **0.00e+00 m** and replays
  identically; its speed measures 0.4500 m/s against 0.45 configured.
- The scan agrees with an **independent** analytic ray-cast of the same scene
  to 0.048 m mean across a moving episode, occlusion included.
- Spencer's own verifier accepts our sensor packet unmodified.

Interface documents for teammates: [`evidence/PPO_Interface.md`](evidence/PPO_Interface.md),
[`evidence/Sensor_Interface.md`](evidence/Sensor_Interface.md).
Sensor-mount study: [`docs/SENSOR_ATTITUDE.md`](docs/SENSOR_ATTITUDE.md).

---

## What is still incomplete

Read this before quoting any result.

1. **One fixed obstacle layout.** The dynamic mode varies *when* the cart
   arrives, not *where* obstacles are, so nothing here shows the controller
   would handle a different room. This is the largest open item.
2. **Only three trained controllers**, and their dynamic-mode spread (47–60%)
   is as wide as the effect being measured.
3. **The scripted baselines are untuned for this scene** and must not be read as
   a ranking of methods. The reactive rule stalls 1.6 m short because its fixed
   1.45 m repulsion radius exceeds the 1.54 m crate-to-goal distance.
4. **The LiDAR mount is an idealised gimbal** — massless, no lag, no limits.
5. **One scan layer**, so the sensor is blind to anything not crossing its
   plane; the validator rejects obstacles shorter than the scan height.
6. **Obstacle motion is kinematic**, driven through the Supervisor rather than
   by contact forces.
7. **Webots' own screen recorder does not work in this automated setup** — it
   produced blank frames (`results/render_probe/render_probe.json`). The video
   was captured through an in-scene `Camera` sensor instead; every frame is
   still a real Webots render of a real run, verified by
   `scripts/verify_mp4.py`.
8. **Simulation is not reality.** No physical flight data was collected.
