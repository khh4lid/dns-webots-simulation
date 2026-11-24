# DNS — Webots Simulation Environment

A drone-navigation simulation built in Webots R2025a, with the evidence behind
every number reported from it. Read this page first.

---

## What this project does

A simulated quadcopter must fly from a start point to a goal point across an
indoor warehouse aisle without hitting anything. It perceives the world only
through a laser scanner (LiDAR) mounted on it — it is never told where the
obstacles are. Some obstacles are fixed; one is a powered cart that crosses its
route.

## What my task was

The project has three parts. **Mine is the simulation environment**: the Webots
world, the quadcopter and its flight controller, the sensor mounting and
configuration, the obstacle geometry and motion, the scenario definitions, the
collision ground truth, and the interfaces that let my teammates plug in.

Spencer owns the reinforcement-learning algorithm and the trained controllers.
A third teammate owns LiDAR signal processing. Their code is used here only to
prove my interfaces work; it was not modified.

## How to run Webots

1. Install **Webots R2025a**.
2. If it is not found automatically, set `WEBOTS_HOME` to its install folder.
3. Python 3 with `numpy` is required. `matplotlib` is needed only to regenerate
   figures.

## How to launch the world

Open `01_SIMULATION/worlds/DNS_Warehouse.wbt` in Webots, or on Windows
double-click `01_SIMULATION/run.bat`.

The drone's controller is `<extern>`, which means Webots loads the scene and
then **waits** for a Python program to connect and fly it. The drone sitting
still is expected — run one of the commands below to drive it.

## How to choose scenarios

Two scenarios share the one world, differing in a single factor:

| File | Mode | Difference |
|---|---|---|
| `config/scenario_warehouse.json` | **C, dynamic** | the transfer cart sweeps the aisle |
| `config/scenario_warehouse_static.json` | **B, static** | the same cart is parked against the wall |

Pass whichever you want with `--config`.

## How to run the simulation

From `01_SIMULATION/`:

```bash
# interface and physics check — 22 assertions
python scripts/with_webots.py --world worlds/DNS_Warehouse.wbt \
    tests/client_validation.py -- --config config/scenario_warehouse.json \
    --dynamics physical --sensing lidar

# checks that need no Webots at all — 46 tests, a few seconds
python tests/unit_tests.py
```

## How to reproduce the recorded experiment

```bash
# all 90 episodes behind Table 3 and the results section (~3 minutes)
python scripts/with_webots.py --timeout 3000 --world worlds/DNS_Warehouse.wbt \
    tests/client_experiments.py -- \
    --dynamic-config config/scenario_warehouse.json \
    --static-config  config/scenario_warehouse_static.json \
    --out results/experiments_warehouse

# moving-obstacle validation (Figure 4)
python scripts/with_webots.py --world worlds/DNS_Warehouse.wbt \
    tests/client_moving_obstacle.py -- --config config/scenario_warehouse.json \
    --out results/moving_obstacle_warehouse.json

# sensor mount ablation (Figure 5a)
python scripts/with_webots.py --timeout 700 --world worlds/DNS_Warehouse.wbt \
    tests/client_lidar_attitude.py -- --config config/scenario_warehouse.json \
    --out results/lidar_attitude_warehouse.json

# re-record the demonstration video
python scripts/with_webots.py --timeout 520 --world worlds/DNS_Warehouse.wbt \
    tests/client_record_demo.py
```

## Where everything is

| What | Where |
|---|---|
| **Video** | `02_DEMONSTRATION/DNS_Webots_Research_Demo.mp4` |
| **Research report** | `03_RESEARCH/DNS_Simulation_Research_Report.pdf` |
| **Figures** | `03_RESEARCH/figures/` (5) |
| **Tables** | `03_RESEARCH/tables/` (3, CSV) |
| **Per-episode data** | `03_RESEARCH/results.csv` (90 rows) |
| **Interface docs** | `04_TEAM_HANDOFF/PPO_Interface.md`, `Sensor_Interface.md` |
| **Sensor sample** | `04_TEAM_HANDOFF/two_frames.json` |
| **Verification** | `05_VERIFICATION/Verification_Report.md`, `test_log.txt` |

## The environment, in numbers

| | |
|---|---|
| Arena | **10.0 x 6.0 m**, walls 2.5 m, 1 m floor grid |
| Frame | +x forward, +y right; origin at a corner |
| Start | **(0.85, 0.00) m**, green pad |
| Goal | **(9.15, 0.00) m**, blue pad, success within **0.4 m** |
| Altitude | 1.0 m, held |
| Drone | 0.55 kg quadcopter, 0.18 m collision radius |
| Static obstacles | 2 storage racks (0.8 x 2.4 m), 2 columns (0.4 x 0.4 m), 1 crate stack (0.7 x 0.7 m) |
| Moving obstacle | transfer cart 0.9 x 0.55 m, crossing at **0.45 m/s** over +/-1.9 m |
| Sensor | 360 deg LiDAR, 720 beams, 5 m range, 12 sectors, gimbal-stabilised |
| Control | 5 Hz commands, max 1.2 m/s per axis, 125 Hz inner loop, 8 ms physics |

## What was verified

Tested and passing, not merely written:

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
- 46 unit tests, 22 simulator checks, 13 moving-obstacle checks.

Full table with evidence paths: `05_VERIFICATION/Verification_Report.md`.

## What is still incomplete

Read this before quoting any result.

1. **One fixed obstacle layout.** The dynamic mode varies *when* the cart
   arrives, not *where* obstacles are, so nothing here shows the controller
   would handle a different room. This is the largest open item.
2. **Only three trained controllers**, and their dynamic-mode spread (47–60%)
   is as wide as the effect being measured.
3. **The scripted baselines are untuned for this scene** and must not be read
   as a ranking of methods. The reactive rule stalls 1.6 m short because its
   fixed 1.45 m repulsion radius exceeds the 1.54 m crate-to-goal distance.
4. **The LiDAR mount is an idealised gimbal** — massless, no lag, no limits.
5. **One scan layer**, so the sensor is blind to anything not crossing its
   plane; the validator now rejects obstacles shorter than the scan height.
6. **Obstacle motion is kinematic**, driven through the Supervisor rather than
   by contact forces.
7. **Webots' own screen recorder does not work in our automated setup** — it
   produced blank frames. The video was captured through an in-scene camera
   sensor instead; every frame is still a real Webots render of the real run.
8. **Simulation is not reality.** No physical flight data was collected.
