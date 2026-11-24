# START HERE — Drone Navigation System (DNS)

This folder is a complete, runnable drone-navigation simulation plus the
evidence behind every number we report. Read this page first; it takes about
five minutes and tells you where everything is.

---

## 1. What is this project?

A simulated drone has to fly from a start point to a goal point in a closed
room without hitting anything. It can only "see" through a laser scanner
(LiDAR) mounted on it — it is never told where the obstacles are. Some
obstacles stand still; one moves across its path.

## 2. What is the research objective?

To find out whether a navigation controller trained by deep reinforcement
learning (an algorithm called **PPO**) can fly that route safely, and how its
behaviour changes when an obstacle **moves** instead of standing still.

Three people work on this. This folder is the **simulation environment** part:
the world, the drone, its flight controller, the sensor, the moving obstacles,
and the connection that lets a trained controller fly inside the simulation.
Spencer owns the PPO algorithm and the trained controllers; a third teammate
owns LiDAR data processing.

## 3. Which simulator?

**Webots R2025a.** Chosen because it runs Python directly, ships the sensor and
rotor models we need, lets a script reset an experiment exactly the same way
every time, and stores worlds as plain text we can generate from a
configuration file.

## 4. How big is the map?

**10.0 m long × 6.0 m wide**, with 2.5 m walls. The drone flies at **1.0 m**
altitude.

The size is deliberate, not arbitrary: the PPO controllers were trained in a
room of exactly these dimensions, so changing it would make the results
incomparable. See section 3 of the report.

## 5. Where are the start and goal?

| | Position (x forward, y right) | Notes |
|---|---|---|
| **Start** | (0.85, 0.00) m | green marker on the floor |
| **Goal** | (9.15, 0.00) m | red marker; counts as reached within **0.4 m** |

Coordinates start at one corner of the room: **+x is forward**, **+y is to the
drone's right**.

## 6. Which obstacles are there?

| ID | Type | Position (m) | Radius | Behaviour |
|---|---|---|---|---|
| `obs_a` | static pillar | (3.50, −0.35) | 0.55 m | still |
| `obs_b` | static pillar | (6.60, 0.55) | 0.50 m | still |
| `mover` | crossing obstacle | (5.10, 0.00) | 0.50 m | slides along y, ±1.6 m, **0.45 m/s** |

All are 1.8 m tall cylinders. The two pillars sit on opposite sides of the
straight line to the goal, so the drone must change side at least once.

The same world runs in two modes:

- **Mode B (static)** — `mover` is parked at the edge; only the pillars block.
- **Mode C (dynamic)** — `mover` sweeps across the route.

## 7. How do I open the world?

1. Install **Webots R2025a**.
2. Open `01_WEBOTS_SIMULATION/worlds/DNS_Research_World.wbt`.

The drone's controller is `<extern>`, which means Webots will load the scene and
then **wait** for a Python program to connect and fly it. That is expected — the
drone sits still until you run one of the commands below.

## 8. How do I run the simulation?

From `01_WEBOTS_SIMULATION/`, with Python 3 and `numpy` installed:

```bash
# the full interface and physics check (22 checks)
python scripts/with_webots.py --world worlds/DNS_Research_World.wbt \
    tests/client_validation.py -- --config config/scenario_research_demo.json \
    --dynamics physical --sensing lidar
```

`scripts/with_webots.py` starts Webots for you, runs the test, and shuts Webots
down afterwards. You do not need to open Webots by hand.

If Webots is not found automatically, set `WEBOTS_HOME` to its install folder.

## 9. How do I reproduce the recorded experiment?

```bash
# all 90 episodes behind the results table (about 2 minutes)
python scripts/with_webots.py --timeout 3000 \
    --world worlds/DNS_Research_World.wbt tests/client_experiments.py

# re-record the demonstration video
python scripts/with_webots.py --timeout 2400 \
    --world worlds/DNS_Research_World.wbt tests/client_record_demo.py

# checks that need no Webots at all (38 tests, a few seconds)
python tests/unit_tests.py
```

## 10. Where is the video?

`02_DEMO_VIDEO/DNS_Research_Demonstration.mp4` — 49 seconds, 1280×720, 1225
frames. It shows the room, the drone, a successful static run, and then the
dynamic run **where the drone crashes into the moving obstacle**. The failure is
shown on purpose.

## 11. Where are the figures and tables?

`04_FIGURES_TABLES_RESULTS/`

- `figures/fig1` map with dimensions · `fig2` system diagram ·
  `fig3` LiDAR validation · `fig4` real flight paths · `fig5` success rates
- `tables/table1` configuration · `table2` scenarios · `table3` results
- `experiment_results.csv` — one row per episode, all 90

## 12. Where is the report?

`03_RESEARCH_REPORT/DNS_Research_Report.pdf` — 10 pages, with the full
reasoning, methods, results and limitations.

## 13. What has actually been validated?

Things that were **tested and passed**, not just written:

- The drone really flies — rotor thrust and physics, no teleporting.
- Commands arrive every 0.200000 s, accurate to better than a nanosecond.
- All four directions move the drone the correct way.
- Reported speed is **measured**, not the command echoed back.
- Collisions are detected; resets repeat to within 3 µm.
- The LiDAR's angle mapping was **measured** (0.015 m error), not assumed.
- The moving obstacle matches its mathematical path **exactly** and replays
  identically.
- Spencer's own verification program accepts our sensor data file unchanged.
- A trained PPO controller flies the whole loop, start to finish.

Full list with evidence links: `05_VERIFICATION_EVIDENCE/VERIFICATION_REPORT.md`.

## 14. What remains incomplete?

Read this before quoting any number.

1. **One obstacle layout only.** The pillars never move between experiments.
   Mode C varies *when* the crossing obstacle arrives, not *where* obstacles
   are. So nothing here shows the controller would handle a different room.
2. **Only three trained controllers.** In Mode C they scored 60%, 53% and 93%.
   That spread between seeds is as wide as the effect we are measuring, so we
   cannot yet say PPO "can" or "cannot" handle crossing obstacles.
3. **The simple scripted baseline beat PPO** in Mode C (100%). A real result on
   this task — and a caution against over-claiming for the learned controller.
4. **The LiDAR mount is idealised** — a perfect, weightless, instant gimbal. A
   real one has mass and lag.
5. **Four sensor changes were made at once**, so the improvement cannot be
   attributed to any single one of them yet.
6. **Webots' own screen recorder does not work in our automated setup.** The
   video was captured through a camera sensor inside the simulation instead;
   every frame is still a genuine Webots render of the real run.
7. **Simulation is not reality.** None of this predicts how a physical drone
   would behave.
