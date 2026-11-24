# PPO Interface — DNS Webots environment

**For:** Spencer (DRL / PPO)
**From:** simulation environment
**Status:** verified against your unmodified `handoff.py` and `navigation.py`.

Nothing in `DroneNavPPO/` was edited. The SHA-256 digests your own
`evidence/seed*/config.json` records for `navigation.py`, `planar.py`, `ppo.py`
and `train.py` still match the files on disk.

---

## 1. The short version

The Webots backend implements your `Backend` protocol, so it drops straight
into `Navigation`:

```python
from dns.launcher import webots_session
with webots_session(world="worlds/DNS_Warehouse.wbt"):
    from dns.backend import WebotsBackend
    from navigation import Navigation          # yours, unmodified
    env = Navigation(WebotsBackend())
    obs, info = env.reset(seed=12, scene="moving")
    obs, reward, terminated, truncated, info = env.step([0.5, -0.25])
```

`obs` is the same 52-feature vector your `observation()` builds, because it is
built by your `observation()`. We never normalise anything.

## 2. Running it

From `01_SIMULATION/`:

```bash
# dynamic warehouse (transfer cart sweeping)
python scripts/with_webots.py --world worlds/DNS_Warehouse.wbt \
    tests/client_experiments.py -- \
    --dynamic-config config/scenario_warehouse.json \
    --static-config  config/scenario_warehouse_static.json

# interface + physics check, 22 assertions
python scripts/with_webots.py --world worlds/DNS_Warehouse.wbt \
    tests/client_validation.py -- --config config/scenario_warehouse.json \
    --dynamics physical --sensing lidar
```

`scripts/with_webots.py` starts Webots, runs the client as a subprocess with
`libController` on its path, and shuts Webots down. Set `WEBOTS_HOME` if Webots
is not auto-detected.

## 3. Coordinate convention

| | |
|---|---|
| +x | forward |
| +y | **right** of the drone |
| altitude | held at 1.0 m |
| heading | held at 0 rad |

Webots is ENU; the mapping is `x_dns = x_webots`, `y_dns = -y_webots`, defined
once in `dns/scenario.py` and used nowhere else by hand.

## 4. Start, goal, termination

| | Value |
|---|---|
| start | (0.85, 0.00) m |
| goal | (9.15, 0.00) m |
| success | drone centre within **0.4 m** of the goal (`navigation.GOAL_TOL`) |
| horizon | 90 steps = 18.0 s (`navigation.HORIZON`) |
| collision | swept test of the drone's 0.18 m disc against exact obstacle footprints and the walls |

Exactly one of `goal`, `collision`, `timeout` ends every episode. No episode is
dropped, so the denominator of any rate is the full cell.

## 5. Action and timing

- You pass **physical velocity** `[vx, vy]` in m/s, already scaled. We do not
  scale again.
- Per-axis cap **1.2 m/s** (`navigation.SPEED`). Exceeding it raises
  `BackendError` rather than silently clipping.
- One `step` advances exactly **0.200000 s** of simulation time (25 x 8 ms).
  Measured timing error is below 3e-16 s.
- The command is held for the whole interval; a cascaded autopilot tracks it at
  125 Hz underneath.

## 6. What comes back

`navigation.Frame`, all in metres and m/s, DNS axes:

| field | meaning |
|---|---|
| `time_s` | simulation seconds since the episode reset (reset returns 0.0) |
| `xy` | drone position |
| `goal_xy` | goal position |
| `velocity` | **measured** from the GPS speed vector |
| `ranges` | 12 sector distances, metres, capped at 5.0 |
| `valid` | per-sector coverage flag |
| `collision` | ground-truth collision for this step |

**`velocity` is a measurement, not your command echoed back.** After one 0.2 s
step a commanded `[1.2, 0.0]` reads about `[0.28, 0.00]` — that is the
vehicle's physical rise time. If you ever see measured exactly equal commanded,
you are in `dynamics="kinematic"` mode, which is an interface-test mode and is
labelled as such in every record.

## 7. Reset

`reset(seed, scene)` restores drone position, altitude, heading, linear and
angular velocity, obstacle poses and phase, and zeroes the episode clock.
Measured: two resets start within **2.8e-6 m** of each other and a
three-command trajectory reproduces to **7.1e-3 m**. Physical mode is not
bit-exact because the settle phase ends on a tolerance test; that is the
expected behaviour of a feedback-controlled rigid body, not drift.

`seed` is recorded and, in the dynamic scenario, selects the crossing obstacle's
initial phase. It does **not** currently randomise obstacle positions — see
limitations.

## 8. Worked example

`04_TEAM_HANDOFF/two_frames.json` is a real packet from the warehouse world. It
passes your verifier unmodified:

```bash
cd DroneNavPPO/DroneNavPPO
python handoff.py --file .../04_TEAM_HANDOFF/two_frames.json
# PASS: command, timing, 52 features, reward, and episode flags
```

Note the packet layout: `frames[i]` carries **exactly** the `navigation.Frame`
fields, because your verifier does `Frame(**raw)` and any extra key is a
`TypeError`. Our richer per-step record (altitude, attitude, mount mode, contact
state, commanded velocity) lives under `dns_records`, a key your code never
reads.

## 9. What you should know before quoting any number

- **One fixed obstacle layout.** The dynamic condition varies *when* the cart
  arrives, not *where* obstacles are. Nothing here supports a generalisation
  claim. Randomised layouts are the next piece of work on our side.
- **Physical mode is near-deterministic.** Repeated episodes on a fixed layout
  measure run-to-run variation, not scenario variation, so a rate over them
  carries little information. Vary the seed (and therefore the phase) instead.
- **The scene is harder than the planar baseline.** Racks are long flat
  surfaces rather than discs, and they straddle the straight line to the goal.
  Your checkpoints reach the goal in every static episode and between 47% and
  60% of dynamic episodes; the spread *between seeds* is as wide as the gap
  between conditions.
- **Our baselines are untuned for this scene** and should not be read as a
  ranking. The reactive rule's repulsion radius (1.45 m) is larger than the
  crate-to-goal distance (1.54 m), so it stalls 1.6 m short without ever
  colliding.
- **The LiDAR is one scan layer** and the mount is an idealised gimbal (zero
  mass, no lag). Both are modelling choices, labelled in every record.
