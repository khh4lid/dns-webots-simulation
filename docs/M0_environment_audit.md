# M0 — Workspace and Research Audit

**Owner:** Khaled (Simulation Environment Development)
**Project:** Drone Navigation System (DNS), Taylor's University
**Supervisor:** Dr. Mehdi Samieiyeganeh
**Audit date:** 2026-11-24

Everything below was read off this machine. Items that could not be verified are
marked explicitly rather than assumed.

---

## 1. Workspace

Confirmed root: `D:\Download\drone_navigation_system`

| Item | Status | Notes |
|---|---|---|
| `Drone.pdf` | present, **NOT READ** | 361 KB. PDF page rendering is unavailable in this environment (`pdftoppm`/poppler not installed), so the research proposal has **not** been read. See *Gaps*. |
| `DroneNavPPO/` | present | Spencer's PPO work, extracted. Nested one level: `DroneNavPPO/DroneNavPPO/`. |
| `DroneNavPPO.zip` | present | 809 KB archive of the same. |
| `webots_dns/` | present | Our Webots simulation project. |
| `photo_*.jpg` x 6 | present | Reference photos, 110–207 KB each. Provenance unconfirmed. |
| `AGENTS.md` | **ABSENT** | Expected per the directive; does not exist at the workspace root. |
| `README.md` (root) | **ABSENT** | Does not exist at the workspace root. |

Not a git repository. Version control is **not** initialised; reproducibility
currently rests on the generated-world + config-file design and the recorded
`protocol.json` / `config.json` metadata, not on commit hashes.

## 2. Toolchain

| Component | Verified value |
|---|---|
| OS | Windows 10 Pro 10.0.19045 |
| Webots | **R2025a**, at `D:\Program files\Webots` (note the lowercase "f"; there is no `C:\Program Files\Webots`) |
| `WEBOTS_HOME` | already set to `D:\Program files\Webots` |
| Webots binary | `D:\Program files\Webots\msys64\mingw64\bin\webots.exe` |
| Python controller bindings | present at `lib/controller/python` |
| Python | **3.15.2** |
| numpy | 2.3.5 |
| matplotlib | **3.11.2 — installed during this session** (see *Deviations*) |
| pytest | **not installed** — unit tests are written with plain asserts and a built-in runner, so no test dependency is required |

Launch model: the drone's controller is `<extern>`. `scripts/with_webots.py`
starts headless Webots on the first free IPC port, runs the client as a child
process with `libController` on its path, and terminates Webots afterwards. The
subprocess design is deliberate — `Supervisor()` blocks forever when no matching
robot is waiting, so a timeout is the only safe failure mode.

## 3. Spencer's PPO work (read-only, unmodified)

Source: `DroneNavPPO/DroneNavPPO/`.

| File | Role |
|---|---|
| `navigation.py` | **The interface contract.** `Frame`, `pack_scan`, `validate`, `observation`, `Navigation`, and the constants `DT=0.2`, `SPEED=1.2`, `RANGE=5.0`, `SECTORS=12`, `OBS_DIM=52`, `RADIUS=0.18`, `GOAL_TOL=0.4`, `HORIZON=90`, `SCHEMA="fixed-yaw-frd-sectors-v1"`. |
| `planar.py` | 2-D reference environment the policies were trained in. |
| `ppo.py` | NumPy PPO-Clip. `PPO.load(path).predict(obs)` returns a tanh-squashed action in [-1, 1]. |
| `evaluate.py` | Paired evaluation; exposes `backend_factory`, plus the `goal_direct` and `gap_rule` scripted baselines. |
| `handoff.py` | Two-frame packet builder/verifier. **This is the acceptance test for our handoff.** |
| `train.py`, `run_experiment.py`, `report_results.py`, `test_project.py` | Training and reporting. Not used by us. |

**Trained checkpoints — real and compatible.** `evidence/seed{7,19,43}/model.npz`,
each a 52→32→32→2 tanh policy plus a 52→32→32→1 value net, tagged
`schema="fixed-yaw-frd-sectors-v1"`. Training config: 96 updates, rollout 1024,
batch 64, 6 epochs, gamma 0.99, GAE lambda 0.95, clip 0.2, hidden [32, 32],
lr 3e-4, entropy bonus 0, backend `PlanarRoom`.

Each `config.json` records `source_sha256` for `navigation.py`, `planar.py`,
`ppo.py` and `train.py`, which gives us a cryptographic check that his sources
are unmodified. **We have not edited any file under `DroneNavPPO/`.**

Also present: `evidence/paired50/`, `evidence/matched_speed/` (his own planar
results) and `figures/*.png` (5 figures). `handoff/two_frames.json` is his
synthetic fixture, generated from `PlanarRoom` — **not** from Webots.

## 4. Drone model decision

The directive suggests investigating the official Crazyflie example. We use a
**custom `DnsQuadcopter` PROTO** instead. Rationale:

- It is built only from Webots base nodes, so the world loads with no network
  access and no remote `EXTERNPROTO` download.
- Its collision geometry is matched deliberately to Spencer's clearance model:
  rotor hubs at +-0.09 m, rotor radius 0.05 m, giving a 0.1773 m tip radius
  against `navigation.RADIUS = 0.18`, and a `boundingObject Cylinder` of radius
  0.18 m so Webots contact physics and our geometric collision test agree.
- Mass budget 0.55 kg, Ixx = Iyy = 0.00272, Izz = 0.00513 kg m^2.

This is a **provisional engineering choice**, not a validated match to any real
airframe. It should be stated as such to the supervisor.

## 5. Milestone status at audit time

| Milestone | Status | Evidence |
|---|---|---|
| M0 workspace audit | **PASS** | this document |
| M1 physical drone flies | **PASS** | `results/validation_physical_lidar.json` |
| M2 PPO velocity interface | **PASS** | 22/22 checks, exact 0.2 s steps |
| M3 lidar integration + handoff | **PASS** | `results/two_frames_webots_physical_lidar.json`, accepted by Spencer's unmodified `handoff.py` |
| M4 dynamic obstacles | **PASS** | `results/moving_obstacle.json` — 12/12 checks on `S2_single_moving`: supervisor matches `centre_at(t)` to 0.0 m, speed 0.5000 m/s as configured, bit-exact replay (2.7e-15 m), lidar tracks it across 3 sectors to 0.012 m, collision fires |
| M5 scenario suite | **PARTIAL** | `default_10x6` (S1 static baseline) and `s2_single_moving` built and verified; S3–S8 not built |
| M6 PPO end-to-end | **PASS (pilot)** | `results/policy_rollout/` — 9/9 PPO episodes reached goal; **single fixed layout**, so not a success-rate or generalisation result |
| M7 research evidence | **PARTIAL** | 5 figures in `evidence/figures/`, all from recorded runs |
| M8 supervisor package | **NOT RUN** | — |

## 6. Technical risks

1. **No layout randomisation.** Every episode runs the same obstacle placement,
   and physical mode is near-deterministic (two of three PPO seeds reproduced
   bit-identical paths across three episodes). Repeated episodes therefore add
   almost no statistical information. Any success rate from the current setup is
   an artefact of one layout. This blocks meaningful M5/M6 evaluation and is the
   single highest-priority gap.
2. **Scene-label mismatch.** Our scenario has only static obstacles, so we pass
   `scene="static"`. Spencer's policies were trained across `static` and
   `moving`. Comparing our numbers with his planar numbers requires matching the
   scene label, which we cannot do until M4 lands.
3. **Sensor model is an idealisation.** The lidar mount is a perfect gimbal with
   zero mass, no actuator lag and no joint limits. Documented in
   `SENSOR_ATTITUDE.md`; it is a simulation choice, not a hardware claim.
4. **Single scan layer.** The lidar is blind to anything not crossing its plane.
   `Scenario._check_lidar_clearances` does not yet verify per-obstacle height
   against the scan height.
5. **Proposal unread.** Design decisions here are driven by the directive and by
   Spencer's source, not by `Drone.pdf`. Any requirement stated only in the PDF
   is currently unaccounted for.
6. **No version control**, so "code version" cannot be cited in a reproducibility
   record.
7. **`docs/DESIGN_NOTES.md` is missing** although `DnsQuadcopter.proto` cites it
   twice (inertia derivation; the helix-collider finding). The reasoning exists
   only in PROTO comments.
8. **Settle cost.** Each reset spends ~9.6 s of simulation time settling the
   airframe, which dominates short episodes. Acceptable now (15 episodes in
   17.6 s wall clock) but it will matter at 30 episodes x many conditions.

## 7. Deviations from the directive

- **matplotlib installed.** The directive says not to install unknown
  executables. matplotlib is a declared dependency in Spencer's
  `requirements.txt` (`matplotlib==3.10.8; python_version >= "3.14"`), so this
  is in scope rather than arbitrary. The pinned 3.10.8 **fails to build** on
  Python 3.15.2 (metadata generation error); **3.11.2** was installed instead.
  This version skew should be recorded in any reproducibility statement.
- **Lidar beam count changed 360 → 720.** Our parameter to own, but it is
  visible to anyone reading the raw range image. Flagged in
  `SENSOR_ATTITUDE.md` and recorded as `lidar_beams` in the handoff packet.

## 8. Gaps to close next

1. Build S1–S5 scenario configs, starting with a moving obstacle to close M4.
2. Add seeded layout randomisation so evaluation means something (S6).
3. Read `Drone.pdf` — needs poppler, or supply the text another way.
4. Write `docs/DESIGN_NOTES.md` to back the PROTO's citations.
5. Decide whether to initialise git for this workspace.
