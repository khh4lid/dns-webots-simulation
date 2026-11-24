# Reproducibility guide

Every figure, table, number and video in this project can be regenerated from
this repository. This page lists the exact commands, in order, with the runtime
to expect.

Commands are written for a shell opened at the repository root. On Windows
PowerShell replace the trailing `\` line continuations with a backtick `` ` ``
or put each command on one line.

---

## 0. Environment

| | Verified configuration |
|---|---|
| OS | Windows 10 Pro (19045) |
| Simulator | Webots **R2025a** |
| Python | 3.15.2 |
| `numpy` | 2.3.5 |
| `matplotlib` | 3.11.2 |
| `Pillow` | 13.0.0 |
| `imageio-ffmpeg` | 0.8.1 |

```bash
pip install -r requirements.txt
```

Webots is located through `WEBOTS_HOME`, falling back to a platform search in
`dns/launcher.py`. If auto-detection fails:

```bash
export WEBOTS_HOME=/path/to/Webots              # bash
$env:WEBOTS_HOME = "D:\Program files\Webots"    # PowerShell
```

Spencer's PPO package must be reachable — one level above this repository, or
pointed at by `DNS_PPO_DIR`. See the README section "Teammate code". Step 1 is
the only step that runs without it.

### How the harness works

`scripts/with_webots.py` picks a free IPC port, launches Webots headless on the
given world, runs the client in a **child process** with `libController` on its
path, then shuts Webots down. The child-process-plus-timeout design is
deliberate: `Supervisor()` blocks forever if no matching robot is waiting, so a
failed connection must time out rather than hang.

Everything after `--` is forwarded to the client. Add `--gui` to watch a run.

---

## 1. Tests that need no simulator

```bash
python tests/unit_tests.py
```

**Expect:** `46/46 unit tests passed`. Runtime: a few seconds.

---

## 2. Regenerate the worlds

Worlds are written from the scenario JSON rather than hand-maintained, so arena
size, start, goal and obstacles always agree with `config/`. **Re-run this after
any change to `config/*.json` or `protos/DnsQuadcopter.proto`**, or the `.wbt`
still holds the old values.

```bash
# the warehouse research world (both warehouse scenarios share it)
python -m dns.worldgen --config config/scenario_warehouse.json \
    --out worlds/DNS_Warehouse.wbt

# the bare arena (default scenario)
python -m dns.worldgen
```

**Expect:** a scenario summary ending in `basicTimeStep : 8 ms (25 steps per
0.2 s command)`. The remaining worlds follow the same pattern — pair each
`config/scenario_*.json` with its world from the table in the README.

---

## 3. Interface and physics validation

```bash
python scripts/with_webots.py --timeout 420 --world worlds/DNS_Warehouse.wbt \
    tests/client_validation.py -- --config config/scenario_warehouse.json \
    --dynamics physical --sensing lidar
```

**Expect:** `22/22 checks passed`, and two written files:
`results/validation_physical_lidar.json`,
`results/two_frames_webots_physical_lidar.json`. Runtime: ~2 minutes.

`--dynamics kinematic` and `--sensing none|geometric` exist for labelled
interface tests only. `kinematic` integrates the commanded velocity directly and
is **not a flight model**; `geometric` is a ground-truth ray-cast, **not a
sensor**. Never report a result from either as a measurement.

---

## 4. The recorded experiment — 90 episodes

This is the data behind Table 3 and the results section.

```bash
python scripts/with_webots.py --timeout 3000 --world worlds/DNS_Warehouse.wbt \
    tests/client_experiments.py -- \
    --dynamic-config config/scenario_warehouse.json \
    --static-config  config/scenario_warehouse_static.json \
    --out results/experiments_warehouse
```

**Expect:** `results/experiments_warehouse/` containing `experiment_results.csv`
(90 rows), `experiment_summary.csv`, `protocol.json` and `traces.json`.
Runtime: ~3 minutes.

`protocol.json` records the seeds and the absolute path of each policy
`model.npz` actually loaded, so a result can always be traced to the controller
that produced it.

---

## 5. Validation studies behind the figures

```bash
# moving-obstacle ground truth (Figure 4) -- 13 checks
python scripts/with_webots.py --world worlds/DNS_Warehouse.wbt \
    tests/client_moving_obstacle.py -- --config config/scenario_warehouse.json \
    --out results/moving_obstacle_warehouse.json

# sensor-mount ablation, stabilised vs body-fixed (Figure 5a)
python scripts/with_webots.py --timeout 700 --world worlds/DNS_Warehouse.wbt \
    tests/client_lidar_attitude.py -- --config config/scenario_warehouse.json \
    --out results/lidar_attitude_warehouse.json

# LiDAR angular calibration against known geometry
python scripts/with_webots.py --world worlds/DNS_Warehouse.wbt \
    tests/client_calibrate.py -- --config config/scenario_warehouse.json \
    --out results/calibration_warehouse.json
```

The mount ablation is the study written up in
[`SENSOR_ATTITUDE.md`](SENSOR_ATTITUDE.md): `body_fixed` measurably corrupts the
scan during manoeuvres, which is why `stabilised` is the configured default.

---

## 6. The demonstration video

```bash
python scripts/with_webots.py --timeout 520 --world worlds/DNS_Warehouse.wbt \
    tests/client_record_demo.py

python scripts/verify_mp4.py results/demo/DNS_Webots_Research_Demo.mp4 \
    --json results/demo/mp4_verification.json
```

**Expect:** `"verdict": "VALID"` with ~1255 frames at 1280×720 and over a
thousand distinct frame sizes.

Two failures in this environment make that verification mandatory, and both are
**silent** — every API call "succeeds" and a valid file appears:

1. **Webots' GUI viewport renders no geometry** when launched non-interactively
   from this harness. `Supervisor.exportImage` returned a uniform fill of the
   background colour at three different drone poses — three byte-identical
   2.9 MB files (`results/render_probe/render_probe.json`). The recorder is
   therefore unusable here; frames are captured from an in-scene `Camera` node,
   which does render (`results/camera_probe/camera_probe.json`: 143 → 564
   distinct colours, frames changing with the drone).
2. **Webots' bundled `ffmpeg.exe` does not run** (exit 53, no output, no
   `ffprobe` at all), so encoding goes through `imageio-ffmpeg`'s binary.

`scripts/verify_mp4.py` parses the ISO-BMFF container directly and reports
frame count and per-frame byte sizes — a blank or static capture collapses to
one or two distinct frame sizes. Never treat `movieIsReady()` /
`movieFailed()` as evidence.

---

## 7. Figures, tables and the report

Needs `matplotlib`. Run in this order — the report consumes the figures and
writes the tables.

```bash
python scripts/make_deliverable_figures.py --config config/scenario_warehouse.json
python scripts/make_deliverable_report.py  --config config/scenario_warehouse.json
```

**Writes:** `evidence/deliverable_figures/` (5 PNGs),
`evidence/deliverable_tables/` (3 CSVs),
`evidence/DNS_Simulation_Research_Report.pdf`, and a page-image preview in
`evidence/deliverable_report_preview/`.

The earlier research-demo round is regenerated with `make_figures.py` and
`make_report.py` against `config/scenario_research_demo.json`, writing to
`evidence/figures/`, `evidence/tables/` and `evidence/DNS_Research_Report.pdf`.

---

## 8. Assemble the deliverable package

```bash
python scripts/build_deliverable.py --zip        # then, into that package:
python scripts/make_verification.py
```

`build_deliverable.py` assembles `DNS_SIMULATION_RESEARCH_DELIVERABLE/`
**outside** this repository (one level up) and prints a per-file manifest with
`MISSING` lines for anything absent. `make_verification.py` then writes
`05_VERIFICATION/Verification_Report.md` and `test_log.txt` into it.

The package is a build artifact and is deliberately git-ignored — rebuild it,
don't commit it.

---

## Interface handoff

To show teammates exactly what the simulator emits:

```bash
python scripts/show_handoff.py results/two_frames_webots_physical_lidar.json
```

It prints the 12 sector ranges with their validity flags, then a
self-consistency block: frame separation 0.200000000 s, action scaling,
measured-vs-commanded velocity, sectors observed, the 52-value observation
length and the `navigation.Frame` key check.

Compatibility with the PPO side is checked by running **Spencer's own**
verifier against our packet, rather than by adjusting his code to fit ours:

```bash
python "$DNS_PPO_DIR/handoff.py" --file results/two_frames_webots_physical_lidar.json
```

---

## Known environment-specific notes

- `docs/M0_environment_audit.md` records the audited local machine, including
  that Webots sits at `D:\Program files\Webots` (lowercase "f" — there is no
  `C:\Program Files\Webots` on that machine). Treat it as a dated audit record,
  not as configuration.
- `protocol.json` and several `results/*.json` files embed the absolute paths
  that were in use when they were produced. That is intentional provenance;
  regenerating them on another machine rewrites the paths and does not change
  the measurements.
- Webots permits only one `Robot` instance per controller process. Reuse a
  single backend and re-point its scenario instead of constructing a second one.
- Grab camera frames from `backend.step_hook`, never by hand-stepping alongside
  `Navigation.step` — that double-advances the clock and trips the PPO side's
  "Unsynchronised frame" guard.
