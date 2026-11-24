# RELEASE_STATE.md — v2 transparency release, working state

Updated at the end of every stage so a fresh session can resume from here
without re-reading the whole history.

**Branch:** `v2-transparency-release` · **Base tag:** `v1.0` (annotated, on the
old `main`) · **Remote:** `khh4lid/dns-webots-simulation` (private)

---

## Stage status

| Stage | What it covers | Status |
|---|---|---|
| **A** | Freeze, tag, branch, security scans, file map, migration | **Done** |
| **B** | Missing visuals: GIFs, V1 demo MP4, F4, ADR images | Not started |
| **C** | Team docs, ADRs, interfaces v2 | Not started |
| **D** | README v2 | Not started |
| **E** | GitHub operations: commits, PR, merge, tags, releases, issues | Not started |
| **F** | Fresh-clone verification, link check, screenshots | Not started |
| **G** | Report PDF + slides (only after A–F) | Not started |

---

## Stage A — what was done

1. **Tag `v1.0`** created (annotated) on the previous `main`
   (`8badfcf`), message "state before the v2 S6 release". **No history was
   rewritten and nothing was force-pushed.**
2. **Branch `v2-transparency-release`** created from `main`.
3. **Security scan** (`scripts/security_scan.py`) over the working tree **and
   every blob reachable from any commit**:

   | Check | Result |
   |---|---|
   | GitHub / AWS / Google / Slack tokens, private keys, bearer tokens | **0 findings** |
   | Generic `secret=`/`password=` assignments | **0 findings** |
   | Google Drive file/folder IDs (Drive handoff builder, `config/`) | **0 findings** |
   | Secrets anywhere in git **history** | **0 findings** |
   | Files > 50 MB | **0** |
   | Personal absolute paths | 26 hits, **all Webots install paths, no usernames** |

   Triage of the 26: `dns/launcher.py` holds a *deliberate* list of common
   Webots install locations (a feature, kept); `scripts/build_deliverable.py`
   hard-coded one and now probes the same list instead (fixed); the README's
   example path is replaced in Stage D; the remaining 21 are inside generated
   result logs, which are machine output rather than instructions to follow.
   Full report: `results/security_scan.json`.
4. **`.gitattributes`** extended: `*.gif`, `*.webm`, `*.npy`, `*.ico` marked
   binary (a CRLF-translated GIF is silently corrupted); `*.svg` as LF text
   excluded from diffs.
5. **Migration applied** (`scripts/migrate_v2.py --apply`) — 4 renames via
   `git mv` with history preserved, 59 new paths. Full table:
   [`evidence/FILE_MAP.md`](FILE_MAP.md).
6. **`tools/` merged into `scripts/`** as the target structure requires;
   29 scripts now in one place, all cross-references updated.

### Key path changes

| Before | After |
|---|---|
| `results/experiments_warehouse/` | `results/v1_L00/` (git mv) |
| `results/s6/runs/` (untracked) | `results/v2_S6/runs/` — 540 run folders, 8.1 MB |
| `results/s6/` (untracked) | `results/v2_S6/_session_logs/` |
| `tools/*.py` | `scripts/*.py` |
| v2 package `07_SCENARIOS/` | `config/scenarios/` — 12 JSONs + `HASHES.txt` + config |
| v2 package `03_FIGURES/` | `assets/figures/` (12 PNG) + `docs/FIGURES.md` |
| v2 package `10_EVIDENCE/` | `docs/verification/` + `evidence/` |
| v2 package `06_DATA/` | `results/v2_S6/` |

### Deliberately not committed

- **Spencer's `DroneNavPPO/`** — his code and checkpoints are his to publish.
  Only the interface description and SHA-256 fingerprints are recorded here.
- **The sensor teammate's processing code** — not written by me, not mine to ship.

---

## Open issues carried into later stages

| # | Item | Stage |
|---|---|---|
| 1 | `gh` CLI **not installed** and no `GH_TOKEN`/`GITHUB_TOKEN` in the environment. Git push/tag work via the credential manager, but **API-only operations** (About text, topics, labels, issues, releases, collaborator invites) cannot be automated. | E |
| 2 | **Section 0 identity fields were all left in brackets** — my full name, Spencer's name + username, the sensor teammate's name + username, Dr. Mehdi's username. Docs will say **TBC**; collaborator invites cannot be sent. | C, E |
| 3 | C2 LiDAR–cart consistency and the cart physics-velocity check were never run. `results/v2_S6/runs/*/lidar.npz` holds sector ranges, pose and obstacle ground truth for 60 runs, so the consistency check is computable offline. | B |
| 4 | L04 excluded pending Dr. Mehdi's decision; needs an issue. | E |
| 5 | README still contains one machine-specific Webots path (line 429). | D |

---

## Next action

Stage B: produce the missing visuals. Inputs already present —
`results/v2_S6/runs/` (540 trajectories, 60 with `lidar.npz`),
`config/scenarios/` (12 layouts), `assets/figures/` (12 PNG),
`scripts/make_figures.py` + `scripts/dns_style.py` (palette already
CVD-validated).
