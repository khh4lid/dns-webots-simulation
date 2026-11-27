# RELEASE_STATE.md — FROZEN for the 27 Nov 2026 meeting

> [!IMPORTANT]
> **Frozen 26 Nov 2026, 20:46 +0800 at commit `f246099`, tag
> `meeting-2026-11-27`.** No code or data changes until after the meeting.
> Build manifest with hashes: [`MEETING_BUILD.txt`](MEETING_BUILD.txt).
> Remaining GitHub actions are browser-only: [`../GITHUB_WEB_STEPS.md`](../GITHUB_WEB_STEPS.md).

## The one rule this build keeps

Results, code, README and slides all describe **one** environment: the
`settle_tolerance` reset. The deterministic reset exists in the code as a
non-default option and has its own 540-episode cohort, but **nothing in the
deck is derived from it**. The fix lives on `reset-fix-v2.1`.

| Branch | Commit | Role |
|---|---|---|
| `main` | `8badfcf` | pre-release; PR open against it |
| `v2-transparency-release` | `f246099` | **the meeting build** |
| `reset-fix-v2.1` | `5ef04c3` | deterministic reset, awaiting re-validation |

---

## Stage status

| Stage | What it covers | Status |
|---|---|---|
| **A** | Freeze, tag, branch, security scans, file map, migration | **Done** |
| **B** | Visuals: 4 GIFs, demo MP4 + poster, F4, mirror evidence, ADR images, C2 | **Done** |
| **C** | 14 ADRs, 4 role guides, 2 interfaces, CHANGELOG/CITATION/CONTRIBUTING, folder READMEs | **Done** |
| **D** | README v2, RESULTS_v2, reproduce.py, Watch section + new figures | **Done** |
| **Videos** | V-TOP (L00 / L02 / L05), V-SIDE, obstacle catalogue, side elevation | **Done** |
| **Slides** | 12 + 6 backups, PPTX + PDF, speaker notes, 18 PNG previews | **Done — names pending** |
| **Presenter kit** | TALKING_POINTS.md: 60 s, 3 min, timing, Q&A, fallbacks | **Done** |
| **E** | Branch and tag **pushed**; PR, merge, v2.0 tag, releases, issues, About/topics | **Blocked — needs `gh` or a token** |
| **F** | Security scan CLEAN, link check clean; fresh-clone repro + screenshots | Partial |
| **G** | Report PDF + slides | Not started |

## Where it stands

- Branch `v2-transparency-release` is pushed (7 commits); tag `v1.0` is pushed.
- All four presentation clips reproduce their logged episode to **4.99e-06 m**
  after the capture client replays the session prefix. Without that replay a
  fresh single-episode session drifts 0.287 m, because the simulator is
  deterministic given the episode SEQUENCE rather than in isolation.
- `gh` re-checked at 06:13 on 25 Nov: still not on PATH and not in any standard
  install location. Remaining API steps are in `GITHUB_TODO.md`.
- **Names were never supplied.** The deck and docs carry visible `NAME NEEDED`
  / `TBC` placeholders; the deck must not be treated as final until filled.
- **PR not opened**: `gh` is not installed and no `GH_TOKEN`/`GITHUB_TOKEN` is
  in the environment, so no GitHub API call can be made. Open it at
  <https://github.com/khh4lid/dns-webots-simulation/pull/new/v2-transparency-release>
- Nothing was merged and no history was rewritten. `main` is untouched.

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
| 2 | **Section 0 identity fields were all left in brackets** — my full name, Spencer's name + username, the sensor teammate's name + username, the supervisor's username. Docs will say **TBC**; collaborator invites cannot be sent. | C, E |
| 3 | C2 LiDAR–cart consistency and the cart physics-velocity check were never run. `results/v2_S6/runs/*/lidar.npz` holds sector ranges, pose and obstacle ground truth for 60 runs, so the consistency check is computable offline. | B |
| 4 | L04 excluded pending the supervisor's decision; needs an issue. | E |
| 5 | README still contains one machine-specific Webots path (line 429). | D |

---

## 27 November 2026 — anonymisation, simple deck, paper section

Tagged **`meeting-2026-11-27-v2`** on `v2-transparency-release`. Environment
code and experiment data were frozen for this session; only presentation and
documentation changed.

### Part A — the supervisor's name removed

All 56 occurrences across 22 tracked files now read "the supervisor", or
"(name withheld by request)" where a document has a named-person field.
`scripts/anonymise_supervisor.py` performs and re-checks the substitution; it
edits the document *generators* as well as their output, so regeneration cannot
reintroduce the name.

Verified **0 hits** in tracked text files and **0 hits** in the extracted text
of all 17 PDFs and PPTXs in the tree. The two research-report PDFs were
regenerated. **Published git history and the existing tags still contain the
name; history was not rewritten.**

### Part B — the simple deck

`scripts/make_slides_simple.py` builds `DNS_SimEnv_SIMPLE_2026-11-27.pptx/.pdf`:
12 picture-first slides plus 4 backups, ~10 minutes, speaker notes on every
slide. **This is now the deck to present from**; the detailed 19-slide deck is
the backup. `MEETING_DAY.md` was updated to say so, and
`TALKING_POINTS_SIMPLE_AR.md` gives an Arabic walkthrough.

The simplicity rules are enforced by the build: headlines are measured against
the slide width and wrapped rather than shrunk, and `audit()` fails the build if
a slide's prose exceeds 16 words. Eight of sixteen slides carry no prose at all.

### Part C — the paper section

`docs/paper/simulation_environment_section.md` (+ `.docx`, PDF preview,
`references.bib`, `figures/`): sections X.1-X.12, **3,545 prose words**, 6
figures, 4 tables, 15 references all carrying a DOI or arXiv ID and all cited,
renumbered to IEEE first-appearance order by script.

### Claims checks — all three at 0 unmatched

| Target | Tokens | Unmatched |
|---|---|---|
| `evidence/SLIDES_SIMPLE_CLAIMS_CHECK.csv` | 134 | 0 |
| `evidence/SLIDES_CLAIMS_CHECK.csv` | 288 | 0 |
| `evidence/PAPER_SECTION_CLAIMS_CHECK.csv` | 194 | 0 |

`scripts/check_slide_claims.py` gained `--source`/`--out`/`--markdown`, now
matches numbers by value rather than by string form, walks the scenario JSON so
geometry traces to the file that defines it, and skips docstrings, hex colours
and matplotlib style strings, which are code and not claims.

### One deviation from the freeze, stated plainly

`dns/__init__.py` is inside a frozen directory and its **module docstring**
named the supervisor. The line was changed. It is a comment: 46/46 and 26/26
tests still pass and the six scenario files still regenerate byte-identically.
`results/`, `config/`, `worlds/`, `protos/` and `controllers/` are untouched,
hash-for-hash.

### New in this session

- `evidence/DEFECTS.csv` — the six defects with status, fix and regression
  test. Simple slide 11 reads its tally from this file rather than asserting it.
- `scripts/anonymise_supervisor.py`, `scripts/make_slides_simple.py`,
  `scripts/make_paper_section.py`.


---

## Next action

1. Fill the four names, then re-run `scripts/make_team_docs.py`,
   `scripts/make_slides.py` and `scripts/make_support_docs.py`.
2. Work `GITHUB_TODO.md` once `gh` is on PATH: PR, merge, `v2.0`, both
   releases, About/topics/labels/issues, collaborators.
3. Outstanding, none blocking Friday: V-OBS orbit video, the composite and
   showreel, `evidence/REPRO_CHECK.md` (fresh-clone `reproduce.py --fast`),
   and Phase 5's progress report.
4. Paper section: hand `docs/paper/simulation_environment_section.docx` to the
   team and merge `docs/paper/references.bib` into the paper bibliography.
   Section numbers are written `X.n` for the editor to renumber.
