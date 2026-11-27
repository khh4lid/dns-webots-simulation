## v2: S6 generalisation + transparency release

Closes the gap v1 could not: with only one obstacle layout, there was no way to
tell navigation competence from memorising one room.

### What is new
- **Seeded scenario generator** — one YAML config plus a seed gives a
  byte-identical scenario JSON (verified across separate processes), from which
  the world is written.
- **Six-layout shift ladder** L00–L05, each validated, A*-feasible and frozen
  before any controller saw it. En-route corridor widths 2.08 / 1.87 / 2.08 /
  1.31 / 0.85 / 0.68 m.
- **540 episodes = 480 distinct configurations** under a pre-registered protocol.
- **LiDAR–cart consistency** vs an independent ray caster: MAE 0.0193 m, n = 650.
- 14 ADRs, 4 role guides, 2 interface specs, 5 real videos, 7 GIFs, a 12-slide
  deck and a presenter kit.

### Headline results
| | |
|---|---|
| L00 reproduction vs v1 | **10/10 cells, max path delta 0 m** (session replayed in order; single episodes drift up to 0.287 m) |
| Mirror test (L00 vs L02) | 24/45 both — *no evidence of left/right memorisation in this test* |
| L05 (traversability confirmed) | **0/45** — the counterweight to the mirror result |
| L04 | **excluded** — the reference tracker cannot fly it either |

### Corrections to my own v1 numbers
- Mode B "3/3" is **1/1** — the episode seed is accepted but never used.
- The reactive explanation compared the crate's *centre* (1.54 m); the
  controller reacts to the *surface*, **1.0582 m**.
- "Cart error 0.00e+00 m" was true by construction — reframed and replaced.
- v1 shipped a `protocol.json` describing a different run than its `results.csv`.

### Checklist
- [x] `scripts/security_scan.py` — tree **and** full history: VERDICT CLEAN
- [x] `tests/test_generator.py` — 26/26
- [x] `scripts/run_gates.py` — 5/6 layouts PASS (L04 documented)
- [x] No file > 50 MB; GIFs ≤ 8 MB; MP4s are Release assets
- [x] Every number traced in `evidence/CLAIMS_EVIDENCE.csv`
- [x] Every visual carries a provenance label
- [x] No history rewrite, no force push; `v1.0` tagged and pushed
- [ ] Three identity details still to confirm (full legal name for `CITATION.cff`, Spencer's surname + handle, sensor teammate's name + handle) — shown as explicit gaps, not guesses
- [ ] Reviewer eye-check (below)

### Please eye-check on this branch
Images load · the 7 GIFs animate · the Mermaid flowchart renders · `[!NOTE]`
blocks appear as coloured callouts, not raw text · the 5 `<details>` sections
expand · role-table links resolve · readable in dark mode and at mobile width.
