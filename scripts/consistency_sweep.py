"""Consistency sweep for the meeting build.

Three classes of statement are made consistent across the README, the slides,
the ADRs and the verification docs, because a reader who finds two different
numbers for the same quantity cannot tell which one the environment actually
implements.

1. **Scan-plane height.** Flight altitude is 1.0 m and the LiDAR mount offset
   read from ``protos/DnsQuadcopter.proto`` is 0.14 m, so the scan plane rides
   at 1.14 m. The current build already says so; the v1-era documents describe a
   superseded configuration (0.075 m mount, 1.2 m walls, 1.075 m plane) and get
   a header pointing at the current value rather than being rewritten --
   silently editing a historical record is worse than annotating it.

2. **Reproducibility wording.** "0 m reproduction" is true only for a session
   replayed in order. Every unqualified occurrence is replaced by the qualified
   form, which states both arms.

3. **Independence wording.** A Wilson interval assumes independent trials. The
   15 cart-phase variants in a cell are coupled through residual airframe
   state, so the intervals are approximate and are now labelled as such.

Idempotent: running it twice changes nothing the second time.

Run::  python scripts/consistency_sweep.py [--check]
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

#: Verbatim records that must never be edited: the brief as received, and the
#: frozen v1 corrections ledger's historical "what v1 said" column.
PROTECTED = {"evidence/TASK_PROMPT_v2.md"}

#: v1-era documents describing a superseded sensor configuration. Annotated,
#: not rewritten.
V1_ERA = {
    "docs/SIMULATION_METHODS.md": "1.075 m scan plane (0.075 m mount)",
    "docs/SENSOR_ATTITUDE.md": "1.2 m walls",
    "docs/VALIDATION_EVIDENCE.md": "1.2 m walls",
}

V1_ERA_BANNER = (
    "> [!NOTE]\n"
    "> **Historical document — v1 configuration.** The numbers below describe\n"
    "> the configuration as it was when this was written ({what}). The current\n"
    "> build flies at **1.0 m** with a **0.14 m** LiDAR mount, so the scan plane\n"
    "> rides at **1.14 m**; see `protos/DnsQuadcopter.proto` (`lidarHeight`) and\n"
    "> `config/scenarios/L00_C.json`. Kept unedited as the record of what was\n"
    "> measured at the time.\n\n")

#: The qualified reproducibility sentence, used wherever the bare claim appeared.
REPRO_LONG = (
    "reproduced exactly when the session is replayed in order "
    "(path delta ≤ 5 µm); single episodes run in isolation can drift up "
    "to 0.287 m (outcomes 5/5 unchanged, step counts 4/5)")

#: Sentinel phrase proving a reproducibility edit (or its generator-side
#: equivalent) is already in place. Checked before the exact-match anchor so a
#: file whose generator now emits the qualified wording is not reported as a
#: missing anchor.
REPRO_SENTINEL = "replayed in order"

#: (relative path, exact old text, new text). Exact strings, so a near-miss
#: fails loudly instead of mangling a sentence.
EDITS: list[tuple[str, str, str]] = [
    ("README.md",
     "| L00 reproduction | re-run v1's protocol on the regenerated world | "
     "**10/10 cells, 0 m path difference** |",
     "| L00 reproduction | re-run v1's protocol on the regenerated world | "
     "**10/10 cells**, 0 m path difference *when the session is replayed in "
     "order*; a single episode in isolation can drift up to 0.287 m "
     "([detail](docs/verification/RESET_DETERMINISM.md)) |"),
    (".github/PR_BODY.md",
     "| L00 reproduction vs v1 | **10/10 cells, max path delta 0 m** |",
     "| L00 reproduction vs v1 | **10/10 cells, max path delta 0 m** "
     "(session replayed in order; single episodes drift up to 0.287 m) |"),
    ("CHANGELOG.md",
     "- v1's L00 protocol reproduces exactly: **10/10 cells, maximum "
     "per-episode path\n  difference 0 m**.",
     "- v1's L00 protocol reproduces **10/10 cells with a maximum per-episode\n"
     "  path difference of 0 m when the session is replayed in order**. A single\n"
     "  episode re-run in isolation drifts up to 0.287 m -- see\n"
     "  `docs/verification/RESET_DETERMINISM.md`."),
    ("docs/decisions/ADR-001-webots-r2025a-simulator.md",
     "assumed: re-running the v1 protocol on the regenerated world reproduced "
     "all ten\ncells with a **maximum per-episode path difference of 0 m**",
     "assumed: re-running the v1 protocol on the regenerated world reproduced "
     "all ten\ncells with a **maximum per-episode path difference of 0 m, with "
     "the session\nreplayed in order**. Determinism is a property of the "
     "sequence, not of an\nepisode in isolation -- a single episode re-run alone "
     "drifts up to 0.287 m\n(`docs/verification/RESET_DETERMINISM.md`)"),
    ("docs/verification/AUDIT_SUMMARY.md",
     "L00 protocol reproduces exactly, the episode-variant semantics are now",
     "L00 protocol reproduces exactly when replayed in order, the "
     "episode-variant semantics are now"),
    ("docs/decisions/DECISIONS_LOG.md",
     "identical, max path difference 0 m). Fresh phases would have made any",
     "identical, max path difference 0 m, session replayed in order). Fresh "
     "phases would have made any"),
]

#: Independence caveat, appended once under each table of Wilson intervals.
INDEPENDENCE = (
    "\n> [!IMPORTANT]\n"
    "> **The Wilson intervals are approximate.** A Wilson interval assumes\n"
    "> independent trials. The 15 cart-phase variants inside a cell are run back\n"
    "> to back and are coupled through residual airframe state across the reset\n"
    "> (`docs/verification/RESET_DETERMINISM.md`), so they are not strictly\n"
    "> independent draws. The deliberate cart-phase variation is far larger than\n"
    "> the coupling, but the intervals should be read as indicative rather than\n"
    "> exact.\n")

INDEPENDENCE_ANCHORS = [
    ("README.md",
     "> - **No significance tests.** Six layouts with one seed per family is a "
     "pilot, not a distribution; no trend is fitted."),
    ("docs/results/RESULTS_v2.md",
     "Per-checkpoint values are in `results/v2_S6/pooled_rates.csv`. They are "
     "**never ranked**: on L00 Mode C they are 9/15, 8/15 and 7/15, a spread "
     "about the size of the effect, with Wilson intervals that overlap almost "
     "completely."),
    ("docs/team/FOR_SUPERVISOR.md",
     "*PPO pooled over the three checkpoints; n = 45 distinct cart phases per "
     "layout\nin Mode C. Per-seed values are plotted but never ranked — their "
     "intervals\noverlap. Source: `results/v2_S6/pooled_rates.csv`.*"),
]


def apply(check: bool) -> int:
    changed, missing = [], []

    # --- 1. v1-era banners ------------------------------------------------
    for relative, what in V1_ERA.items():
        path = ROOT / relative
        if not path.is_file():
            missing.append(f"{relative} (absent)")
            continue
        text = path.read_text(encoding="utf8")
        if "Historical document — v1 configuration" in text:
            continue
        lines = text.split("\n")
        insert = 1 if lines and lines[0].startswith("#") else 0
        banner = V1_ERA_BANNER.format(what=what)
        new = "\n".join(lines[:insert]) + "\n\n" + banner + "\n".join(lines[insert:])
        if not check:
            path.write_text(new, encoding="utf8")
        changed.append(f"{relative}: v1-era banner")

    # --- 2. reproducibility wording ---------------------------------------
    for relative, old, new in EDITS:
        path = ROOT / relative
        if relative in PROTECTED:
            continue
        if not path.is_file():
            missing.append(f"{relative} (absent)")
            continue
        text = path.read_text(encoding="utf8")
        if new in text or REPRO_SENTINEL in text:
            continue
        if old not in text:
            missing.append(f"{relative}: anchor not found")
            continue
        if not check:
            path.write_text(text.replace(old, new, 1), encoding="utf8")
        changed.append(f"{relative}: reproducibility wording")

    # --- 3. independence caveat -------------------------------------------
    for relative, anchor in INDEPENDENCE_ANCHORS:
        path = ROOT / relative
        if not path.is_file():
            missing.append(f"{relative} (absent)")
            continue
        text = path.read_text(encoding="utf8")
        if "The Wilson intervals are approximate" in text:
            continue
        if anchor not in text:
            missing.append(f"{relative}: independence anchor not found")
            continue
        if not check:
            path.write_text(text.replace(anchor, anchor + "\n" + INDEPENDENCE, 1),
                            encoding="utf8")
        changed.append(f"{relative}: independence caveat")

    for item in changed:
        print(("WOULD CHANGE  " if check else "changed  ") + item)
    for item in missing:
        print(f"!! {item}")
    print(f"\n{len(changed)} edit(s), {len(missing)} problem(s)")
    return 1 if missing else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="report what would change without writing")
    return apply(parser.parse_args().check)


if __name__ == "__main__":
    raise SystemExit(main())
