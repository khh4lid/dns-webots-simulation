"""Run the S6 experiment on every layout, one Webots session each, then merge.

One session per layout rather than one per layout x mode: Mode B and Mode C
share an identical static obstacle set, so the same loaded world serves both and
the B/C comparison is made on bit-identical geometry.

Run::

    python tools/run_s6.py
    python tools/run_s6.py L04 L05
"""
from __future__ import annotations

import csv
import json
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
OUT = PROJECT_ROOT / "results" / "s6"
LAYOUTS = ("L00", "L01", "L02", "L03", "L04", "L05")


def main() -> int:
    layouts = sys.argv[1:] or list(LAYOUTS)
    OUT.mkdir(parents=True, exist_ok=True)
    meta = {}
    for layout in layouts:
        started = time.time()
        command = [
            sys.executable, str(PROJECT_ROOT / "scripts" / "with_webots.py"),
            "--boot-wait", "12", "--timeout", "1500",
            "--world", f"worlds/DNS_{layout}_C.wbt",
            str(PROJECT_ROOT / "tests" / "client_s6_experiment.py"),
            "--", "--layout", layout,
        ]
        print(f"\n{'=' * 64}\n{layout}\n{'=' * 64}", flush=True)
        done = subprocess.run(command, cwd=str(PROJECT_ROOT),
                              capture_output=True, text=True, timeout=1800)
        for line in done.stdout.splitlines():
            if line.startswith(f"[{layout}]") or "goal " in line or "MODE" in line:
                print(line, flush=True)
        if done.returncode != 0:
            print(f"  !! {layout} exited {done.returncode}", flush=True)
            print(done.stdout[-3000:], flush=True)
            print(done.stderr[-3000:], flush=True)
        meta[layout] = {"exit_code": done.returncode,
                        "wall_s": round(time.time() - started, 1)}

    # ---- merge every per-layout CSV into results_v2.csv -------------------
    rows: list[dict] = []
    columns: list[str] = []
    for layout in LAYOUTS:
        path = OUT / f"results_{layout}.csv"
        if not path.is_file():
            continue
        with path.open(encoding="utf8") as handle:
            reader = csv.DictReader(handle)
            columns = reader.fieldnames or columns
            rows.extend(reader)
    if rows:
        (DATA).mkdir(parents=True, exist_ok=True)
        target = DATA / "results_v2.csv"
        with target.open("w", newline="", encoding="utf8") as handle:
            writer = csv.DictWriter(handle, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nwrote {target} ({len(rows)} episodes)")

    (OUT / "RUN_META.json").write_text(json.dumps(meta, indent=2), encoding="utf8")
    total = sum(v["wall_s"] for v in meta.values())
    print(f"total wall clock {total:.0f} s for {len(rows)} episodes")
    return 0 if all(v["exit_code"] == 0 for v in meta.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
