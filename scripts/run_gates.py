"""Run the blocking layout gate on every layout, one Webots session each.

A layout that does not pass here is blocked from the S6 experiment. The summary
is written to ``results/layout_gate/SUMMARY.json`` and folded into
``Verification_Report_v2.md``.

Run::

    python scripts/run_gates.py
    python scripts/run_gates.py L03 L04
"""
from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
GATE_DIR = PROJECT_ROOT / "results" / "layout_gate"
LAYOUTS = ("L00", "L01", "L02", "L03", "L04", "L05")


def main() -> int:
    layouts = sys.argv[1:] or list(LAYOUTS)
    GATE_DIR.mkdir(parents=True, exist_ok=True)
    summary = {}
    for layout in layouts:
        started = time.time()
        command = [
            sys.executable, str(PROJECT_ROOT / "scripts" / "with_webots.py"),
            "--boot-wait", "12", "--timeout", "300",
            "--world", f"worlds/DNS_{layout}_C.wbt",
            str(PROJECT_ROOT / "tests" / "client_layout_gate.py"),
            "--", "--layout", layout,
        ]
        done = subprocess.run(command, cwd=str(PROJECT_ROOT),
                              capture_output=True, text=True, timeout=420)
        for line in done.stdout.splitlines():
            if line.startswith(f"[{layout}]") or "FAIL" in line:
                print(line, flush=True)
        report_path = GATE_DIR / f"{layout}.json"
        verdict = "NOT RUN"
        detail = {}
        if report_path.is_file():
            report = json.loads(report_path.read_text(encoding="utf8"))
            verdict = report["verdict"]
            detail = {
                "contract_tests": f"{report['contract_tests_passed']}/"
                                  f"{report['contract_tests_total']}",
                "reference_traversal_outcome":
                    report["reference_traversal"]["outcome"],
                "reference_traversal_steps":
                    report["reference_traversal"].get("steps"),
                "max_t0_pose_error_m":
                    report["world_check"]["checks"]["obstacle_t0_pose_max_error_m"],
                "scenario_sha256": report["scenario_sha256"],
            }
        summary[layout] = {"verdict": verdict, "exit_code": done.returncode,
                           "wall_s": round(time.time() - started, 1), **detail}
        print(f"  -> {layout}: {verdict} ({summary[layout]['wall_s']} s)\n",
              flush=True)

    (GATE_DIR / "SUMMARY.json").write_text(json.dumps(summary, indent=2),
                                           encoding="utf8")
    blocked = [k for k, v in summary.items() if v["verdict"] != "PASS"]
    print("=" * 62)
    for layout, entry in summary.items():
        print(f"{layout}  {entry['verdict']:<6} contracts="
              f"{entry.get('contract_tests', '-'):<7} "
              f"traversal={entry.get('reference_traversal_outcome', '-'):<10} "
              f"t0_err={entry.get('max_t0_pose_error_m', float('nan')):.1e} m")
    print(f"\n{len(summary) - len(blocked)}/{len(summary)} layouts cleared"
          + (f"; BLOCKED: {blocked}" if blocked else ""))
    return 1 if blocked else 0


if __name__ == "__main__":
    raise SystemExit(main())
