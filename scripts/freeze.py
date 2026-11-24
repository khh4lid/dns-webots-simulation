"""Phase 0 freeze: SHA-256 manifest of everything that must not change, plus
a machine description.

Two outputs, both written into the v2 package's evidence folder:

``BASELINE_HASHES.txt``
    One line per file, ``sha256  size_bytes  relative/path``, over the frozen v1
    deliverable, its ZIP, the baseline snapshot, Spencer's whole DroneNavPPO
    tree and the three PPO checkpoints. The point is to be able to *prove*, at
    the end of the work, that none of it moved. ``--verify`` re-reads the file
    and reports any hash that changed, any file that disappeared and any file
    that appeared.

``ENVIRONMENT.txt``
    Windows build, CPU, GPU, Webots version, Python version and ``pip freeze``.
    A result that cannot be tied to a machine cannot be reproduced on another.

Run::

    python scripts/freeze.py --write
    python scripts/freeze.py --verify
"""
from __future__ import annotations

import argparse
import hashlib
import os
import platform
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]          # .../webots_dns
WORKSPACE = PROJECT_ROOT.parent                             # .../drone_navigation_system
PACKAGE = WORKSPACE / "DNS_FRIDAY_MEETING_2026-11-27"
EVIDENCE = PACKAGE / "10_EVIDENCE"

#: Trees and files that are frozen for the duration of this work. Each entry is
#: (label, path). Directories are walked; single files are hashed directly.
FROZEN = (
    ("v1_deliverable", WORKSPACE / "DNS_SIMULATION_RESEARCH_DELIVERABLE"),
    ("v1_zip", WORKSPACE / "DNS_SIMULATION_RESEARCH_DELIVERABLE.zip"),
    ("baseline_snapshot", WORKSPACE / "webots_dns_BASELINE_SNAPSHOT_2026-11-24"),
    ("spencer_ppo", WORKSPACE / "DroneNavPPO" / "DroneNavPPO"),
)

#: Called out separately in the report, so hashed under their own labels too.
NAMED = (
    ("handoff.py", WORKSPACE / "DroneNavPPO" / "DroneNavPPO" / "handoff.py"),
    ("navigation.py", WORKSPACE / "DroneNavPPO" / "DroneNavPPO" / "navigation.py"),
    ("evaluate.py", WORKSPACE / "DroneNavPPO" / "DroneNavPPO" / "evaluate.py"),
    ("ppo.py", WORKSPACE / "DroneNavPPO" / "DroneNavPPO" / "ppo.py"),
    ("planar.py", WORKSPACE / "DroneNavPPO" / "DroneNavPPO" / "planar.py"),
    ("checkpoint_seed7",
     WORKSPACE / "DroneNavPPO" / "DroneNavPPO" / "evidence" / "seed7" / "model.npz"),
    ("checkpoint_seed19",
     WORKSPACE / "DroneNavPPO" / "DroneNavPPO" / "evidence" / "seed19" / "model.npz"),
    ("checkpoint_seed43",
     WORKSPACE / "DroneNavPPO" / "DroneNavPPO" / "evidence" / "seed43" / "model.npz"),
)

#: Skipped when walking a tree: caches are regenerated and carry no evidence.
_SKIP_DIRS = {"__pycache__", ".git", ".pytest_cache"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def walk(root: Path):
    """Yield every file under ``root``, caches excluded, in sorted order."""
    if root.is_file():
        yield root
        return
    for directory, subdirs, files in os.walk(root):
        subdirs[:] = sorted(d for d in subdirs if d not in _SKIP_DIRS)
        for name in sorted(files):
            yield Path(directory) / name


def manifest() -> list[tuple[str, str, int, str]]:
    """``(label, sha256, size_bytes, path_relative_to_workspace)`` for every
    frozen file. Missing roots are recorded explicitly rather than skipped --
    a vanished tree is exactly the kind of change this is meant to catch."""
    rows: list[tuple[str, str, int, str]] = []
    for label, root in FROZEN + NAMED:
        if not root.exists():
            rows.append((label, "MISSING", -1, str(root.relative_to(WORKSPACE)).replace("\\", "/")))
            continue
        for path in walk(root):
            rows.append((label, sha256(path), path.stat().st_size,
                         str(path.relative_to(WORKSPACE)).replace("\\", "/")))
    return rows


def _run(command: list[str]) -> str:
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=120)
        return (done.stdout or done.stderr or "").strip()
    except Exception as error:                                   # noqa: BLE001
        return f"<failed: {error}>"


def webots_binary() -> Path | None:
    home = os.environ.get("WEBOTS_HOME")
    if not home:
        return None
    for relative in ("msys64/mingw64/bin/webots.exe", "webots.exe"):
        candidate = Path(home) / relative
        if candidate.is_file():
            return candidate
    return None


def environment_text() -> str:
    binary = webots_binary()
    lines = [
        "DNS v2 -- execution environment",
        "=" * 60,
        "",
        "[machine]",
        f"  os                 : {platform.system()} {platform.release()} "
        f"build {platform.version()}",
        f"  os_product         : {_run(['powershell', '-NoProfile', '-Command', '(Get-CimInstance Win32_OperatingSystem).Caption'])}",
        f"  cpu                : {_run(['powershell', '-NoProfile', '-Command', '(Get-CimInstance Win32_Processor).Name'])}",
        f"  cpu_cores_logical  : {os.cpu_count()}",
        f"  gpu                : {_run(['powershell', '-NoProfile', '-Command', '(Get-CimInstance Win32_VideoController).Name'])}",
        f"  machine            : {platform.machine()}",
        "",
        "[webots]",
        f"  WEBOTS_HOME        : {os.environ.get('WEBOTS_HOME', '<unset>')}",
        f"  binary             : {binary if binary else '<not found>'}",
        f"  version            : {_run([str(binary), '--version']) if binary else '<not found>'}",
        "",
        "[python]",
        f"  version            : {sys.version.splitlines()[0]}",
        f"  executable         : {sys.executable}",
        f"  implementation     : {platform.python_implementation()}",
        "",
        "[pip freeze]",
    ]
    frozen = _run([sys.executable, "-m", "pip", "freeze"])
    lines += [f"  {line}" for line in frozen.splitlines()] or ["  <none>"]
    return "\n".join(lines) + "\n"


HEADER = """# DNS v2 baseline hash manifest
#
# Everything listed here is FROZEN for the duration of the pre-meeting work:
# the v1 deliverable and its ZIP, the baseline snapshot, Spencer's DroneNavPPO
# tree (his source is the authoritative interface spec) and the three trained
# PPO checkpoints. Re-run `python scripts/freeze.py --verify` at the freeze to
# prove none of it was modified.
#
# format: <label>  <sha256>  <size_bytes>  <path relative to drone_navigation_system/>
"""


def write() -> int:
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    rows = manifest()
    target = EVIDENCE / "BASELINE_HASHES.txt"
    body = "".join(f"{label}  {digest}  {size}  {path}\n"
                   for label, digest, size, path in rows)
    target.write_text(HEADER + body, encoding="utf8")

    (EVIDENCE / "ENVIRONMENT.txt").write_text(environment_text(), encoding="utf8")

    missing = [r for r in rows if r[1] == "MISSING"]
    print(f"wrote {target}  ({len(rows)} entries, "
          f"{sum(r[2] for r in rows if r[2] > 0) / 1e6:.2f} MB)")
    print(f"wrote {EVIDENCE / 'ENVIRONMENT.txt'}")
    for label, root in FROZEN + NAMED:
        count = sum(1 for r in rows if r[0] == label and r[1] != "MISSING")
        print(f"  {label:<20} {count:>4} files")
    if missing:
        print(f"WARNING: {len(missing)} frozen root(s) missing")
        return 1
    return 0


def verify() -> int:
    target = EVIDENCE / "BASELINE_HASHES.txt"
    if not target.is_file():
        print(f"FAIL: {target} does not exist; run --write first")
        return 1
    recorded: dict[str, tuple[str, int]] = {}
    for line in target.read_text(encoding="utf8").splitlines():
        if not line or line.startswith("#"):
            continue
        _label, digest, size, path = line.split("  ", 3)
        recorded[path] = (digest, int(size))

    current = {path: (digest, size) for _l, digest, size, path in manifest()}

    changed = [p for p in recorded if p in current and current[p] != recorded[p]]
    vanished = [p for p in recorded if p not in current]
    appeared = [p for p in current if p not in recorded]

    for label, items in (("CHANGED", changed), ("MISSING", vanished),
                         ("NEW", appeared)):
        for path in sorted(items):
            print(f"{label}: {path}")
    verdict = "PASS" if not (changed or vanished) else "FAIL"
    print(f"\n{verdict}: {len(recorded)} recorded, {len(changed)} changed, "
          f"{len(vanished)} missing, {len(appeared)} new")
    if appeared and not (changed or vanished):
        print("NOTE: new files under a frozen root are reported but do not fail "
              "the check unless they shadow a recorded path.")
    return 0 if verdict == "PASS" else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--write", action="store_true",
                       help="record the manifest and the environment")
    group.add_argument("--verify", action="store_true",
                       help="re-hash and report any change")
    args = parser.parse_args()
    return write() if args.write else verify()


if __name__ == "__main__":
    raise SystemExit(main())
