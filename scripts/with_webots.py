"""Run a Python client against a freshly launched Webots instance.

Webots is started on the DNS arena world, the client is executed in a child
process with ``libController`` on its path, and Webots is shut down afterwards.
The client runs as a subprocess so a failed connection times out instead of
hanging the harness -- ``Supervisor()`` blocks forever when no matching robot
is waiting.

Usage::

    python scripts/with_webots.py tests/client_smoke.py
    python scripts/with_webots.py --gui --timeout 300 tests/client_validation.py -- --json results/v.json
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from dns.launcher import (  # noqa: E402
    DEFAULT_ROBOT_NAME, DEFAULT_WORLD, controller_env, find_free_port,
    find_webots_home, start_webots, webots_ports_in_use, webots_version,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("client", type=Path, help="Python script to run against the simulator")
    parser.add_argument("client_args", nargs="*", help="arguments forwarded to the client")
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--robot", default=DEFAULT_ROBOT_NAME)
    parser.add_argument("--port", type=int, default=None,
                        help="Webots IPC port; default picks the first free one "
                             "so an already-open Webots window is left untouched")
    parser.add_argument("--gui", action="store_true", help="show the Webots window")
    parser.add_argument("--webots-arg", action="append", default=[],
                        metavar="ARG",
                        help="extra flag passed straight to webots.exe, "
                             "repeatable (e.g. --webots-arg=--mode=realtime). "
                             "Movie recording needs real-time mode, otherwise "
                             "the simulation outruns the renderer")
    parser.add_argument("--timeout", type=float, default=240.0, help="client timeout, seconds")
    parser.add_argument("--boot-wait", type=float, default=6.0,
                        help="seconds to wait for Webots to bind its IPC endpoint")
    parser.add_argument("--webots-log", type=Path, default=PROJECT_ROOT / "results" / "webots.log")
    args = parser.parse_args(argv)

    client = args.client if args.client.is_absolute() else (PROJECT_ROOT / args.client)
    if not client.is_file():
        parser.error(f"client script not found: {client}")

    home = find_webots_home()
    args.webots_log.parent.mkdir(parents=True, exist_ok=True)
    busy = sorted(webots_ports_in_use())
    port = args.port if args.port is not None else find_free_port()
    print(f"[with_webots] Webots {webots_version(home)} at {home}", flush=True)
    print(f"[with_webots] world  : {args.world}", flush=True)
    print(f"[with_webots] client : {client.name} {' '.join(args.client_args)}", flush=True)
    print(f"[with_webots] port   : {port}"
          + (f"  (already in use by other Webots instances: {busy})" if busy else ""), flush=True)

    session = start_webots(args.world, headless=not args.gui, port=port,
                           robot_name=args.robot, log_path=args.webots_log,
                           extra_args=args.webots_arg)
    code = 1
    try:
        time.sleep(args.boot_wait)
        if session.process is not None and session.process.poll() is not None:
            print(f"[with_webots] Webots exited early with {session.process.returncode}", flush=True)
            print(session.log_text(), flush=True)
            return 2

        env = controller_env(home, args.robot, port)
        env["PYTHONPATH"] = f"{PROJECT_ROOT}{';' if sys.platform == 'win32' else ':'}{env['PYTHONPATH']}"
        env["PYTHONUNBUFFERED"] = "1"
        try:
            completed = subprocess.run(
                [sys.executable, str(client), *args.client_args],
                env=env, cwd=str(PROJECT_ROOT), timeout=args.timeout)
            code = completed.returncode
        except subprocess.TimeoutExpired:
            print(f"[with_webots] client timed out after {args.timeout}s", flush=True)
            code = 124
    finally:
        session.terminate()

    log = session.log_text().strip()
    if log:
        print("[with_webots] ---- Webots log ----", flush=True)
        print(log, flush=True)
    print(f"[with_webots] client exit code {code}", flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
