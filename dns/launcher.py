"""Start Webots and prepare the environment for an ``<extern>`` controller.

The drone in ``dns_arena.wbt`` uses ``controller "<extern>"``: Webots holds the
robot until a separate Python process attaches through ``libController``. That
keeps Spencer's PPO code, this package and the simulator in one process space
with no custom IPC.

Typical use::

    from dns.launcher import webots_session
    with webots_session() as session:
        from dns.backend import WebotsBackend     # import AFTER the env is set
        ...
"""
from __future__ import annotations

import contextlib
import os
import re
import shutil
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WORLD = PROJECT_ROOT / "worlds" / "dns_arena.wbt"
DEFAULT_ROBOT_NAME = "DnsDrone"

#: Checked in order when ``WEBOTS_HOME`` is unset.
_WEBOTS_CANDIDATES = (
    r"D:\Program Files\Webots",
    r"C:\Program Files\Webots",
    r"C:\Program Files (x86)\Webots",
    os.path.expandvars(r"%LOCALAPPDATA%\Programs\Webots"),
    "/usr/local/webots",
    "/Applications/Webots.app",
)


def find_webots_home() -> Path:
    home = os.environ.get("WEBOTS_HOME")
    if home and Path(home).is_dir():
        return Path(home)
    for candidate in _WEBOTS_CANDIDATES:
        path = Path(candidate)
        if (path / "resources" / "version.txt").is_file():
            return path
    raise FileNotFoundError(
        "Webots not found. Set WEBOTS_HOME to the installation directory."
    )


def webots_version(home: Path | None = None) -> str:
    home = home or find_webots_home()
    return (home / "resources" / "version.txt").read_text(encoding="utf8").strip()


def webots_binary(home: Path | None = None) -> Path:
    home = home or find_webots_home()
    for relative in ("msys64/mingw64/bin/webots.exe", "webots.exe", "webots", "Contents/MacOS/webots"):
        path = home / relative
        if path.is_file():
            return path
    found = shutil.which("webots")
    if found:
        return Path(found)
    raise FileNotFoundError(f"No webots executable under {home}")


def webots_ports_in_use() -> set[int]:
    """Ports already claimed by running Webots instances.

    Webots names its IPC endpoint ``webots-<port>-<robot>``, so the live pipe
    list is an exact record of which ports are taken. A developer with the
    Webots GUI already open normally holds the default 1234.
    """
    ports: set[int] = set()
    if sys.platform == "win32":
        try:
            for name in os.listdir(r"\\.\pipe"):
                match = re.match(r"webots-(\d+)-", name)
                if match:
                    ports.add(int(match.group(1)))
        except OSError:
            pass
    else:
        tmp = Path("/tmp")
        if tmp.is_dir():
            for entry in tmp.glob("webots-*"):
                match = re.match(r"webots-(\d+)", entry.name)
                if match:
                    ports.add(int(match.group(1)))
    return ports


def find_free_port(start: int = 1234, span: int = 60) -> int:
    """First port in ``[start, start + span)`` that no Webots instance holds.

    Picking a fresh port means an automated run never collides with -- or
    silently attaches to -- a Webots window the user already has open.
    """
    taken = webots_ports_in_use()
    for port in range(start, start + span):
        if port in taken:
            continue
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(("127.0.0.1", port))
            except OSError:
                continue
        return port
    raise RuntimeError(f"no free Webots port in [{start}, {start + span})")


def controller_env(home: Path | None = None, robot_name: str = DEFAULT_ROBOT_NAME,
                   port: int = 1234, base: dict[str, str] | None = None) -> dict[str, str]:
    """Environment variables an extern controller process needs."""
    home = home or find_webots_home()
    env = dict(os.environ if base is None else base)
    env["WEBOTS_HOME"] = str(home)

    python_lib = home / "lib" / "controller" / "python"
    existing = env.get("PYTHONPATH", "")
    parts = [str(python_lib)] + ([existing] if existing else [])
    env["PYTHONPATH"] = os.pathsep.join(parts)

    # libController plus its MinGW runtime dependencies must be loadable.
    path_parts = [str(home / "lib" / "controller")]
    mingw = home / "msys64" / "mingw64" / "bin"
    if mingw.is_dir():
        path_parts.append(str(mingw))
    env["PATH"] = os.pathsep.join(path_parts + [env.get("PATH", "")])

    env["WEBOTS_CONTROLLER_URL"] = f"ipc://{port}/{robot_name}"
    return env


def install_controller_env(home: Path | None = None, robot_name: str = DEFAULT_ROBOT_NAME,
                           port: int = 1234) -> None:
    """Apply :func:`controller_env` to this process.

    Must run **before** ``from controller import Supervisor``. On Windows the
    DLL directory is registered explicitly because Python 3.8+ ignores ``PATH``
    for extension-module dependencies.
    """
    env = controller_env(home, robot_name, port)
    for key in ("WEBOTS_HOME", "PYTHONPATH", "PATH", "WEBOTS_CONTROLLER_URL"):
        os.environ[key] = env[key]

    python_lib = Path(env["WEBOTS_HOME"]) / "lib" / "controller" / "python"
    if str(python_lib) not in sys.path:
        sys.path.insert(0, str(python_lib))

    if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
        for folder in (Path(env["WEBOTS_HOME"]) / "lib" / "controller",
                       Path(env["WEBOTS_HOME"]) / "msys64" / "mingw64" / "bin"):
            if folder.is_dir():
                with contextlib.suppress(OSError):
                    os.add_dll_directory(str(folder))


@dataclass
class Session:
    process: subprocess.Popen | None
    world: Path
    home: Path
    robot_name: str
    port: int
    log_path: Path | None = None

    def terminate(self, timeout: float = 10.0) -> None:
        if self.process is None or self.process.poll() is not None:
            return
        self.process.terminate()
        try:
            self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=timeout)

    def log_text(self) -> str:
        if self.log_path and self.log_path.is_file():
            return self.log_path.read_text(encoding="utf8", errors="replace")
        return ""


def start_webots(world: Path = DEFAULT_WORLD, *, headless: bool = True,
                 port: int = 1234, robot_name: str = DEFAULT_ROBOT_NAME,
                 log_path: Path | None = None, extra_args: list[str] | None = None) -> Session:
    """Launch Webots on ``world``. The caller is responsible for terminating it."""
    home = find_webots_home()
    binary = webots_binary(home)
    world = Path(world).resolve()
    if not world.is_file():
        raise FileNotFoundError(f"world not found: {world}")

    args = [str(binary), "--batch", "--stdout", "--stderr", f"--port={port}"]
    if headless:
        args += ["--mode=fast", "--no-rendering", "--minimize"]
    args += list(extra_args or [])
    args.append(str(world))

    env = controller_env(home, robot_name, port)
    stream = open(log_path, "w", encoding="utf8") if log_path else subprocess.DEVNULL
    process = subprocess.Popen(args, env=env, stdout=stream,
                               stderr=subprocess.STDOUT if log_path else subprocess.DEVNULL)
    return Session(process, world, home, robot_name, port, log_path)


@contextlib.contextmanager
def webots_session(world: Path = DEFAULT_WORLD, *, headless: bool = True,
                   port: int = 1234, robot_name: str = DEFAULT_ROBOT_NAME,
                   log_path: Path | None = None, settle_s: float = 2.0):
    """Start Webots, prepare this process to attach, and clean up afterwards."""
    session = start_webots(world, headless=headless, port=port,
                           robot_name=robot_name, log_path=log_path)
    try:
        install_controller_env(session.home, robot_name, port)
        time.sleep(settle_s)          # let Webots bind the IPC endpoint
        yield session
    finally:
        session.terminate()


def main(argv: list[str] | None = None) -> int:
    import argparse
    parser = argparse.ArgumentParser(description="Report the detected Webots installation")
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    args = parser.parse_args(argv)
    home = find_webots_home()
    print(f"WEBOTS_HOME : {home}")
    print(f"version     : {webots_version(home)}")
    print(f"binary      : {webots_binary(home)}")
    print(f"world       : {args.world}  (exists={args.world.is_file()})")
    print(f"python      : {sys.version.split()[0]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
