"""Verify an MP4 without external tools, by parsing the ISO-BMFF container.

Webots ships an ``ffmpeg.exe`` that does not run in this environment (exit 53,
no output), and ``ffprobe`` is absent, so "is this video real?" is answered here
by reading the file's own index instead of by decoding it.

What is read:

* ``moov/mvhd``                 -> timescale and duration
* ``moov/trak/tkhd``            -> track dimensions
* ``moov/trak/mdia/minf/stbl``  -> ``stsz`` sample sizes (one per frame) and
                                   ``stts`` time-to-sample (frame timing)

Frame *sizes* are the useful signal for "do the frames actually change":
inter-frame compression makes a static scene collapse to a handful of bytes per
frame, so a video of a genuinely moving simulation has large, varying samples.

Usage::

    python scripts/verify_mp4.py path/to/movie.mp4
    python scripts/verify_mp4.py path/to/movie.mp4 --json report.json
"""
from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

CONTAINERS = {b"moov", b"trak", b"mdia", b"minf", b"stbl", b"edts", b"udta"}


def walk(data: bytes, start: int, end: int, depth: int = 0):
    """Yield ``(type, payload_start, payload_end, depth)`` for each box."""
    offset = start
    while offset + 8 <= end:
        size = struct.unpack(">I", data[offset:offset + 4])[0]
        kind = data[offset + 4:offset + 8]
        header = 8
        if size == 1:                       # 64-bit extended size
            if offset + 16 > end:
                return
            size = struct.unpack(">Q", data[offset + 8:offset + 16])[0]
            header = 16
        elif size == 0:                     # extends to end of file
            size = end - offset
        if size < header or offset + size > end:
            return
        yield kind, offset + header, offset + size, depth
        if kind in CONTAINERS:
            yield from walk(data, offset + header, offset + size, depth + 1)
        offset += size


def parse(path: Path) -> dict:
    data = path.read_bytes()
    report: dict = {
        "path": str(path), "size_bytes": len(data),
        "boxes": [], "tracks": [], "errors": [],
    }
    top = [k.decode("latin1") for k, _, _, d in walk(data, 0, len(data)) if d == 0]
    report["boxes"] = top
    report["has_ftyp"] = "ftyp" in top
    report["has_moov"] = "moov" in top
    report["has_mdat"] = "mdat" in top

    current: dict | None = None
    for kind, begin, finish, _ in walk(data, 0, len(data)):
        payload = data[begin:finish]
        if kind == b"mvhd" and len(payload) >= 20:
            version = payload[0]
            if version == 1 and len(payload) >= 32:
                timescale = struct.unpack(">I", payload[20:24])[0]
                duration = struct.unpack(">Q", payload[24:32])[0]
            else:
                timescale = struct.unpack(">I", payload[12:16])[0]
                duration = struct.unpack(">I", payload[16:20])[0]
            report["timescale"] = timescale
            report["duration_s"] = (duration / timescale) if timescale else None
        elif kind == b"tkhd" and len(payload) >= 84:
            # v0: 4 version/flags + 4 creation + 4 modification + 4 track_id
            #     + 4 reserved + 4 duration = 24; v1 widens the times to 8 -> 36.
            # Then 8 reserved + layer/altgroup/volume/reserved (8) + matrix (36)
            # puts width at base + 52 as a 16.16 fixed-point pair.
            version = payload[0]
            base = 36 if version == 1 else 24
            if len(payload) >= base + 60:
                width = struct.unpack(">I", payload[base + 52:base + 56])[0] >> 16
                height = struct.unpack(">I", payload[base + 56:base + 60])[0] >> 16
                current = {"width": width, "height": height}
                report["tracks"].append(current)
        elif kind == b"stsz" and current is not None and len(payload) >= 12:
            uniform = struct.unpack(">I", payload[4:8])[0]
            count = struct.unpack(">I", payload[8:12])[0]
            current["frame_count"] = count
            if uniform:
                sizes = [uniform] * count
            else:
                usable = min(count, max(0, (len(payload) - 12) // 4))
                sizes = list(struct.unpack(f">{usable}I", payload[12:12 + 4 * usable]))
            if sizes:
                current["frame_bytes_min"] = min(sizes)
                current["frame_bytes_max"] = max(sizes)
                current["frame_bytes_mean"] = sum(sizes) / len(sizes)
                # A still scene compresses to near-identical tiny samples.
                current["distinct_frame_sizes"] = len(set(sizes))
        elif kind == b"stts" and current is not None and len(payload) >= 8:
            entries = struct.unpack(">I", payload[4:8])[0]
            total = 0
            samples = 0
            for i in range(min(entries, (len(payload) - 8) // 8)):
                chunk = payload[8 + 8 * i: 16 + 8 * i]
                count, delta = struct.unpack(">II", chunk)
                samples += count
                total += count * delta
            current["stts_samples"] = samples
            current["stts_total_units"] = total
    return report


def verdict(report: dict, *, min_frames: int, min_bytes: int) -> tuple[bool, list[str]]:
    problems: list[str] = []
    if not report.get("has_ftyp"):
        problems.append("no ftyp box: not a recognisable MP4 container")
    if not report.get("has_moov"):
        problems.append("no moov box: the file has no index (encoder likely aborted)")
    if not report.get("has_mdat"):
        problems.append("no mdat box: the file carries no media payload")
    if report["size_bytes"] < min_bytes:
        problems.append(f"file is {report['size_bytes']} bytes, "
                        f"under the {min_bytes}-byte floor")
    video = [t for t in report["tracks"] if t.get("frame_count")]
    if not video:
        problems.append("no track with a sample table: zero frames were written")
    else:
        track = max(video, key=lambda t: t.get("frame_count", 0))
        if track["frame_count"] < min_frames:
            problems.append(f"only {track['frame_count']} frames, "
                            f"under the {min_frames}-frame floor")
        if track.get("distinct_frame_sizes", 0) <= 2 and track["frame_count"] > 4:
            problems.append("frame sizes are all but identical: the scene is "
                            "probably static (duplicated frames)")
    return (not problems), problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("movie", type=Path)
    parser.add_argument("--json", type=Path, default=None)
    parser.add_argument("--min-frames", type=int, default=30)
    parser.add_argument("--min-bytes", type=int, default=200_000)
    args = parser.parse_args()

    if not args.movie.is_file():
        print(f"MISSING: {args.movie}")
        return 1
    report = parse(args.movie)
    ok, problems = verdict(report, min_frames=args.min_frames,
                           min_bytes=args.min_bytes)
    report["verdict"] = "VALID" if ok else "INVALID"
    report["problems"] = problems

    print(f"file        : {report['path']}")
    print(f"size        : {report['size_bytes']:,} bytes")
    print(f"top boxes   : {report['boxes']}")
    if report.get("duration_s") is not None:
        print(f"duration    : {report['duration_s']:.2f} s "
              f"(timescale {report.get('timescale')})")
    for i, track in enumerate(report["tracks"]):
        print(f"track {i}     : {track.get('width')}x{track.get('height')}  "
              f"frames={track.get('frame_count')}  "
              f"distinct sizes={track.get('distinct_frame_sizes')}  "
              f"bytes/frame mean={track.get('frame_bytes_mean')}")
    print(f"verdict     : {report['verdict']}")
    for problem in problems:
        print(f"  ! {problem}")

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2), encoding="utf8")
        print(f"wrote {args.json}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
