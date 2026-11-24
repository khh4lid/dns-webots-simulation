"""Report the real pixel content of a PNG: size, colour spread, uniformity."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PIL import Image


def describe(path: Path) -> dict:
    with Image.open(path) as handle:
        image = handle.convert("RGB")
        array = np.asarray(image)
    flat = array.reshape(-1, array.shape[-1])
    unique = np.unique(flat, axis=0)
    return {
        "path": str(path),
        "bytes": path.stat().st_size,
        "size": f"{array.shape[1]}x{array.shape[0]}",
        "unique_colours": int(len(unique)),
        "mean_rgb": [round(float(v), 2) for v in flat.mean(axis=0)],
        "std_rgb": [round(float(v), 2) for v in flat.std(axis=0)],
        "min_rgb": [int(v) for v in flat.min(axis=0)],
        "max_rgb": [int(v) for v in flat.max(axis=0)],
    }


def main() -> int:
    paths = [Path(p) for p in sys.argv[1:]]
    if not paths:
        print("usage: inspect_png.py <file.png> [...]")
        return 1
    for path in paths:
        if not path.is_file():
            print(f"MISSING {path}")
            continue
        info = describe(path)
        print(f"{path.name:<14} {info['size']:>11}  {info['bytes']:>9,} B  "
              f"unique={info['unique_colours']:>8,}  "
              f"mean={info['mean_rgb']}  std={info['std_rgb']}  "
              f"range={info['min_rgb']}..{info['max_rgb']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
