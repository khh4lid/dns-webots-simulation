"""Assemble the Google Drive handoff folder for the research team.

Produces a small, viewer-friendly bundle beside the repository -- PDFs for
reading, the demonstration video, the evaluation table, and the teammate
handoff material. It is deliberately *not* a copy of the repository: GitHub
carries versioned source, configuration and tests; Drive carries the things
people want to open and look at.

    DNS_Research_Team_Shared/
    |-- README_FIRST.pdf
    |-- DNS_Research_Demo.mp4
    |-- DNS_Simulation_Methods.pdf
    |-- DNS_Key_Figures.pdf
    |-- DNS_Evaluation_Summary.csv
    |-- DNS_Validation_Evidence.pdf
    |-- DNS_Team_Handoffs/
    |   |-- PPO_Interface.pdf
    |   |-- LiDAR_Interface.pdf
    |   |-- two_frames.json
    |-- UPLOAD_INSTRUCTIONS.md
    |-- MANIFEST.json

Run::

    python scripts/build_drive_package.py

Nothing is uploaded. The folder is written locally for review first.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import sys
import textwrap
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

DOCS = PROJECT_ROOT / "docs"
ASSETS = PROJECT_ROOT / "assets"
RESULTS = PROJECT_ROOT / "results"

INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
RULE = "#c3c2b7"
SURFACE = "#ffffff"

MANIFEST: list[dict] = []


# --------------------------------------------------------------------------
# a small markdown -> PDF renderer
# --------------------------------------------------------------------------
class PdfWriter:
    """Lay out headings, paragraphs, bullets, tables and code onto A4 pages."""

    TOP = 0.945
    BOTTOM = 0.075
    LEFT = 0.085
    RIGHT = 0.945

    def __init__(self, pdf, title: str, subtitle: str = ""):
        self.pdf = pdf
        self.title = title
        self.subtitle = subtitle
        self.fig = None
        self.y = 0.0
        self.page = 0
        self._new_page(first=True)

    def _new_page(self, first: bool = False) -> None:
        import matplotlib.pyplot as plt
        if self.fig is not None:
            self._footer()
            self.pdf.savefig(self.fig)
            plt.close(self.fig)
        self.page += 1
        self.fig = plt.figure(figsize=(8.27, 11.69), dpi=200, facecolor=SURFACE)
        self.y = self.TOP
        if first:
            self._text(self.title, 17.0, INK, weight="bold")
            if self.subtitle:
                self.y -= 0.004
                self._text(self.subtitle, 9.5, INK_2)
            self.y -= 0.012
            self.fig.add_artist(plt.Line2D(
                [self.LEFT, self.RIGHT], [self.y, self.y],
                color=RULE, linewidth=1.0))
            self.y -= 0.022

    def _footer(self) -> None:
        self.fig.text(self.RIGHT, 0.042, f"{self.page}", fontsize=7.5,
                      color=MUTED, ha="right")
        self.fig.text(self.LEFT, 0.042,
                      "DNS — Simulation Environment Development · "
                      "Taylor's University · private research material",
                      fontsize=7.0, color=MUTED, ha="left")

    def _room(self, need: float) -> None:
        if self.y - need < self.BOTTOM:
            self._new_page()

    def _text(self, text: str, size: float, colour: str, *,
              weight: str = "normal", indent: float = 0.0,
              family: str = "sans-serif") -> None:
        self.fig.text(self.LEFT + indent, self.y, text, fontsize=size,
                      color=colour, va="top", ha="left", fontweight=weight,
                      family=family, linespacing=1.5)
        self.y -= 0.0132 * (size / 9.0) * (text.count("\n") + 1)

    # -- block builders ----------------------------------------------------
    def heading(self, text: str, level: int) -> None:
        size = {1: 14.0, 2: 12.0, 3: 10.2}.get(level, 9.6)
        self._room(0.055)
        self.y -= 0.016 if level <= 2 else 0.010
        self._text(text, size, INK, weight="bold")
        self.y -= 0.006

    def body(self, text: str, size: float = 8.6) -> None:
        wrapped = "\n".join(textwrap.wrap(text, width=104)) or " "
        self._room(0.016 * (wrapped.count("\n") + 1) + 0.01)
        self._text(wrapped, size, INK_2)
        self.y -= 0.006

    def bullets(self, items: list[str]) -> None:
        for item in items:
            lines = textwrap.wrap(item, width=98)
            if not lines:
                continue
            self._room(0.016 * len(lines) + 0.006)
            self._text("•  " + lines[0], 8.6, INK_2, indent=0.012)
            for extra in lines[1:]:
                self._text("    " + extra, 8.6, INK_2, indent=0.012)
        self.y -= 0.008

    def code(self, lines: list[str]) -> None:
        block = [ln[:98] for ln in lines]
        self._room(0.0125 * len(block) + 0.016)
        top = self.y + 0.008
        height = 0.0125 * len(block) + 0.012
        import matplotlib.patches as mpatches
        self.fig.add_artist(mpatches.Rectangle(
            (self.LEFT - 0.012, top - height), self.RIGHT - self.LEFT + 0.024,
            height, facecolor="#f4f4f1", edgecolor="#e1e0d9", linewidth=0.7))
        for ln in block:
            self._text(ln or " ", 7.6, INK, family="monospace", indent=0.0)
        self.y -= 0.012

    def table(self, rows: list[list[str]]) -> None:
        if not rows:
            return
        ncol = max(len(r) for r in rows)
        rows = [r + [""] * (ncol - len(r)) for r in rows]
        width = (self.RIGHT - self.LEFT) / ncol
        chars = max(10, int(104 / ncol))
        header, *body = rows
        self._room(0.05)
        import matplotlib.pyplot as plt
        y0 = self.y
        for j, cell in enumerate(header):
            self.fig.text(self.LEFT + j * width, y0,
                          "\n".join(textwrap.wrap(cell, chars))[:120],
                          fontsize=7.4, color=INK, va="top", ha="left",
                          fontweight="bold")
        self.y -= 0.0135 * (1 + max(len(textwrap.wrap(c, chars)) for c in header))
        self.fig.add_artist(plt.Line2D([self.LEFT, self.RIGHT],
                                       [self.y + 0.004, self.y + 0.004],
                                       color=RULE, linewidth=0.8))
        self.y -= 0.004
        for row in body:
            cells = ["\n".join(textwrap.wrap(c, chars)) for c in row]
            depth = max(c.count("\n") + 1 for c in cells)
            self._room(0.0125 * depth + 0.008)
            y0 = self.y
            for j, cell in enumerate(cells):
                self.fig.text(self.LEFT + j * width, y0, cell, fontsize=7.2,
                              color=INK_2, va="top", ha="left")
            self.y -= 0.0125 * depth + 0.003
        self.y -= 0.010

    def image(self, path: Path, caption: str = "") -> None:
        from matplotlib.image import imread
        try:
            img = imread(str(path))
        except Exception:                                         # noqa: BLE001
            return
        h, w = img.shape[:2]
        avail_w = self.RIGHT - self.LEFT
        draw_h = avail_w * (h / w) * (8.27 / 11.69)
        if self.y - draw_h - 0.03 < self.BOTTOM:
            self._new_page()
        ax = self.fig.add_axes([self.LEFT, self.y - draw_h, avail_w, draw_h])
        ax.imshow(img)
        ax.axis("off")
        self.y -= draw_h + 0.008
        if caption:
            self.body(caption, size=7.6)
        self.y -= 0.006

    def close(self) -> None:
        import matplotlib.pyplot as plt
        if self.fig is not None:
            self._footer()
            self.pdf.savefig(self.fig)
            plt.close(self.fig)
            self.fig = None


_LINK = re.compile(r"\[([^\]]+)\]\([^)]+\)")
_BOLD = re.compile(r"\*\*([^*]+)\*\*")
_CODE = re.compile(r"`([^`]+)`")
_IMG = re.compile(r"^!\[")


#: LaTeX fragments used in the source documents, mapped to plain text. The
#: display-math blocks are rendered as monospace rather than handed to
#: matplotlib's mathtext, which supports only a subset of LaTeX and raises on
#: the rest -- a PDF build must not fail because of an equation.
_LATEX = [
    (r"\left(", "("), (r"\right)", ")"),
    (r"\bigl(", "("), (r"\bigr)", ")"),
    (r"\lvert", "|"), (r"\rvert", "|"),
    (r"\bmod", " mod "), (r"\qquad", "    "), (r"\quad", "  "),
    (r"\varphi", "phi"), (r"\omega", "w"), (r"\Delta", "D"), (r"\pi", "pi"),
    (r"\,", " "), (r"\;", " "),
]
_FRAC = re.compile(r"\\frac\{([^{}]*)\}\{([^{}]*)\}")
_SUB = re.compile(r"_\{?(\w)\}?")


def latex_to_text(block: str) -> str:
    text = block.strip().strip("$").strip()
    for _ in range(3):                      # nested fractions are shallow here
        text = _FRAC.sub(r"(\1)/(\2)", text)
    for src, dst in _LATEX:
        text = text.replace(src, dst)
    text = _SUB.sub(r"\1", text)
    text = text.replace("\\", "")
    return re.sub(r"[ \t]+", " ", text).strip()


#: Characters the PDF font (Segoe UI) has no glyph for, which would otherwise
#: be drawn as empty boxes.
_GLYPHS = {"⚠": "!", "∈": " in ", "✓": "yes", "✗": "no"}


def clean(text: str) -> str:
    text = _LINK.sub(r"\1", text)
    text = _BOLD.sub(r"\1", text)
    text = _CODE.sub(r"\1", text)
    for src, dst in _GLYPHS.items():
        text = text.replace(src, dst)
    # A bare "$" makes matplotlib try to parse mathtext; escape it.
    return text.replace("&amp;", "&").replace("$", r"\$").strip()


def render_markdown(writer: PdfWriter, md: str, *, skip_to: str | None = None,
                    stop_at: str | None = None) -> None:
    lines = md.splitlines()
    started = skip_to is None
    para: list[str] = []
    bullets: list[str] = []
    table: list[list[str]] = []
    code: list[str] | None = None
    math: list[str] | None = None

    def flush() -> None:
        nonlocal para, bullets, table
        if para:
            writer.body(clean(" ".join(para)))
            para = []
        if bullets:
            writer.bullets([clean(b) for b in bullets])
            bullets = []
        if table:
            writer.table(table)
            table = []

    for raw in lines:
        line = raw.rstrip()
        if not started:
            if skip_to and line.startswith(skip_to):
                started = True
            else:
                continue
        if stop_at and line.startswith(stop_at) and started and not line.startswith(skip_to or "\0"):
            break

        if line.startswith("```"):
            if code is None:
                flush()
                code = []
            else:
                writer.code(code)
                code = None
            continue
        if code is not None:
            code.append(line)
            continue

        # Display math: accumulate until the closing "$$", then render the
        # whole block as plain monospace.
        if math is not None:
            math.append(line)
            if line.rstrip().endswith("$$"):
                writer.code([latex_to_text(" ".join(math))])
                math = None
            continue
        if line.lstrip().startswith("$$"):
            flush()
            stripped = line.strip()
            if stripped.endswith("$$") and len(stripped) > 4:
                writer.code([latex_to_text(stripped)])
            else:
                math = [stripped]
            continue

        if _IMG.match(line.strip()):
            continue
        if line.startswith("> "):
            para.append(line[2:])
            continue
        if re.match(r"^\s*$", line):
            flush()
            continue
        if line.startswith("---"):
            flush()
            continue
        if line.startswith("#"):
            flush()
            level = len(line) - len(line.lstrip("#"))
            writer.heading(clean(line.lstrip("# ")), level)
            continue
        if line.lstrip().startswith(("- ", "* ")):
            if para:
                writer.body(clean(" ".join(para)))
                para = []
            bullets.append(line.lstrip()[2:])
            continue
        if re.match(r"^\s*\d+\.\s", line):
            if para:
                writer.body(clean(" ".join(para)))
                para = []
            bullets.append(re.sub(r"^\s*\d+\.\s*", "", line))
            continue
        if line.startswith("|"):
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            if all(set(c) <= set("-: ") for c in cells):
                continue
            if para or bullets:
                flush()
            table.append([clean(c) for c in cells])
            continue
        if table:
            writer.table(table)
            table = []
        para.append(line.strip())

    if code:
        writer.code(code)
    flush()


# --------------------------------------------------------------------------
def md_to_pdf(src: Path, dest: Path, title: str, subtitle: str = "", *,
              skip_to: str | None = None, stop_at: str | None = None) -> bool:
    from matplotlib.backends.backend_pdf import PdfPages
    if not src.is_file():
        print(f"  MISSING {src.relative_to(PROJECT_ROOT)}")
        return False
    dest.parent.mkdir(parents=True, exist_ok=True)
    with PdfPages(dest) as pdf:
        writer = PdfWriter(pdf, title, subtitle)
        render_markdown(writer, src.read_text(encoding="utf8"),
                        skip_to=skip_to, stop_at=stop_at)
        writer.close()
    return True


def figures_pdf(dest: Path) -> bool:
    from matplotlib.backends.backend_pdf import PdfPages
    figures = [
        (ASSETS / "screenshots" / "hero_annotated.png",
         "The Webots warehouse scene, rendered by the simulator and labelled by "
         "projecting the configured world coordinates through the recorded "
         "camera pose."),
        (ASSETS / "figures" / "environment_map.png",
         "Scaled plan drawn from the same scenario file that generates the "
         "Webots world."),
        (ASSETS / "figures" / "success_rate.png",
         "Goal-reaching rate with 95% Wilson intervals over 90 episodes. The "
         "intervals overlap heavily; this does not rank the seeds, and the "
         "untuned scripted baselines are a floor rather than a comparison."),
        (ASSETS / "figures" / "trajectories.png",
         "First recorded episode of each controller, failures included."),
        (ASSETS / "figures" / "moving_obstacle_validation.png",
         "Moving-obstacle path (exact by construction) and the independent "
         "comparison of the measured scan against an analytic ray-cast."),
        (ASSETS / "figures" / "lidar_validation.png",
         "LiDAR sector ranges against an independent ray-cast of the same "
         "scene; the angular mapping was measured, not assumed."),
    ]
    present = [(p, c) for p, c in figures if p.is_file()]
    if not present:
        print("  MISSING all figures -- run scripts/make_assets.py first")
        return False
    dest.parent.mkdir(parents=True, exist_ok=True)
    with PdfPages(dest) as pdf:
        writer = PdfWriter(
            pdf, "DNS — Key Figures",
            "Simulation Environment Development · every figure generated from "
            "project data by scripts/make_assets.py")
        for path, caption in present:
            writer.heading(path.stem.replace("_", " ").title(), 2)
            writer.image(path, caption)
        writer.close()
    if len(present) < len(figures):
        print(f"  note: {len(figures) - len(present)} figure(s) missing")
    return True


def copy(src: Path, dest: Path, note: str) -> bool:
    if not src.is_file():
        print(f"  MISSING {src}")
        return False
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)
    record(dest, src, note)
    return True


def record(dest: Path, src: Path | None, note: str) -> None:
    digest = hashlib.sha256(dest.read_bytes()).hexdigest()[:16]
    MANIFEST.append({
        "file": dest.name,
        "path": str(dest.relative_to(PACKAGE)).replace("\\", "/"),
        "source": (str(src.relative_to(PROJECT_ROOT)).replace("\\", "/")
                   if src else "generated"),
        "sha256_16": digest,
        "bytes": dest.stat().st_size,
        "note": note,
    })
    print(f"  + {dest.relative_to(PACKAGE)}")


PACKAGE = PROJECT_ROOT.parent / "DNS_Research_Team_Shared"


def main(argv: list[str] | None = None) -> int:
    global PACKAGE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=PACKAGE)
    args = parser.parse_args(argv)
    PACKAGE = args.out.resolve()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams.update({"font.family": "sans-serif",
                             "font.sans-serif": ["Segoe UI", "DejaVu Sans"],
                             "font.monospace": ["Consolas", "DejaVu Sans Mono"]})
    except ImportError:
        print("matplotlib is required: pip install -r requirements.txt")
        return 1

    PACKAGE.mkdir(parents=True, exist_ok=True)
    handoffs = PACKAGE / "DNS_Team_Handoffs"
    handoffs.mkdir(parents=True, exist_ok=True)
    print(f"building {PACKAGE}\n")

    if md_to_pdf(PROJECT_ROOT / "README.md", PACKAGE / "README_FIRST.pdf",
                 "DNS — Webots Drone Navigation Simulation",
                 "Read this first · Simulation Environment Development · "
                 "Taylor's University"):
        record(PACKAGE / "README_FIRST.pdf", PROJECT_ROOT / "README.md",
               "Repository front page rendered for reading off Drive.")

    if md_to_pdf(DOCS / "SIMULATION_METHODS.md",
                 PACKAGE / "DNS_Simulation_Methods.pdf",
                 "Design and Verification of a Webots Simulation Environment",
                 "UAV navigation in static and dynamic scenarios"):
        record(PACKAGE / "DNS_Simulation_Methods.pdf",
               DOCS / "SIMULATION_METHODS.md",
               "Full methods: geometry, obstacle models, drone model, control, "
               "timing, collision, reproducibility.")

    if md_to_pdf(DOCS / "VALIDATION_EVIDENCE.md",
                 PACKAGE / "DNS_Validation_Evidence.pdf",
                 "DNS — Validation and Evidence",
                 "Verification results, experiment provenance, limitations, "
                 "reproduction recipe"):
        record(PACKAGE / "DNS_Validation_Evidence.pdf",
               DOCS / "VALIDATION_EVIDENCE.md",
               "Every verification result with its evidence file.")

    if figures_pdf(PACKAGE / "DNS_Key_Figures.pdf"):
        record(PACKAGE / "DNS_Key_Figures.pdf", None,
               "All README figures with captions, generated from project data.")

    md_to_pdf(DOCS / "TEAM_INTEGRATION.md", handoffs / "PPO_Interface.pdf",
              "PPO Integration — DNS Webots environment",
              "For Spencer (deep reinforcement learning)",
              skip_to="# A. PPO integration", stop_at="# B. LiDAR integration")
    if (handoffs / "PPO_Interface.pdf").is_file():
        record(handoffs / "PPO_Interface.pdf", DOCS / "TEAM_INTEGRATION.md",
               "Section A of the team integration document.")

    md_to_pdf(DOCS / "TEAM_INTEGRATION.md", handoffs / "LiDAR_Interface.pdf",
              "LiDAR Integration — DNS Webots sensor",
              "For the sensors and perception workstream",
              skip_to="# B. LiDAR integration", stop_at="# C. Flagged")
    if (handoffs / "LiDAR_Interface.pdf").is_file():
        record(handoffs / "LiDAR_Interface.pdf", DOCS / "TEAM_INTEGRATION.md",
               "Section B of the team integration document.")

    copy(RESULTS / "demo" / "DNS_Webots_Research_Demo.mp4",
         PACKAGE / "DNS_Research_Demo.mp4",
         "50.2 s Webots capture. Static segments reach the goal; both dynamic "
         "segments end in a collision with the transfer cart.")
    copy(RESULTS / "experiments_warehouse" / "experiment_summary.csv",
         PACKAGE / "DNS_Evaluation_Summary.csv",
         "Per-cell outcomes and Wilson intervals over 90 episodes.")
    copy(RESULTS / "two_frames_webots_physical_lidar.json",
         handoffs / "two_frames.json",
         "Two consecutive frames, exactly 0.200000 s apart, from a real run.")

    (PACKAGE / "UPLOAD_INSTRUCTIONS.md").write_text(INSTRUCTIONS, encoding="utf8")
    record(PACKAGE / "UPLOAD_INSTRUCTIONS.md", None,
           "How to upload and share this folder.")

    (PACKAGE / "MANIFEST.json").write_text(json.dumps({
        "package": PACKAGE.name,
        "project": "Drone Navigation System — Simulation Environment Development",
        "institution": "Taylor's University",
        "supervisor": "Dr. Mehdi Samieiyeganeh",
        "visibility": "private research material — share only with the "
                      "supervisor and authorised team members",
        "built_from": "webots_dns repository",
        "statement": "Every file here is derived from this project's own "
                     "verified outputs. No third-party material is included.",
        "files": MANIFEST,
    }, indent=2), encoding="utf8")
    print(f"\n{len(MANIFEST)} files written to {PACKAGE}")
    print("Nothing has been uploaded. Review the folder, then follow "
          "UPLOAD_INSTRUCTIONS.md.")
    return 0


INSTRUCTIONS = """# Uploading this folder to Google Drive

Nothing in this folder has been uploaded. It was assembled locally so it can be
reviewed first.

## 1. Check the contents

Open `MANIFEST.json`. Every entry lists the file, the project source it came
from, its size and a short SHA-256 prefix. Confirm that each item is cleared for
sharing with the people who will get the link.

Two things deserve a deliberate decision before sharing:

- **`DNS_Research_Demo.mp4`** shows two dynamic episodes that end in a collision
  with the transfer cart. That is accurate and intentional, not a bad take.
- **The PDFs state the project's limitations plainly**, including that only one
  obstacle layout was evaluated. Keep those sections in; they are what makes the
  rest credible.

## 2. Create the folder

1. Go to <https://drive.google.com>.
2. **New → Folder**, name it exactly `DNS_Research_Team_Shared`.
3. Open it, then **New → Folder upload** and select this folder's contents, or
   drag the files in. Keep `DNS_Team_Handoffs/` as a subfolder.

## 3. Share it deliberately

Use **Share → restricted**, then add people by email address individually:

- the supervisor, Dr. Mehdi Samieiyeganeh
- Spencer (deep reinforcement learning)
- the sensors and perception teammate

Set them to **Viewer** unless someone needs to add material. Do **not** use
"Anyone with the link" — this is unpublished university research.

## 4. Link the video back into the repository

Once the video is uploaded, open it in Drive, choose **Share → Copy link**, and
paste that URL into two places in the repository:

- `README.md`, section 12 — replace
  `**Demonstration video — hosted link pending.**` with a Markdown link, and
  optionally make the thumbnail clickable:
  `[![...](assets/demo-thumbnail.jpg)](PASTE_DRIVE_URL)`
- `README.md`, section 16 — replace `hosted link pending` in the documents
  table.

A Drive folder link can go in section 16 the same way. Do not add a URL before
it exists; a broken placeholder is worse than an honest "pending".

## 5. Keep the split

GitHub carries versioned source, configuration, tests and raw results. Drive
carries what people want to open: the video, the PDFs, the figures and the
evaluation table. Avoid copying the repository into Drive — the two will drift,
and the Git history is what makes the repository trustworthy.

To refresh this folder after new results:

```bash
python scripts/make_assets.py
python scripts/build_drive_package.py
```
"""


if __name__ == "__main__":
    raise SystemExit(main())
