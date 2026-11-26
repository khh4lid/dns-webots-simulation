"""Package the simulation-environment paper section: figures, DOCX and a PDF.

``docs/paper/simulation_environment_section.md`` is the source of truth. This
script does not rewrite it; it collects the figures the section cites into
``docs/paper/figures/``, renders the one figure that does not already exist as a
repository asset (the team-loop diagram), and emits a ``.docx`` for the team to
paste from plus a PDF preview for reading.

The DOCX is deliberately plain -- headings, paragraphs, real tables and placed
images -- because it is a paste source for someone else's template, not a
finished layout.

Run::

    python scripts/make_paper_section.py
"""
from __future__ import annotations

import re
import shutil
import sys
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import dns_style as S                                            # noqa: E402
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages             # noqa: E402

PAPER = ROOT / "docs" / "paper"
FIGDIR = PAPER / "figures"
SOURCE = PAPER / "simulation_environment_section.md"
ASSETS = ROOT / "assets" / "figures"

#: Section figure number -> (destination name, source asset or None if drawn).
FIGURES = {
    1: ("fig1_team_loop.png", None),
    2: ("fig2_obstacle_catalogue.png", ASSETS / "obstacle_catalogue.png"),
    3: ("fig3_side_elevation.png", ASSETS / "side_elevation.png"),
    4: ("fig4_shift_ladder.png", ASSETS / "F1_shift_ladder.png"),
    5: ("fig5_verification_panel.png", ASSETS / "F4_verification_panel.png"),
    6: ("fig6_generalisation.png", ASSETS / "F2_generalization.png"),
}


def draw_team_loop(path: Path) -> None:
    """Figure 1: the four-stage loop, with the environment stage highlighted.

    The slide version of this diagram lives in ``make_slides_simple`` and is
    drawn against a 16:9 slide. Its icon helpers compensate for the axis aspect
    through a module-level constant, so that constant is re-pointed at this
    figure's aspect before they are called.
    """
    import make_slides_simple as deck

    width, height = 9.0, 2.9
    figure = plt.figure(figsize=(width, height))
    figure.patch.set_facecolor("white")
    axis = figure.add_axes([0, 0, 1, 1])
    axis.axis("off")
    axis.set_xlim(0, 1)
    axis.set_ylim(0, 1)
    deck.ASPECT = width / height
    deck.CURRENT = "paper"
    deck.LABELS.setdefault("paper", [])

    boxes = [("World", "simulation environment", deck.MINE, deck.icon_world),
             ("LiDAR", "Section [Perception]", deck.NOT_CHOSEN, deck.icon_lidar),
             ("Policy", "Section [PPO]", deck.NOT_CHOSEN, deck.icon_policy),
             ("Vehicle moves", "", deck.NOT_CHOSEN, deck.icon_drone)]
    from matplotlib.patches import FancyBboxPatch

    x0, bw, gap, y, bh = 0.035, 0.205, 0.048, 0.34, 0.52
    for index, (title, owner, colour, icon) in enumerate(boxes):
        x = x0 + index * (bw + gap)
        axis.add_patch(FancyBboxPatch(
            (x, y), bw, bh, boxstyle="round,pad=0.006,rounding_size=0.02",
            facecolor=deck.MINE if colour is deck.MINE else "#f4f5f7",
            alpha=0.16 if colour is deck.MINE else 1.0,
            edgecolor=colour, linewidth=2.2 if colour is deck.MINE else 1.0))
        icon(axis, x + bw / 2, y + bh * 0.63, 0.052)
        axis.text(x + bw / 2, y + 0.115, title, fontsize=12.5,
                  fontweight="bold",
                  color=colour if colour is deck.MINE else S.INK,
                  ha="center", va="center")
        if owner:
            axis.text(x + bw / 2, y + 0.055, owner, fontsize=8.5,
                      color=S.INK_MUTED, ha="center", va="center")
        if index < len(boxes) - 1:
            axis.annotate("", (x + bw + gap - 0.008, y + bh / 2),
                          (x + bw + 0.008, y + bh / 2),
                          arrowprops=dict(arrowstyle="-|>", color=S.INK_SOFT,
                                          lw=1.8))
    right = x0 + 3 * (bw + gap) + bw
    drop = y - 0.14
    axis.plot([right - 0.06, right - 0.06, x0 + 0.06, x0 + 0.06],
              [y, drop, drop, y - 0.01], color=S.INK_SOFT, linewidth=1.8,
              solid_joinstyle="round")
    axis.annotate("", (x0 + 0.06, y - 0.004), (x0 + 0.06, drop + 0.05),
                  arrowprops=dict(arrowstyle="-|>", color=S.INK_SOFT, lw=1.8))
    axis.text((x0 + right) / 2, drop - 0.085,
              "the world advances one 0.2 s control step", fontsize=9,
              color=S.INK_MUTED, ha="center", va="center")
    figure.savefig(path, dpi=300, facecolor="white", bbox_inches=None,
                   pad_inches=0.02)
    plt.close(figure)


# ---------------------------------------------------------------------------
# a very small markdown reader -- only the constructs this section uses
# ---------------------------------------------------------------------------
def parse(text: str) -> list[tuple[str, object]]:
    """Return a list of (kind, payload) blocks: heading/para/table/caption."""
    # Drop the note-to-the-team block: it is not part of the paper.
    text = text.split("---", 1)[1] if text.startswith("#") else text
    blocks: list[tuple[str, object]] = []
    rows: list[list[str]] = []

    def flush() -> None:
        if rows:
            blocks.append(("table", [r[:] for r in rows]))
            rows.clear()

    for raw in text.splitlines():
        line = raw.rstrip()
        if line.startswith("|"):
            cells = [c.strip() for c in line.strip("|").split("|")]
            if all(set(c) <= set("-: ") for c in cells):
                continue                      # the --- separator row
            rows.append(cells)
            continue
        flush()
        if not line or line == "---":
            blocks.append(("blank", ""))
        elif line.startswith("#"):
            level = len(line) - len(line.lstrip("#"))
            blocks.append((f"h{level}", line.lstrip("# ").strip()))
        elif line.startswith("**Figure") or line.startswith("**Table"):
            blocks.append(("caption", line))
        else:
            blocks.append(("para", line))
    flush()

    # Join wrapped paragraph and caption lines back into single blocks.
    merged: list[tuple[str, object]] = []
    for kind, payload in blocks:
        if (kind in {"para", "caption"} and merged
                and merged[-1][0] == kind):
            merged[-1] = (kind, f"{merged[-1][1]} {payload}")
        elif kind != "blank":
            merged.append((kind, payload))
        else:
            merged.append((kind, payload))
    return [b for b in merged if b[0] != "blank" or True]


def strip_markup(text: str) -> list[tuple[str, bool, bool]]:
    """Split inline markup into (text, bold, italic) runs."""
    runs: list[tuple[str, bool, bool]] = []
    for piece in re.split(r"(\*\*[^*]+\*\*|\*[^*]+\*|`[^`]+`)", text):
        if not piece:
            continue
        if piece.startswith("**"):
            runs.append((piece[2:-2], True, False))
        elif piece.startswith("*"):
            runs.append((piece[1:-1], False, True))
        elif piece.startswith("`"):
            runs.append((piece[1:-1], False, False))
        else:
            runs.append((piece, False, False))
    return runs


def build_docx(blocks, path: Path) -> None:
    from docx import Document
    from docx.enum.text import WD_ALIGN_PARAGRAPH
    from docx.shared import Inches, Pt

    document = Document()
    style = document.styles["Normal"]
    style.font.name = "Times New Roman"
    style.font.size = Pt(10)

    for kind, payload in blocks:
        if kind == "blank":
            continue
        if kind.startswith("h"):
            document.add_heading(payload, level=min(int(kind[1:]), 4))
        elif kind == "table":
            rows = payload
            table = document.add_table(rows=len(rows), cols=len(rows[0]))
            table.style = "Table Grid"
            for r, row in enumerate(rows):
                for c, cell in enumerate(row):
                    paragraph = table.cell(r, c).paragraphs[0]
                    for run_text, bold, italic in strip_markup(cell):
                        run = paragraph.add_run(run_text)
                        run.bold = bold or r == 0
                        run.italic = italic
                        run.font.size = Pt(8.5)
        elif kind == "caption":
            number = re.match(r"\*\*(Figure|Table) (\d)", payload)
            if number and number.group(1) == "Figure":
                name, _src = FIGURES[int(number.group(2))]
                image = FIGDIR / name
                if image.is_file():
                    paragraph = document.add_paragraph()
                    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
                    paragraph.add_run().add_picture(str(image),
                                                    width=Inches(6.0))
            paragraph = document.add_paragraph()
            paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
            for run_text, bold, italic in strip_markup(payload):
                run = paragraph.add_run(run_text)
                run.bold = bold
                run.italic = italic
                run.font.size = Pt(8.5)
        else:
            paragraph = document.add_paragraph()
            paragraph.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
            for run_text, bold, italic in strip_markup(payload):
                run = paragraph.add_run(run_text)
                run.bold = bold
                run.italic = italic
    document.save(str(path))


def build_pdf(blocks, path: Path) -> None:
    """A reading preview, not a typeset paper: flowing text on A4 pages."""
    state = {"figure": None, "axis": None, "y": 0.0}

    def new_page(pdf):
        if state["figure"] is not None:
            pdf.savefig(state["figure"], facecolor="white")
            plt.close(state["figure"])
        figure = plt.figure(figsize=(8.27, 11.69))
        figure.patch.set_facecolor("white")
        axis = figure.add_axes([0, 0, 1, 1])
        axis.axis("off")
        axis.set_xlim(0, 1)
        axis.set_ylim(0, 1)
        state.update(figure=figure, axis=axis, y=0.94)

    def need(pdf, space):
        if state["y"] - space < 0.06:
            new_page(pdf)

    with PdfPages(path) as pdf:
        new_page(pdf)
        for kind, payload in blocks:
            axis = state["axis"]
            if kind == "blank":
                state["y"] -= 0.008
            elif kind.startswith("h"):
                size = {1: 15, 2: 13, 3: 11.5}.get(int(kind[1:]), 10.5)
                need(pdf, 0.05)
                axis = state["axis"]
                state["y"] -= 0.018
                axis.text(0.09, state["y"], payload, fontsize=size,
                          fontweight="bold", color=S.INK, va="top")
                state["y"] -= 0.026
            elif kind == "table":
                rows = payload
                need(pdf, 0.016 * (len(rows) + 1))
                axis = state["axis"]
                widths = [max(len(r[c]) for r in rows)
                          for c in range(len(rows[0]))]
                total = sum(widths) or 1
                xs, acc = [], 0.09
                for w in widths:
                    xs.append(acc)
                    acc += 0.82 * w / total
                for r, row in enumerate(rows):
                    for cell, x in zip(row, xs):
                        axis.text(x, state["y"],
                                  textwrap.shorten(re.sub(r"[*`]", "", cell),
                                                   max(12, int(118 * 0.82
                                                               * len(cell)
                                                               / max(total, 1))
                                                       + 6),
                                                   placeholder="…"),
                                  fontsize=6.6,
                                  fontweight="bold" if r == 0 else "normal",
                                  color=S.INK if r == 0 else S.INK_SOFT,
                                  va="top")
                    state["y"] -= 0.0155
                state["y"] -= 0.008
            elif kind == "caption":
                number = re.match(r"\*\*(Figure) (\d)", payload)
                if number:
                    name, _src = FIGURES[int(number.group(2))]
                    image = FIGDIR / name
                    if image.is_file():
                        picture = plt.imread(str(image))
                        h, w = picture.shape[:2]
                        box_w = 0.72
                        box_h = box_w * (h / w) * (8.27 / 11.69)
                        need(pdf, box_h + 0.04)
                        axis = state["axis"]
                        inset = state["figure"].add_axes(
                            [0.14, state["y"] - box_h, box_w, box_h])
                        inset.imshow(picture)
                        inset.axis("off")
                        state["y"] -= box_h + 0.012
                need(pdf, 0.03)
                axis = state["axis"]
                for line in textwrap.wrap(re.sub(r"[*`]", "", payload), 112):
                    axis.text(0.09, state["y"], line, fontsize=7.2,
                              color=S.INK_MUTED, va="top")
                    state["y"] -= 0.0135
                state["y"] -= 0.008
            else:
                text = re.sub(r"[*`]", "", payload)
                lines = textwrap.wrap(" ".join(text.split()), 104)
                need(pdf, 0.016 * len(lines))
                axis = state["axis"]
                for line in lines:
                    if state["y"] < 0.06:
                        new_page(pdf)
                        axis = state["axis"]
                    axis.text(0.09, state["y"], line, fontsize=8.4,
                              color=S.INK, va="top")
                    state["y"] -= 0.0155
                state["y"] -= 0.009
        pdf.savefig(state["figure"], facecolor="white")
        plt.close(state["figure"])


def main() -> int:
    S.apply()
    FIGDIR.mkdir(parents=True, exist_ok=True)

    missing = []
    for number, (name, source) in FIGURES.items():
        target = FIGDIR / name
        if source is None:
            draw_team_loop(target)
        elif source.is_file():
            shutil.copyfile(source, target)
        else:
            missing.append(str(source))
        print(f"  figure {number}: {target.relative_to(ROOT)}"
              f"{'  [SOURCE MISSING]' if not target.is_file() else ''}")
    if missing:
        print(f"  !! missing sources: {missing}")

    blocks = parse(SOURCE.read_text(encoding="utf8"))
    docx_path = PAPER / "simulation_environment_section.docx"
    pdf_path = PAPER / "simulation_environment_section.pdf"
    build_docx(blocks, docx_path)
    print(f"  wrote {docx_path.relative_to(ROOT)} "
          f"({docx_path.stat().st_size/1e6:.2f} MB)")
    build_pdf(blocks, pdf_path)
    print(f"  wrote {pdf_path.relative_to(ROOT)} "
          f"({pdf_path.stat().st_size/1e6:.2f} MB)")
    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())
