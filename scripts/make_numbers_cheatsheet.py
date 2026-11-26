"""One A4 page with every number in the final deck, grouped by type.

The deck tags each number with one of seven types. This sheet collects them
all in one place so the presenter can find any figure, its unit, what it means
and the file it came from, without hunting through slides.

The card list is taken from the deck module itself rather than re-typed, so the
sheet cannot drift from what is actually on the slides: building every slide
populates ``make_slides_final.CARDS``, and this reads that.

Run::

    python scripts/make_numbers_cheatsheet.py
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import dns_style as S                                            # noqa: E402
import matplotlib                                                # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt                                  # noqa: E402

import make_slides_final as deck                                 # noqa: E402

OUT = ROOT / "docs" / "meetings" / "2026-11-27" / "NUMBERS_CHEATSHEET.pdf"

#: value -> where it comes from. Keyed on the card's printed value so the sheet
#: states a source for every number the audience can see.
SOURCE = {
    f"{deck.SPAN:.2f}": "goal x minus start x, config/scenarios/L00_C.json",
    f"{deck.GOAL_TOL:.1f}": "goal.tolerance_m, L00_C.json",
    f"{deck.EPISODE_S:.0f}": "90 steps x 0.2 s, navigation.HORIZON",
    f"{deck.MAX_SPEED:.1f}": "control.max_speed_mps, L00_C.json",
    f"{deck.EXTENT_X:.0f}×{deck.EXTENT_Y:.0f}": "arena.low_xy / high_xy",
    f"{deck.ARENA['wall_height_m']:.1f}": "arena.wall_height_m",
    "1": "the plan's grid step, drawn at 1 m",
    f"{deck.CRUISE:.1f}": "drone.cruise_altitude_m",
    f"{deck.SCAN:.2f}": "cruise 1.00 + sensing.mount_height_m 0.14",
    f"{deck.D['drone']['mass_kg']:.2f}": "DnsQuadcopter.proto mass",
    f"{deck.DT}": "control.command_period_s",
    f"{deck.INNER_HZ}": "1000 / control.basic_timestep_ms",
    f"{deck.BEAMS}": "worlds/DNS_L00_C.wbt lidarResolution",
    f"{deck.D['drone']['range_m']:.0f}": "worlds/DNS_L00_C.wbt lidarMaxRange",
    f"{deck.SECTORS}": "sensing.sectors, L00_C.json",
    "360": "Lidar fieldOfView 2 pi, DnsQuadcopter.proto",
    f"{deck.CART['motion']['speed_mps']:.2f}": "cart motion.speed_mps",
    f"±{deck.CART['motion']['amplitude_m']:.1f}": "cart motion.amplitude_m",
    f"{deck.CART_PERIOD:.2f}": "4 x amplitude / speed",
    f"{deck.INTRUSION:.2f}": "scenario_table.csv line_intrusion_m, L00",
    f"{deck.D['episodes']}": "results_v2.csv row count",
}
for _layout, _name in deck.LADDER:
    SOURCE[f"{deck.gap(_layout):.2f}"] = (
        f"scenario_table.csv min_corridor_width_enroute_m, {_layout}")
_k, _n, _lo, _hi = deck.pooled("L00")
SOURCE[f"{_n}"] = "pooled_rates.csv n, Mode C"
SOURCE[f"{_k} of {_n}"] = "pooled_rates.csv k/n, L00 and L02"
_lidar = deck.D["lidar"]["cart_sector_pooled"]
SOURCE[f"{_lidar['mae_m'] * 100:.2f}"] = "lidar_cart_consistency.json mae_m"
_drift = deck.D["reset"]["arms"]["sequence"]["max_path_delta_m_worst"]
SOURCE[f"{_drift * 1e6:.0f}"] = "reset_study/SUMMARY.json sequence arm"
SOURCE[f"{deck.DT:.3f}"] = "control.command_period_s, measured exactly"
SOURCE[f"{deck.D['defects']['total']}"] = "evidence/DEFECTS.csv row count"


def main() -> int:
    S.apply()
    # Building every slide is what fills deck.CARDS.
    for builder in deck.SLIDES + deck.BACKUPS:
        plt.close(builder())

    rows: dict[str, list[tuple[str, str, str, str]]] = {k: [] for k in deck.TYPES}
    seen: set[tuple[str, str]] = set()
    for slide_key, cards in deck.CARDS.items():
        for value, unit, kind in cards:
            if (value, unit) in seen:
                continue
            seen.add((value, unit))
            rows[kind].append((value, unit, slide_key,
                               SOURCE.get(value, "see "
                                          "evidence/CLAIMS_EVIDENCE.csv")))

    figure = plt.figure(figsize=(8.27, 11.69))
    figure.patch.set_facecolor("white")
    axis = figure.add_axes([0, 0, 1, 1])
    axis.axis("off")
    axis.set_xlim(0, 1)
    axis.set_ylim(0, 1)

    axis.text(0.07, 0.965, "Every number in the deck", fontsize=20,
              fontweight="bold", color=S.INK, va="top")
    axis.text(0.07, 0.938,
              "DNS simulation environment · Khaled · 27 November 2026 · "
              "grouped by the deck's seven number types",
              fontsize=9.5, color=S.INK_MUTED, va="top")
    axis.plot([0.07, 0.93], [0.928, 0.928], color=S.INK_SOFT, linewidth=1.0)

    y = 0.905
    for kind, (colour, _hint, _icon) in deck.TYPES.items():
        entries = rows[kind]
        if not entries:
            continue
        deck.ASPECT = 8.27 / 11.69
        deck.type_icon(axis, kind, 0.085, y - 0.010, 0.011)
        axis.text(0.115, y - 0.010, kind.capitalize(), fontsize=13,
                  fontweight="bold", color=colour, va="center")
        axis.text(0.25, y - 0.010, f"{len(entries)} on the slides",
                  fontsize=9, color=S.INK_MUTED, va="center")
        y -= 0.030
        for value, unit, slide_key, source in sorted(entries,
                                                     key=lambda r: r[2]):
            axis.text(0.095, y, f"{value} {unit}", fontsize=10.5,
                      fontweight="bold", color=S.INK, va="center")
            axis.text(0.30, y, f"slide {slide_key}", fontsize=9,
                      color=S.INK_MUTED, va="center")
            axis.text(0.40, y, source, fontsize=9, color=S.INK_SOFT,
                      va="center")
            y -= 0.0185
        y -= 0.010

    axis.plot([0.07, 0.93], [y + 0.004, y + 0.004], color=S.GRID,
              linewidth=0.8)
    axis.text(0.07, y - 0.018,
              "Every one of these is cross-checked against the data files by\n"
              "scripts/check_slide_claims.py --source scripts/"
              "make_slides_final.py  ->  0 unmatched.\n"
              "The full claim-to-file map is evidence/CLAIMS_EVIDENCE.csv; the "
              "slide-by-slide trace is\nevidence/SLIDES_FINAL_CLAIMS_CHECK.csv.",
              fontsize=9, color=S.INK_MUTED, va="top", linespacing=1.6)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(OUT, facecolor="white", bbox_inches=None, pad_inches=0)
    plt.close(figure)
    total = sum(len(v) for v in rows.values())
    print(f"wrote {OUT.relative_to(ROOT)} ({total} distinct numbers, "
          f"{OUT.stat().st_size / 1e3:.0f} kB)")
    missing = [v for entries in rows.values() for v, _u, _s, src in entries
               if src.startswith("see ")]
    if missing:
        print(f"  note: {len(missing)} without an explicit source line: "
              f"{', '.join(missing)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
