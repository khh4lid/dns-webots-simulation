"""Check that every relative link, image and code path in the docs resolves.

Catches the three things that silently rot in a README: a moved figure, a
renamed document, and a command that names a file which no longer exists.

Run::

    python scripts/check_docs.py
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]

LINK = re.compile(r"!?\[(?P<text>[^\]]*)\]\((?P<href>[^)\s]+)(?:\s+\"[^\"]*\")?\)")
IMAGE = re.compile(r"!\[(?P<alt>[^\]]*)\]\((?P<href>[^)\s]+)\)")
# Only paths that name a directory are checked. A bare basename in prose --
# `planar.py`, `protocol.json` -- is usually a reference to a teammate-owned
# file or a short-hand for one of several copies, not a repo-relative path.
BACKTICK_PATH = re.compile(
    r"`([A-Za-z0-9_-]+(?:/[A-Za-z0-9_.-]+)+\.(?:py|json|wbt|proto|csv|md|mp4|png|jpg))`")

#: Documents that are deliberately untracked are not part of the published set.
EXCLUDE = {"M0_environment_audit.md"}

DOCS = [PROJECT_ROOT / "README.md",
        *(p for p in sorted((PROJECT_ROOT / "docs").glob("*.md"))
          if p.name not in EXCLUDE)]


def anchor(text: str) -> str:
    slug = text.strip().lower()
    slug = re.sub(r"[^\w\s-]", "", slug)
    return re.sub(r"[\s]+", "-", slug)


def headings(path: Path) -> set[str]:
    out: set[str] = set()
    for line in path.read_text(encoding="utf8").splitlines():
        if line.startswith("#"):
            out.add(anchor(line.lstrip("# ")))
    return out


def main() -> int:
    problems: list[str] = []
    checked = {"links": 0, "images": 0, "paths": 0, "alt": 0}

    for doc in DOCS:
        if not doc.is_file():
            problems.append(f"{doc.name}: missing document")
            continue
        text = doc.read_text(encoding="utf8")
        rel = doc.relative_to(PROJECT_ROOT)

        for m in IMAGE.finditer(text):
            checked["images"] += 1
            alt = m.group("alt").strip()
            if len(alt) < 15:
                problems.append(f"{rel}: image alt text too short or missing: "
                                f"{m.group('href')}")
            else:
                checked["alt"] += 1

        for m in LINK.finditer(text):
            href = m.group("href")
            if href.startswith(("http://", "https://", "mailto:")):
                continue
            checked["links"] += 1
            target, _, frag = href.partition("#")
            if not target:
                if frag and frag not in headings(doc):
                    problems.append(f"{rel}: in-page anchor #{frag} has no heading")
                continue
            resolved = (doc.parent / target).resolve()
            if not resolved.exists():
                problems.append(f"{rel}: link target does not exist: {href}")
            elif frag and resolved.suffix == ".md" and frag not in headings(resolved):
                problems.append(f"{rel}: anchor #{frag} not found in {target}")

        for m in BACKTICK_PATH.finditer(text):
            candidate = m.group(1)
            if candidate.startswith(("http", "navigation.", "evaluate.")):
                continue
            checked["paths"] += 1
            if not (PROJECT_ROOT / candidate).exists() and \
               not (doc.parent / candidate).exists():
                problems.append(f"{rel}: referenced path not found: {candidate}")

    print(f"checked {len(DOCS)} documents: {checked['links']} relative links, "
          f"{checked['images']} images ({checked['alt']} with usable alt text), "
          f"{checked['paths']} file references\n")
    if problems:
        for p in problems:
            print(f"  FAIL {p}")
        print(f"\n{len(problems)} problem(s)")
        return 1
    print("all links, images and referenced paths resolve")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
