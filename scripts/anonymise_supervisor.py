"""Remove the supervisor's name from every tracked file, replacing it with the role.

The name identifies a real person and the project owner asked for it to be
removed. Only the *role* is kept, because several documents genuinely need to
say who a decision belongs to.

Two properties matter:

* **Longest pattern first.** "Dr. Mehdi Samieiyeganeh" is replaced before
  "Dr. Mehdi", which is replaced before "Mehdi". Replacing the short form first
  would leave an orphan surname behind.
* **The generators are edited too.** Several documents are generated
  (`make_slides.py`, `make_team_docs.py`, `make_adrs.py`, ...), so editing only
  their output would reintroduce the name on the next regeneration.

Git history and existing tags still contain the name. Rewriting published
history is out of scope and was not requested; this script only changes the
working tree going forward.

Run::

    python scripts/anonymise_supervisor.py --check
    python scripts/anonymise_supervisor.py
"""
from __future__ import annotations

import argparse
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

#: Ordered longest-first. Each entry is (pattern, replacement). Patterns are
#: case-insensitive; replacements are chosen so the surrounding grammar still
#: reads correctly, including possessives and the GitHub handle placeholder.
RULES: list[tuple[str, str]] = [
    # handle placeholders, before the bare name
    (r"MEHDI_HANDLE", "SUPERVISOR_HANDLE"),
    (r"Dr\.?\s*Mehdi\s+Samieiyeganeh's", "the supervisor's"),
    (r"Dr\.?\s*Mehdi\s+Samieiyeganeh", "the supervisor"),
    (r"Mehdi\s+Samieiyeganeh's", "the supervisor's"),
    (r"Mehdi\s+Samieiyeganeh", "the supervisor"),
    (r"Dr\.?\s*Mehdi'?s\b", "the supervisor's"),
    (r"Dr\.?\s*Mehdi\b", "the supervisor"),
    (r"Mehdi'?s\b", "the supervisor's"),
    (r"\bMehdi\b", "the supervisor"),
    (r"\bSamieiyeganeh\b", "the supervisor"),
]

#: Tidy-ups for phrasings that become clumsy once the name is a role noun.
TIDY: list[tuple[str, str]] = [
    ("**the supervisor (supervisor)**", "**the supervisor**"),
    ("the supervisor (supervisor)", "the supervisor"),
    ("| **the supervisor** | Supervisor |", "| **The supervisor** | Supervisor |"),
    ("Supervised by the supervisor at", "Supervised at"),
    ("supervised by the supervisor)", "supervised)"),
    ("Supervisor: the supervisor", "Supervisor: see the team"),
    ("**Supervisor:** the supervisor ·", "**Supervisor:** (name withheld) ·"),
    ("**Supervisor:** the supervisor\n", "**Supervisor:** (name withheld)\n"),
    ("· Supervisor: **the supervisor** ·", "· Supervisor: (name withheld) ·"),
    ('"supervisor": "the supervisor",', '"supervisor": "(name withheld)",'),
    ("- the supervisor, the supervisor", "- the supervisor"),
    ("the supervisor, Spencer (PPO), sensor", "the supervisor, Spencer (PPO), sensor"),
]

TEXT_SUFFIXES = {".py", ".md", ".txt", ".json", ".yaml", ".yml", ".csv",
                 ".cff", ".bib", ".wbt", ".proto", ".bat", ".ps1", ".sh"}

PATTERN = re.compile(r"mehdi|samieiyeganeh", re.IGNORECASE)


def tracked() -> list[str]:
    out = subprocess.run(["git", "ls-files"], cwd=ROOT, capture_output=True,
                         text=True).stdout
    return [line for line in out.splitlines() if line]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    before = after = 0
    touched: list[tuple[str, int]] = []
    for relative in tracked():
        path = ROOT / relative
        if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        try:
            text = path.read_text(encoding="utf8")
        except (OSError, UnicodeDecodeError):
            continue
        hits = len(PATTERN.findall(text))
        if not hits:
            continue
        before += hits

        new = text
        for pattern, replacement in RULES:
            new = re.sub(pattern, replacement, new, flags=re.IGNORECASE)
        for old, replacement in TIDY:
            new = new.replace(old, replacement)

        remaining = len(PATTERN.findall(new))
        after += remaining
        touched.append((relative, hits))
        if not args.check and new != text:
            path.write_text(new, encoding="utf8")

    verb = "would change" if args.check else "changed"
    for relative, hits in touched:
        print(f"  {verb}  {relative}  ({hits} occurrence(s))")
    print(f"\nfiles: {len(touched)}   occurrences before: {before}   "
          f"remaining after replacement: {after}")
    if after:
        print("!! some occurrences survived -- add a rule for them")
    return 1 if after else 0


if __name__ == "__main__":
    raise SystemExit(main())
