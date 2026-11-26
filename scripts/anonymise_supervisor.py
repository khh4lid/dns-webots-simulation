"""Replace a person's name with their role throughout the working tree.

Written to remove the supervisor's name from this repository at the project
owner's request, and kept so the check can be re-run.

**The name is not stored in this file.** It is supplied on the command line, or
read from a file outside the repository, because hard-coding the search pattern
would put the very name being removed back into a tracked file. A SHA-256
digest of the expected name is recorded instead, so a future run can confirm it
was given the same name without the name itself ever appearing here.

Two properties matter:

* **Longest pattern first.** The full "Title Given Family" form is replaced
  before "Title Given", which is replaced before the bare given name.
  Replacing the short form first would leave an orphan surname behind.
* **The generators are edited too.** Several documents in this repository are
  generated (``make_slides``, ``make_team_docs``, ``make_adrs``,
  ``make_report``, ``make_deliverable_report``, ``build_drive_package``), so
  editing only their output would reintroduce the name on the next
  regeneration.

Git history and existing tags still contain whatever was published. Rewriting
published history is out of scope and was not requested; this script only
changes the working tree going forward.

Run::

    python scripts/anonymise_supervisor.py --name "Given Family" --check
    python scripts/anonymise_supervisor.py --name "Given Family"
    python scripts/anonymise_supervisor.py --name-file ../name.txt

With no name supplied it still reports whether the previously removed name is
present, using the recorded digest, and exits non-zero if it reappears.
"""
from __future__ import annotations

import argparse
import hashlib
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

#: SHA-256 of the lower-cased name that was removed from this repository on
#: 27 November 2026, as "given family". Recorded so a re-run can verify it was
#: handed the same name, without the name being written down here.
EXPECTED_DIGEST = "482710e9bd5d3c6a74415e7a4046bae76f1d193b402dddea93db8afb594d4185"

#: Replacement for the role. The repository keeps the role because several
#: documents genuinely need to say whose decision something is.
ROLE = "the supervisor"

TEXT_SUFFIXES = {".py", ".md", ".txt", ".json", ".yaml", ".yml", ".csv",
                 ".cff", ".bib", ".wbt", ".proto", ".bat", ".ps1", ".sh"}

#: Tidy-ups for phrasings that read badly once a name becomes a role noun.
TIDY: list[tuple[str, str]] = [
    (f"**{ROLE} (supervisor)**", f"**{ROLE}**"),
    (f"{ROLE} (supervisor)", ROLE),
    (f"| **{ROLE}** | Supervisor |", "| **The supervisor** | Supervisor |"),
    (f"Supervised by {ROLE} at", "Supervised at"),
    (f"supervised by {ROLE})", "supervised)"),
    (f"Supervisor: {ROLE}", "Supervisor: (name withheld by request)"),
    (f"**Supervisor:** {ROLE} ·", "**Supervisor:** (name withheld) ·"),
    (f"· Supervisor: **{ROLE}** ·", "· Supervisor: (name withheld) ·"),
    (f'"supervisor": "{ROLE}",', '"supervisor": "(name withheld)",'),
    (f"- {ROLE}, {ROLE}", f"- {ROLE}"),
]


def digest(name: str) -> str:
    return hashlib.sha256(name.strip().lower().encode("utf8")).hexdigest()


def build_rules(name: str) -> list[tuple[str, str]]:
    """Longest-first substitutions derived from a "Given Family" name."""
    parts = name.split()
    given, family = parts[0], parts[-1] if len(parts) > 1 else ""
    g, f = re.escape(given), re.escape(family)
    title = r"(?:Dr\.?|Prof\.?|Mr\.?|Ms\.?|Mrs\.?)\s*"
    rules: list[tuple[str, str]] = [
        (rf"{given.upper()}_HANDLE", "SUPERVISOR_HANDLE"),
    ]
    if family:
        rules += [
            (rf"{title}{g}\s+{f}'s", f"{ROLE}'s"),
            (rf"{title}{g}\s+{f}", ROLE),
            (rf"{g}\s+{f}'s", f"{ROLE}'s"),
            (rf"{g}\s+{f}", ROLE),
        ]
    rules += [
        (rf"{title}{g}'?s\b", f"{ROLE}'s"),
        (rf"{title}{g}\b", ROLE),
        (rf"{g}'?s\b", f"{ROLE}'s"),
        (rf"\b{g}\b", ROLE),
    ]
    if family:
        rules.append((rf"\b{f}\b", ROLE))
    return rules


def tracked() -> list[str]:
    out = subprocess.run(["git", "ls-files"], cwd=ROOT, capture_output=True,
                         text=True).stdout
    return [line for line in out.splitlines() if line]


def text_files():
    for relative in tracked():
        path = ROOT / relative
        if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        try:
            yield relative, path, path.read_text(encoding="utf8")
        except (OSError, UnicodeDecodeError):
            continue


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", help='the name to remove, "Given Family"')
    parser.add_argument("--name-file", type=Path,
                        help="read the name from this file (keep it outside "
                             "the repository)")
    parser.add_argument("--check", action="store_true",
                        help="report what would change, write nothing")
    args = parser.parse_args()

    name = args.name
    if args.name_file:
        name = args.name_file.read_text(encoding="utf8").strip()
    if not name:
        print("No name supplied. Re-run with --name or --name-file to repeat "
              "the substitution.\nThis repository's tracked text files are "
              "checked below only for the\nplaceholder role string, which is "
              "expected to be present.")
        role_hits = sum(1 for _r, _p, t in text_files() if ROLE in t)
        print(f'  files mentioning "{ROLE}": {role_hits}')
        return 0

    if digest(name) != EXPECTED_DIGEST:
        print("note: the name given does not match the digest recorded in this "
              "script.\n      That is fine for a new name; it means this is not "
              "a re-run of the\n      original substitution.")

    rules = build_rules(name)
    pattern = re.compile("|".join(re.escape(p) for p in name.split()
                                  if len(p) > 2), re.IGNORECASE)

    before = after = 0
    touched: list[tuple[str, int]] = []
    for relative, path, text in text_files():
        hits = len(pattern.findall(text))
        if not hits:
            continue
        before += hits
        new = text
        for rule, replacement in rules:
            new = re.sub(rule, replacement, new, flags=re.IGNORECASE)
        for old, replacement in TIDY:
            new = new.replace(old, replacement)
        after += len(pattern.findall(new))
        touched.append((relative, hits))
        if not args.check and new != text:
            path.write_text(new, encoding="utf8")

    verb = "would change" if args.check else "changed"
    for relative, hits in touched:
        print(f"  {verb}  {relative}  ({hits} occurrence(s))")
    print(f"\nfiles: {len(touched)}   occurrences before: {before}   "
          f"remaining after replacement: {after}")
    if after:
        print("!! some occurrences survived -- extend build_rules() for them")
    return 1 if after else 0


if __name__ == "__main__":
    sys.exit(main())
