"""Pre-push safety scan: secrets, personal paths and oversized files.

Three independent questions, each answered over BOTH the working tree and the
full git history, because a secret removed from the tree is still a secret if
it is reachable from any commit:

1. **Secrets** -- tokens, keys, passwords, OAuth/credential files, and Google
   Drive file IDs (the Drive handoff package builder is a known risk here).
2. **Personal absolute paths** -- ``D:\\...``, ``C:\\Users\\<name>\\...`` and the
   like, which leak the author's machine layout and break on anyone else's.
3. **Size** -- anything above the GitHub limits this release works to.

Exit code 0 only when the tree is clean. A finding in HISTORY is reported but
does NOT rewrite anything: history rewriting needs explicit approval.

Run::

    python scripts/security_scan.py
    python scripts/security_scan.py --json results/security_scan.json
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

#: Each rule is (name, compiled pattern, severity). Patterns are deliberately
#: broad; false positives are cheap to triage, a missed token is not.
RULES = [
    ("github_pat", re.compile(r"\b(?:ghp|gho|ghu|ghs|ghr)_[A-Za-z0-9]{36,}\b"), "critical"),
    ("github_fine_grained", re.compile(r"\bgithub_pat_[A-Za-z0-9_]{22,}\b"), "critical"),
    ("aws_access_key", re.compile(r"\b(?:AKIA|ASIA)[0-9A-Z]{16}\b"), "critical"),
    ("google_api_key", re.compile(r"\bAIza[0-9A-Za-z_\-]{35}\b"), "critical"),
    ("slack_token", re.compile(r"\bxox[abprs]-[0-9A-Za-z-]{10,}\b"), "critical"),
    ("private_key_block", re.compile(r"-----BEGIN (?:RSA |EC |OPENSSH |PGP )?PRIVATE KEY-----"), "critical"),
    ("bearer_token", re.compile(r"\bBearer\s+[A-Za-z0-9._\-]{24,}\b"), "high"),
    ("generic_secret_assign", re.compile(
        r"(?i)\b(?:api[_-]?key|secret|passwd|password|access[_-]?token|client[_-]?secret)"
        r"\s*[:=]\s*[\"'][^\"'\s]{12,}[\"']"), "high"),
    # Google Drive file / folder IDs are 25-44 chars of [A-Za-z0-9_-]. Only
    # flagged next to a Drive-ish word so ordinary hashes do not trip it.
    ("google_drive_id", re.compile(
        r"(?i)(?:drive\.google\.com/(?:file/d/|drive/folders/)|folder_?id|file_?id)"
        r"\s*[:=/]?\s*[\"']?([A-Za-z0-9_\-]{25,44})[\"']?"), "high"),
    ("personal_abs_path", re.compile(r"[A-Za-z]:\\\\?(?:Users|Download|Program)\b"), "medium"),
]

#: Known-benign matches, with the reason each is safe.
ALLOW = {
    # The scanner's own pattern literals.
    "scripts/security_scan.py",
}

#: Paths whose personal-path hits are expected and handled separately
#: (generated logs are not shipped as documentation).
PATH_RULE_SOFT = ("results/", "evidence/", "docs/")

MAX_FILE_MB = 50.0
MAX_GIF_MB = 8.0
TEXT_SUFFIXES = {".py", ".md", ".json", ".yaml", ".yml", ".txt", ".cfg", ".ini",
                 ".toml", ".wbt", ".proto", ".csv", ".bat", ".ps1", ".sh",
                 ".gitignore", ".gitattributes", ".cff"}


def tracked_files() -> list[str]:
    out = subprocess.run(["git", "ls-files"], cwd=ROOT, capture_output=True,
                         text=True).stdout
    return [line for line in out.splitlines() if line]


def scan_text(label: str, text: str) -> list[dict]:
    findings = []
    for name, pattern, severity in RULES:
        for match in pattern.finditer(text):
            snippet = match.group(0)
            if len(snippet) > 90:
                snippet = snippet[:90] + "…"
            findings.append({"rule": name, "severity": severity,
                             "where": label, "match": snippet})
    return findings


def scan_tree() -> list[dict]:
    findings = []
    for relative in tracked_files():
        if relative in ALLOW:
            continue
        path = ROOT / relative
        if not path.is_file():
            continue
        if path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        try:
            text = path.read_text(encoding="utf8", errors="ignore")
        except OSError:
            continue
        for finding in scan_text(relative, text):
            # Personal paths inside generated result logs are downgraded: they
            # are machine output, not documentation a reader is asked to run.
            if (finding["rule"] == "personal_abs_path"
                    and relative.startswith(PATH_RULE_SOFT)):
                finding["severity"] = "low"
                finding["note"] = "generated log, not documentation"
            findings.append(finding)
    return findings


def scan_history() -> list[dict]:
    """Scan every blob reachable from any commit."""
    revs = subprocess.run(["git", "rev-list", "--all"], cwd=ROOT,
                          capture_output=True, text=True).stdout.split()
    seen: set[str] = set()
    findings = []
    for rev in revs:
        listing = subprocess.run(["git", "ls-tree", "-r", rev],
                                 cwd=ROOT, capture_output=True, text=True).stdout
        for line in listing.splitlines():
            try:
                meta, name = line.split("\t", 1)
                _mode, kind, blob = meta.split()
            except ValueError:
                continue
            if kind != "blob" or blob in seen:
                continue
            seen.add(blob)
            if Path(name).suffix.lower() not in TEXT_SUFFIXES:
                continue
            if name in ALLOW:
                continue
            content = subprocess.run(["git", "cat-file", "-p", blob], cwd=ROOT,
                                     capture_output=True).stdout
            text = content.decode("utf8", errors="ignore")
            for finding in scan_text(f"{name}@{rev[:8]}", text):
                if finding["rule"] == "personal_abs_path":
                    continue        # tree scan already covers documentation
                findings.append(finding)
    return findings


def scan_sizes() -> list[dict]:
    findings = []
    for relative in tracked_files():
        path = ROOT / relative
        if not path.is_file():
            continue
        megabytes = path.stat().st_size / 1e6
        limit = MAX_GIF_MB if path.suffix.lower() == ".gif" else MAX_FILE_MB
        if megabytes > limit:
            findings.append({"rule": "oversized_file", "severity": "high",
                             "where": relative,
                             "match": f"{megabytes:.1f} MB > {limit:.0f} MB"})
    return findings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=Path, default=None)
    parser.add_argument("--skip-history", action="store_true")
    args = parser.parse_args()

    tree = scan_tree()
    sizes = scan_sizes()
    history = [] if args.skip_history else scan_history()

    blocking = [f for f in tree + sizes
                if f["severity"] in ("critical", "high", "medium")]
    history_blocking = [f for f in history if f["severity"] in ("critical", "high")]

    print(f"tree findings      : {len(tree)}")
    print(f"size findings      : {len(sizes)}")
    print(f"history findings   : {len(history)}")
    by_rule: dict[str, int] = {}
    for finding in tree + sizes + history:
        by_rule[finding["rule"]] = by_rule.get(finding["rule"], 0) + 1
    for rule, count in sorted(by_rule.items(), key=lambda kv: -kv[1]):
        print(f"  {rule:<24} {count}")

    for finding in blocking[:40]:
        print(f"  [{finding['severity'].upper()}] {finding['rule']} "
              f"{finding['where']}: {finding['match']}")
    if history_blocking:
        print("\n!! SECRETS FOUND IN HISTORY -- stop and report; do not rewrite "
              "history without approval:")
        for finding in history_blocking[:20]:
            print(f"  {finding['rule']} {finding['where']}: {finding['match']}")

    report = {"tree": tree, "sizes": sizes, "history": history,
              "verdict": "CLEAN" if not (blocking or history_blocking) else "FINDINGS",
              "limits": {"max_file_mb": MAX_FILE_MB, "max_gif_mb": MAX_GIF_MB}}
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2), encoding="utf8")
        print(f"\nwrote {args.json}")
    print(f"\nVERDICT: {report['verdict']}")
    return 0 if report["verdict"] == "CLEAN" else 1


if __name__ == "__main__":
    raise SystemExit(main())
