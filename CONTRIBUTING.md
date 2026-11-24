# Contributing

This repository holds **one** contribution to the DNS project: the Webots
simulation environment. The PPO policy and raw sensor processing live in their
owners' own repositories and are deliberately not vendored here.

## Ground rules

1. **Do not commit other people's code.** Record the interface and the SHA-256
   fingerprint of what you ran against instead. `python scripts/freeze.py
   --verify` proves the frozen baseline is untouched.
2. **Do not hand-edit a `.wbt`.** Worlds are generated from
   `config/scenarios/*.json` by `scripts/build_worlds.py`. A hand edit is
   silently overwritten, and `dns/worldcheck.py` rejects the mismatch.
3. **Every number in a document must come from a file in the repository** and
   appear in `evidence/CLAIMS_EVIDENCE.csv`. If a file disagrees with a
   document, the file wins and the document is fixed.
4. **Label every visual** WEBOTS CAPTURE, LOGGED-DATA RENDER or SCHEMATIC.
5. **Never delete a failed run.** Negative and surprising results are the point.

## Branches and pull requests

Branch from `main` as `feature/<name>`, `fix/<name>` or `docs/<name>`. No
force-pushes to `main`, no history rewriting. Open a PR and work the checklist:

```
[ ] python scripts/security_scan.py      -> VERDICT: CLEAN
[ ] python tests/test_generator.py       -> 26 passed, 0 failed
[ ] python scripts/run_gates.py          -> every layout PASS
[ ] python reproduce.py --fast           -> figures and tables rebuild
[ ] every new number is in evidence/CLAIMS_EVIDENCE.csv
[ ] every new visual has a provenance label and a caption
[ ] no file over 50 MB; GIFs under 8 MB; MP4s go to Release assets
[ ] evidence/RELEASE_STATE.md updated
```

## Where things go

| You are adding | Put it in |
|---|---|
| A new layout family | `config/scenarios/generator_config.yaml`, then regenerate |
| An environment test tool | `scripts/` |
| A Webots client | `tests/client_*.py` |
| A design decision | a new ADR via `scripts/make_adrs.py` |
| A figure | `scripts/make_figures.py` or `make_visuals.py` — never a hand-drawn image |
| A teammate-facing guide | `docs/team/` |

## Issues

Label with one of `env`, `ppo`, `sensor`, `verification`, `decision-needed`,
`question`, `bug`. An issue reporting a measurement should link the run id and
the file it came from.
