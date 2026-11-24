# GITHUB_TODO.md — the steps I could not run

`gh` is **not installed** on this machine. Checked both shells and the usual
install locations:

```
$ gh auth status
bash: gh: command not found
PS> Get-Command gh        ->  not found
PS> Test-Path "$env:ProgramFiles\GitHub CLI\gh.exe"            -> False
PS> Test-Path "$env:LOCALAPPDATA\GitHubCLI\gh.exe"             -> False
PS> Test-Path "$env:LOCALAPPDATA\Microsoft\WinGet\Links\gh.exe"-> False
```

Git push works through the Windows credential manager, so **the branch and the
`v1.0` tag are already pushed**. Everything below needs the GitHub API. I did
not read or extract any stored credential.

If `gh` is on PATH in a fresh terminal, run `python scripts/github_finalise.py`
— it executes all of this in order. Otherwise paste the commands.

---

## 0. Before anything — fill in the names

Still `NAME NEEDED` / `TBC` in the deck, the README team table, `CITATION.cff`
and the role guides:

| Field | Set in |
|---|---|
| My full name | `scripts/make_slides.py` (`PRESENTER`), `scripts/make_team_docs.py` (`PEOPLE["me_name"]`) |
| Spencer's name + handle | `scripts/make_team_docs.py` (`PEOPLE["spencer_*"]`) |
| Sensor teammate's name + handle | `scripts/make_team_docs.py` (`PEOPLE["sensor_*"]`) |
| Dr. Mehdi's handle (optional) | `scripts/make_team_docs.py` (`PEOPLE["supervisor_handle"]`) |

Then:

```powershell
python scripts/make_team_docs.py
python scripts/make_slides.py
python scripts/make_support_docs.py
git commit -am "Fill in the team names across the docs, deck and citation"
git push
```

---

## 1. Check auth

```powershell
gh auth status
```

## 2. Structural render check of the README

```powershell
gh api -X POST markdown -f mode=gfm -f context=khh4lid/dns-webots-simulation `
  -f text="$(Get-Content -Raw README.md)" > evidence/readme_rendered.html
Select-String -Path evidence/readme_rendered.html -Pattern '\[!NOTE\]','\[!WARNING\]' 
# ^ expect NO matches: the alerts should have become <div class="markdown-alert">
Select-String -Path evidence/readme_rendered.html -Pattern '<details' | Measure-Object
# ^ expect 5
```

## 3. Pull request → merge → tag

```powershell
gh pr create --repo khh4lid/dns-webots-simulation `
  --base main --head v2-transparency-release `
  --title "v2: S6 generalisation + transparency release" `
  --body-file .github/PR_BODY.md

gh pr merge v2-transparency-release --merge --delete-branch=false
git checkout main; git pull
git tag -a v2.0 -m "v2.0 - S6 generalisation and transparency release"
git push origin v2.0
```

> Check the rendered README on the PR **before** merging — images load, the four
> GIFs animate, the Mermaid flowchart renders, alerts appear as coloured
> callouts, `<details>` expand, and it reads in dark mode and at mobile width.

## 4. Releases

```powershell
gh release create v1.0 --title "v1.0 - first research milestone" `
  --notes "One layout, 90 episodes, v1 verification report. State before the S6 release." `
  "../DNS_SIMULATION_RESEARCH_DELIVERABLE.zip#v1 deliverable package"

gh release create v2.0 --title "v2.0 - S6 generalisation and transparency release" `
  --notes-file CHANGELOG.md `
  "results/video/V1_main_demo.mp4#Main demonstration" `
  "results/video/V-TOP_L00_success.mp4#Top view - L00 success" `
  "results/video/V-TOP_L02_mirror.mp4#Top view - mirrored layout" `
  "results/video/V-TOP_L05_failure.mp4#Top view - hardest layout" `
  "results/video/V-SIDE_L00_altitude.mp4#Side view - altitude and heights" `
  "docs/meetings/2026-11-27/DNS_SimEnv_2026-11-27.pptx#Slides (PPTX, embedded video)" `
  "docs/meetings/2026-11-27/DNS_SimEnv_2026-11-27.pdf#Slides (PDF)"
```

## 5. About, topics, labels, issues

```powershell
gh repo edit khh4lid/dns-webots-simulation `
  --description "Reproducible Webots simulation environment for UAV navigation among static and dynamic obstacles. Seeded scenario generator, six-layout shift ladder, 540 episodes, every claim traced to a file." `
  --add-topic webots --add-topic uav --add-topic drone --add-topic quadcopter `
  --add-topic reinforcement-learning --add-topic ppo --add-topic lidar `
  --add-topic robotics-simulation --add-topic autonomous-navigation `
  --add-topic reproducibility

# the stray "نسخ الوصف" is removed by the --description above

foreach ($l in @("env:0072B2","ppo:009E73","sensor:E69F00","verification:56B4E9","decision-needed:D55E00","question:CC79A7","bug:B60205")) {
  $n,$c = $l.Split(":"); gh label create $n --color $c --force
}
```

### The seven issues

```powershell
gh issue create --label decision-needed --title "L04: exclude, or run a longer-horizon diagnostic?" --body "All five controllers score 0/90 on L04, but the reference tracker never reaches the goal either: stable and collision-free at <=0.48 m/s but timing out at the 90-step horizon, colliding above that. So 'the policies fail' cannot be separated from 'the layout may be outside the airframe's envelope within the horizon'. See docs/decisions/ADR-014-l04-excluded-generalisation-conclusion.md and results/layout_gate/L04.json. Owner: Dr. Mehdi."

gh issue create --label ppo --title "More PPO training seeds" --body "Three checkpoints cannot separate checkpoint-to-checkpoint variance from layout effects; on L00 Mode C they give 9/15, 8/15 and 7/15 with almost fully overlapping Wilson intervals. See results/v2_S6/pooled_rates.csv and docs/team/FOR_SPENCER.md. Owner: Spencer."

gh issue create --label sensor --title "Unexplained LiDAR reading: 0.0536 m vs 0.62 m ground-truth clearance" --body "A valid sector reported 0.0536 m while supervisor ground-truth clearance was 0.62 m. It matters because gap_rule's repulsion scales with (1.45 - range). Reproduce: python scripts/with_webots.py --boot-wait 12 --timeout 400 --world worlds/DNS_L00_C.wbt tests/client_capture.py -- --layout L00 --mode B --controller reactive --variant 0. See docs/team/FOR_SENSOR_TEAM.md. Owner: sensor teammate."

gh issue create --label env --title "Cart contact response is not physically modelled" --body "The cart is repositioned by Field.setSFVec3f, so Webots reports no rigid-body velocity for it and it cannot transfer momentum. A collision_cart outcome means the swept footprints intersected, not that an impact was simulated. Fix would be a Physics node driven by a motor. See ADR-006."

gh issue create --label env --title "A tumbled airframe cannot re-settle, aborting the next episode" --body "After a tumble the reset's 6 s settle budget is not enough (measured 0.023 m/s against a 0.02 m/s tolerance), so the following episode raises BackendError. Found while building the L04 reference traversal. See results/layout_gate/L04.json."

gh issue create --label verification --title "Stabilisation bracket: ideal gimbal vs body-fixed, not yet run" --body "The LiDAR mount is an ideal gimbal, which is a best case rather than a real sensor. Holding walls, mount height and beam count fixed and toggling stabilisation ON/OFF would bracket a real gimbal. See ADR-007."

gh issue create --label verification --title "Six layouts, one seed per family, is a pilot not a distribution" --body "No trend is fitted and none should be read into the scatter in assets/figures/E3_success_vs_descriptors.png. Extra seeds per family would need a fresh held-out set if any are used for training."
```

## 6. Collaborators — **blocked**

Section 0 left every username in brackets, so I cannot invite anyone. Once the
handles exist:

```powershell
gh api -X PUT repos/khh4lid/dns-webots-simulation/collaborators/SPENCER_HANDLE -f permission=push
gh api -X PUT repos/khh4lid/dns-webots-simulation/collaborators/SENSOR_HANDLE  -f permission=push
gh api -X PUT repos/khh4lid/dns-webots-simulation/collaborators/MEHDI_HANDLE   -f permission=pull
```

## 7. Keep it private — verify, do not assume

```powershell
gh repo view khh4lid/dns-webots-simulation --json isPrivate
# expect: {"isPrivate":true}
```
