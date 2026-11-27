# GITHUB_WEB_STEPS.md — click-by-click, no CLI needed

`gh` is not on PATH on this machine, so everything below is the browser route.
Git itself works, so **the branch and both tags are already pushed** — you are
only doing the parts that need the web UI.

Keep `GITHUB_TODO.md` for when `gh` is available; it is the same actions as CLI
commands.

Repository: <https://github.com/khh4lid/dns-webots-simulation> · keep it
**private** throughout. Nothing below changes visibility.

---

## 1 · Open the pull request

1. Go to **<https://github.com/khh4lid/dns-webots-simulation/pull/new/v2-transparency-release>**
2. Confirm the header reads **`base: main` ← `compare: v2-transparency-release`**.
3. **Title:** `v2: S6 generalisation + transparency release`
4. **Body:** open `.github/PR_BODY.md` in a text editor, select all, paste.
5. Leave *Allow edits by maintainers* as it is. Click **Create pull request**.

> Do **not** merge yet — step 2 first.

---

## 2 · Eye-check the rendered README on the branch

Open <https://github.com/khh4lid/dns-webots-simulation/tree/v2-transparency-release>
and confirm, scrolling top to bottom:

- [ ] Images load (not broken icons, not raw `<img>` text)
- [ ] The four GIFs in **Watch** animate
- [ ] The Mermaid flowchart under *How it works* renders as a diagram
- [ ] `[!NOTE]` / `[!IMPORTANT]` appear as coloured callouts, **not literal text**
- [ ] The five `<details>` sections expand when clicked
- [ ] Links in the *Start here* role table resolve
- [ ] Readable with the dark-mode toggle on, and at a narrow browser width

If any check fails, fix it on the branch and push before merging — the PR
updates itself.

---

## 3 · Merge

1. On the PR page, click **Merge pull request**.
2. In the dropdown choose **Create a merge commit**.
   **Not** *Squash and merge*. **Not** *Rebase and merge* — both rewrite the
   commit history this release deliberately preserves.
3. Confirm. **Do not** delete the branch — `reset-fix-v2.1` is built on it.

---

## 4 · Tag the merge commit as v2.0

1. Go to **<https://github.com/khh4lid/dns-webots-simulation/releases/new>**
2. Click **Choose a tag** → type `v2.0` → **Create new tag: v2.0 on publish**.
3. **Target:** select **`main`** (now at the merge commit).
4. **Release title:** `v2.0 — S6 generalisation and transparency release`
5. **Description:** open `CHANGELOG.md`, copy the `## [2.0]` section, paste.
6. Tick **Set as a pre-release** — this is a research pre-release, not a
   finished artefact.

### Upload these files (drag into the attachment box)

All paths below are relative to the repository root — the folder containing
`README.md`. Open it in Explorer and drag the files from there.

| File | What it is |
|---|---|
| `results/video/V-TOP_L00_success.mp4` | Top view — PPO reaches the goal on L00 |
| `results/video/V-TOP_L02_mirror.mp4` | Top view — the mirrored layout |
| `results/video/V-TOP_L05_failure.mp4` | Top view — the hardest layout |
| `results/video/V-SIDE_L00_altitude.mp4` | Side view — altitude vs obstacle heights |
| `results/video/V1_main_demo.mp4` | The original combined demo |
| `docs/meetings/2026-11-27/DNS_SimEnv_2026-11-27.pptx` | Slides, embedded video |
| `docs/meetings/2026-11-27/DNS_SimEnv_2026-11-27.pdf` | Slides, PDF fallback |

7. Click **Publish release**.

> The four MP4 links in the README point at this release page. They will 404
> until you publish it, and resolve immediately afterwards — no README edit needed.

---

## 5 · Also publish v1.0 (the tag already exists)

1. **<https://github.com/khh4lid/dns-webots-simulation/releases/new>**
2. **Choose a tag** → pick the existing **`v1.0`** from the list.
3. Title: `v1.0 — first research milestone`
4. Description: *One layout, 90 episodes, the v1 verification report. The state
   before the S6 release.*
5. Attach `DNS_SIMULATION_RESEARCH_DELIVERABLE.zip` — it sits one directory
   **above** the repository root.
6. Tick **Set as a pre-release**. Publish.

---

## 6 · About text and topics

1. Go to the repository home page.
2. Click the **gear icon** beside *About* (top right).
3. **Description** — replace the whole field, which removes the stray
   `نسخ الوصف`. Paste:

```
Reproducible Webots simulation environment for UAV navigation among static and dynamic obstacles. Seeded scenario generator, six-layout shift ladder, 540 episodes, every claim traced to a file.
```

4. **Topics** — add these ten, pressing Enter after each:

```
webots  uav  drone  quadcopter  reinforcement-learning
ppo  lidar  robotics-simulation  autonomous-navigation  reproducibility
```

5. Untick **Releases** only if you do not want them in the sidebar; otherwise
   leave the checkboxes alone. **Save changes**.

---

## 7 · Labels

**<https://github.com/khh4lid/dns-webots-simulation/labels>** → **New label**,
seven times:

| Name | Colour |
|---|---|
| `env` | `0072B2` |
| `ppo` | `009E73` |
| `sensor` | `E69F00` |
| `verification` | `56B4E9` |
| `decision-needed` | `D55E00` |
| `question` | `CC79A7` |
| `bug` | `B60205` |

---

## 8 · The eight issues

**<https://github.com/khh4lid/dns-webots-simulation/issues/new>** for each. The
titles and bodies are written out in **`GITHUB_TODO.md` § 5** — copy each
`--title` value into the title box and each `--body` value into the body box.

| # | Title | Label |
|---|---|---|
| 1 | L04: exclude, or run a longer-horizon diagnostic? | `decision-needed` |
| 2 | More PPO training seeds | `ppo` |
| 3 | Unexplained LiDAR reading: 0.0536 m vs 0.62 m ground-truth clearance | `sensor` |
| 4 | Cart contact response is not physically modelled | `env` |
| 5 | A tumbled airframe cannot re-settle, aborting the next episode | `env` |
| 6 | Stabilisation bracket: ideal gimbal vs body-fixed, not yet run | `verification` |
| 7 | Six layouts, one seed per family, is a pilot not a distribution | `verification` |
| 8 | **Merge reset-fix-v2.1 after a full re-evaluation** — see below | `decision-needed` |

Issue 8 is new tonight; body to paste:

```
The deterministic reset on branch reset-fix-v2.1 removes the residual-state
coupling: a fixed-step settle instead of settle-to-tolerance. It clears the
order-independence bar on 4 of 5 targets (three at ~5e-10 m) but one still
drifts 2.1e-04 m against a 1e-05 m threshold, so it was NOT merged for the
27 Nov build. The full 540-episode protocol has already been re-run under it
(results/v2_S6_deterministic/) and the cell-by-cell comparison is in
docs/verification/RESET_MODE_COMPARISON.md: the mirror result survives and no
cell moves by more than two episodes.

To merge: close the 2.1e-04 m gap on ppo_seed7_v01, then regenerate the
analysis, figures, README, slides and both claims checks from the deterministic
cohort. Owner: Khaled. Decision needed from the supervisor on whether v2.1 is worth
the re-run.
```

---

## 9 · Collaborators — **blocked, nothing to do yet**

Spencer's and the sensor teammate's GitHub handles were never supplied, so no
invitation can be sent. Once you have them:

**Settings → Collaborators → Add people** — Spencer and the sensor teammate as
**Write**, the supervisor as **Read**.

---

## 10 · Final check

- [ ] Repository still shows **Private** beside its name
- [ ] `main` is at the merge commit; `v2-transparency-release` still exists
- [ ] Tags `v1.0`, `v2.0` and `meeting-2026-11-27` all listed under **Tags**
- [ ] Release v2.0 shows 7 attachments and is marked **Pre-release**
- [ ] The four README video links open the v2.0 release page
