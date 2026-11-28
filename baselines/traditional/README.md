# Traditional path-planning baselines: A* and RRT

**Author:** Ariana (traditional algorithms and performance evaluation; spelling of the name to confirm).
**Status:** 2D prototype complete (Google Colab), received 28 Nov 2026. The files below are her originals, unmodified.
**Role in the project:** the traditional-algorithm baseline that the team's PPO policy will be compared against. See the [research plan](../../docs/RESEARCH_PLAN.md), work package WP-BASE.

| File | What it is |
|---|---|
| [`Drone_Navigation_Traditional_Algorithms.ipynb`](Drone_Navigation_Traditional_Algorithms.ipynb) | Colab notebook: A* and RRT implementations, experiments, tables and plots, with outputs |
| [`Traditional_Path_Planning_Progress_Report.pdf`](Traditional_Path_Planning_Progress_Report.pdf) | Progress report (19 pages): aim, method, results, limitations, future work |
| [`Traditional_Algorithms_Drone_Navigation_Slides.pptx`](Traditional_Algorithms_Drone_Navigation_Slides.pptx) | Presentation (8 slides) |

---

## What was done

Two classical planners on 10 × 10 two-dimensional occupancy grids: free cells are 0 and obstacles are 1. Start is (0, 0) and goal is (9, 9), fixed. Only static obstacles.

| Algorithm | Settings |
|---|---|
| **A\*** | 4-connected grid, unit move cost, Manhattan heuristic, `heapq` priority queue; records explored nodes |
| **RRT** | step size 1.0, at most 500 iterations, goal threshold 1.0; segment collision check by sampling points along each edge |
| **RRT with goal bias** | the goal is sampled with probability 0 %, 5 %, 10 % or 20 % (Map 3 only) |

- **Trials:** 10 per configuration. The RRT trials are seeded with `random.seed(0…9)`, so they are reproducible.
- **Maps:** Map 1 was the development grid. Map 2 has horizontal obstacle sections. Map 3 has three vertical barriers with openings.

## Results

All values below are from the notebook outputs and match the report and slides.

**Map 2 (horizontal obstacles)**

| Metric | A\* | RRT (0 % goal bias) |
|---|---|---|
| Success | 10/10 | 10/10 |
| Mean planning time | 0.145 ms | 7.65 ms |
| Mean path length | 18 grid steps | 19.05 units |
| Collision-free (of returned paths) | 100 % | 100 % |

**Map 3 (vertical barriers), including the goal-bias sweep**

| Configuration | Success | Mean planning time | Mean path length (successful runs) |
|---|---|---|---|
| A\* | 10/10 | 0.220 ms | 18 grid steps |
| RRT, 0 % goal bias | 6/10 | 15.76 ms | 16.55 units |
| RRT, 5 % | 7/10 | 12.84 ms | 15.92 units |
| RRT, 10 % | **8/10** | **10.91 ms** | 15.64 units |
| RRT, 20 % | 7/10 | 15.34 ms | 15.23 units |

**Her conclusion:** A\* was the most consistent on these static grids. A moderate goal bias (10 %) gave RRT its best balance of success and planning time; 20 % did not improve further.

## Reading these numbers correctly

These notes are for anyone comparing these numbers with ours; they are not criticisms of the work.

- **A\* is deterministic.** Its 10 trials repeat the same search, so they measure timing variation, not success variation.
- **Path lengths are not comparable across planners.** A\* counts grid steps; RRT measures Euclidean length. The report says so.
- **Planning times are Colab wall-clock times.** They depend on the machine.
- **The sample is small.** With 10 trials, a 6/10 vs 8/10 difference is inside the uncertainty: the 95 % Wilson intervals are [0.31, 0.83] and [0.49, 0.94].
- **The Excel export is not here.** The notebook's last cell writes `Drone_Navigation_Experimental_Results.xlsx` and downloads it through `google.colab`, so outside Colab that cell fails.

## How to run

Open the notebook in Google Colab, or in Jupyter with `numpy`, `matplotlib` and `pandas`, and run the cells in order. Only the last cell, the export, needs Colab.

## Next step: the same planners in our Webots world

The report names a 3D simulator and dynamic obstacles as future work. The team's simulator is **Webots R2025a** (this repository), so the port targets it. Following the [research plan](../../docs/RESEARCH_PLAN.md), WP-BASE:

1. **Build the grid.** Make an occupancy grid from each scenario file (`config/scenarios/L0*_C.json`, 10 × 6 m) at 0.05 m resolution. Inflate obstacles by the drone's 0.18 m collision radius plus a safety margin.
2. **Plan.** Run A\* (8-connected) and RRT with 10 % goal bias, and optionally RRT\* and a reactive local planner, from the scenario's start to its goal.
3. **Fly the plan in Webots.** Use the existing reference tracker, with the same 0.2 s command interface and 1.2 m/s limit as the PPO policy.
4. **Two variants, so the comparison is fair.**
   - *Oracle map:* the full static map is known. This is an upper bound.
   - *LiDAR replanning:* every 0.2 s, replan on an occupancy grid built only from the LiDAR scan. Same information as the PPO policy.
5. **Measure the same way.** Use the same metrics, episodes, cart timings and 95 % intervals as the PPO evaluation (`results/v2_S6/` format). Run at least 30 trials per configuration.
