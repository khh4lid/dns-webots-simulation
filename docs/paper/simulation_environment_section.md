# Simulation Environment — paper section (draft)

> **Note to the team, not part of the paper.** This section was prepared by
> Khaled and covers only the simulation-environment contribution: platform
> selection, navigation-environment design, start and goal definition, static
> obstacles, the moving-obstacle model, the scenario generator and the
> verification of all of these. It deliberately does not describe policy
> training, reward design, network architecture or the sensor-processing
> algorithm; those belong to Sections [PPO] and [Perception] and are referred to
> by cross-reference only. Section numbers are written as **X.1 … X.12** so the
> editor can renumber once the running order is fixed. Figure and table numbers
> are local to this section.
>
> **Suggested AI-assistance disclosure, if the venue or university requires
> one:** "An AI coding assistant (Anthropic Claude) was used for code review,
> test scaffolding, figure generation and language editing in the
> simulation-environment work. All experimental design, parameter choices,
> result interpretation and conclusions are the authors'. Every quantitative
> claim in this section is traceable to a file in the project repository."
>
> Every number below is read from a file in the project repository and listed in
> `evidence/CLAIMS_EVIDENCE.csv`.

---

## X. The simulation environment

### X.1 Role of the environment in the system

The navigation system is a closed loop of four parts (Fig. 1). A simulated
warehouse aisle holds the vehicle and the obstacles; a simulated LiDAR observes
it; the perception module compresses that scan into the fixed-width feature
vector consumed by the policy (Section [Perception]); and the policy [1] emits
a horizontal velocity command that moves the vehicle, after which the world
advances and the loop repeats (Section [PPO]).

The environment is the component that makes the other three measurable. It
supplies ground truth that no on-board sensor can — the exact pose of every
object at every instant — and it is the only part of the loop that can be held
fixed while another is varied. This work therefore treats the environment as an
instrument, and asks of it the question asked of any instrument: by what
evidence do we believe its readings? Following Sargent's distinction [2], X.9
reports *verification* — evidence that the implemented model behaves as
specified — and not validation against a physical vehicle, which was outside
this work's scope.

**Figure 1.** The four-stage control loop, with the simulation environment
highlighted. Grey stages are contributed by other sections. The interface
between stages is fixed and documented: a 52-element `float32` observation in,
a two-element normalised velocity command out, once per 0.2 s control step.

### X.2 Simulation platform selection

Four candidate platforms were considered: Webots [3], Gazebo [4], AirSim [5]
and gym-pybullet-drones [6]. Selection was made against four requirements
derived from the experiment the environment had to support, rather than against
a general-purpose feature comparison (Table 1).

The requirements were: (i) a rotating range sensor available as a first-class
simulated device, so that the sensor model is part of the simulator's
maintained behaviour rather than project code; (ii) a scripting interface able
to read and write the exact pose of any object during a run, which is what makes
ground-truth comparison and deterministic episode reset possible; (iii)
bit-level repeatability of a run given the same inputs; and (iv) the ability to
run headless on commodity laptops, since the evaluation protocol executes
hundreds of episodes and no cluster was available.

Webots R2025a was selected. It provides a `Lidar` device with configurable beam
count, field of view and range cap; a `Supervisor` API that exposes object
translation and rotation fields for reading and writing; a fixed-step integrator
that reproduces a run exactly; and a headless mode with an external-controller
protocol that allowed the policy to run in a separate Python process without
modification. The decision and its alternatives are recorded in ADR-001 of the
project repository.

Two limitations of this choice are revisited in X.11: the simulated LiDAR is a
rendering-based sensor and inherits the renderer's discretisation, which
motivated the independent ray-caster comparison in X.9; and no cross-platform
replication was attempted, so platform-specific artefacts cannot be excluded.

**Table 1.** Platform selection against the four requirements.

| Requirement | Webots [3] | Gazebo [4] | AirSim [5] | gym-pybullet-drones [6] |
|---|---|---|---|---|
| Built-in rotating range sensor | yes, `Lidar` device | yes, via plugin | yes | not built in |
| Scriptable ground-truth pose read/write | yes, `Supervisor` | yes | partial | yes |
| Fixed-step, reproducible runs | yes | yes | engine-dependent | yes |
| Headless on a laptop | yes | yes | heavy (Unreal) | yes |
| Selected | **yes** | no | no | no |

### X.3 Navigation environment

The arena is a single rectangular warehouse aisle of 10.0 m × 6.0 m, bounded by
walls 2.5 m high and 0.1 m thick, with a metric floor grid for visual scale. All
geometry is expressed in a planar navigation frame whose origin is the
floor-plane corner of the arena, with *x* along the aisle and *y* across it; the
transform to the simulator's own coordinate convention is a single sign flip on
*y* and is isolated in one module, so no other code performs frame conversion.

The aisle dimensions were not chosen for realism but for comparability. The
policies under evaluation were trained in an environment of exactly these
dimensions, so altering the extent would confound any measured change with a
change in the task itself. The width in particular is load-bearing: at 6.0 m,
two 2.4 m racks placed on opposite sides leave a traversable corridor of
2.08 m at its narrowest point en route — wide enough to be flyable, and narrow
enough that the obstacle layout rather than the arena determines the difficulty.
A larger arena would have admitted trajectories that avoid the obstacles
altogether.

### X.4 Start and goal definition

The vehicle starts at (0.85, 0.0) m and the goal is at (9.15, 0.0) m, an
8.30 m separation along the aisle centreline. An episode is scored a success
when the vehicle's horizontal position enters a disc of radius 0.40 m about the
goal within the episode horizon.

Both positions lie on the centreline deliberately, so that the straight line
between them is the shortest path and also an obstructed one. In the baseline
layout the racks intrude 0.20 m across that straight line, which is the property
that makes a straight-line policy fail. The shortest collision-free path is
9.00 m against the 8.30 m straight-line distance, a tortuosity of 1.085, so
lateral deviation is unavoidable rather than merely preferable. Had the straight
line been clear, a policy ignoring its sensor entirely would have scored
highly.

### X.5 Static obstacles

Five static obstacles of three types populate the baseline layout
(Fig. 2, Table 2).
Each was included to exercise a distinct property of the navigation problem
rather than to furnish a scene, and each obstacle record in the scenario file
carries an explicit `purpose` field stating which property, so the rationale is
part of the data and not only of the prose.

Storage racks present a flat 2.4 m face: a plane returns a predictable range
profile under a rotating sensor, which makes the racks the primary calibration
target for the sensor checks in X.9, and their length is what blocks the
straight line. Structural columns are small, square and precisely located, and
serve as a geometric reference whose true position is known to the millimetre.
The crate stack is placed off the direct route to test whether clearance is
used when it is available.

All obstacle footprints are treated as **exact rotated rectangles**, not as
bounding circles. The distinction is not cosmetic. A 0.8 m × 2.4 m rack has a
bounding radius of 1.265 m, so a bounding-circle test reports contact while the
vehicle still has 0.50 m of clear air beside the rack face — an error larger
than the vehicle's own 0.18 m collision radius and larger than the margins the
experiment is trying to resolve. Clearance is therefore computed as the exact
point-to-rectangle distance, visibility as a slab–ray intersection, and contact
between consecutive control steps as a swept test along the segment travelled,
so that no displacement can pass through an obstacle unobserved. Configuration
space is formed by inflating each exact footprint by the collision radius, in
the sense of Lozano-Pérez [7]. This correction was found by verification and is
the first entry in the defect register discussed in X.9.

**Figure 2.** The obstacle catalogue, rendered from the simulator: the four
object types, their dimensions and the stated purpose of each.

**Table 2.** Obstacle catalogue for the baseline layout. Dimensions are
footprint *x* × *y* and height, in metres.

| Type | Count | Footprint | Height | Property exercised |
|---|---|---|---|---|
| Storage rack | 2 | 0.80 × 2.40 | 2.00 | blocks the straight line; flat calibration face |
| Structural column | 2 | 0.40 × 0.40 | 2.50 | precisely known geometric reference |
| Crate stack | 1 | 0.70 × 0.70 | 1.60 | clearance off the direct route |
| Transfer cart (moving) | 1 | 0.90 × 0.55 | 1.50 | time-varying free space |

### X.6 Moving-obstacle model

One obstacle moves. A powered transfer cart of footprint 0.90 m × 0.55 m and
height 1.50 m traverses the aisle perpendicular to the direction of travel,
oscillating ±1.90 m along *y* at a constant 0.45 m s⁻¹, giving a period of
16.89 s. Its pose is computed analytically as a function of simulation time and
written through the simulator's supervisor interface at every physics step; it
is therefore **kinematic**, neither accelerating nor responding to contact.

This was a deliberate choice over a physically driven cart. Because the
trajectory is an exact function of time, two episodes that differ in the policy
under test, and in nothing else, see identical obstacle motion, so the
difference between their outcomes is attributable. A physically pushed cart
would introduce a second stochastic process into every comparison. The cart also
converts the task from a purely geometric one into a timing one: free space in
its lane is a function of time, so a policy must decide when to cross and not
only where.

The cost of the choice is that contact with the cart is detected
geometrically — by the swept-footprint test of X.5 evaluated at the cart's pose
for that step — and is not physically modelled; a real collision would transfer
momentum to both bodies. Cart contacts are therefore reported as a separate
outcome category and are not interpreted as physical collision dynamics. The
analytic trajectory was verified against the simulator's own reported pose over
90 control steps, agreeing to 0.00 × 10⁰ m; as noted in X.9 this check is true
by construction and is reported as a consistency check rather than as
independent evidence.

### X.7 Sensing and timing interface

The vehicle carries a single rotating LiDAR with 720 beams over a full 360°
field of view, a beam spacing of 0.5°, and a range cap of 5.0 m. The sensor is
stabilised: it is mounted 0.14 m above the vehicle body and held level, so the
scan is a horizontal plane at 1.14 m above the floor while the vehicle cruises
at 1.00 m. Every static obstacle is taller than the scan plane (Fig. 3), so no obstacle
can be passed beneath or missed by the sensor. The 720 raw ranges are reduced to
12 angular sectors of 30°, indexed so that sectors 0, 3, 6 and 9 correspond to
front, right, rear and left; the reduction itself belongs to Section
[Perception].

Three nested clocks govern a run. The physics integrator advances in 8 ms steps.
An inner attitude-and-altitude loop runs at 125 Hz, one update per physics step.
The policy is queried every 0.2 s, which is exactly 25 physics steps, so the
control period is an integer multiple of the integrator step and no interpolation
is required at any level. An episode runs for a horizon of 90 policy steps,
equivalently 18.0 s. Before each episode the vehicle is driven to its start pose
and allowed to settle until its speed and altitude error fall below 0.02 m s⁻¹
and 0.02 m respectively, subject to a 6.0 s timeout; the consequences of this
tolerance-based criterion are examined in X.9.

**Figure 3.** Side elevation of the baseline layout, drawn from the scenario
file: the 1.00 m cruise altitude, the 1.14 m scan plane, and the height of every
obstacle. No obstacle is shorter than the scan plane, so none can be passed
beneath the sensor.

### X.8 Scenario generator and evaluation layouts

A single layout cannot distinguish navigation from memorisation of one room, a
confound documented across reinforcement-learning evaluation [8, 9] and a known
source of over-stated results [10]. We therefore replaced the hand-authored world
with a seeded generator that emits layouts as canonical JSON, hashes each file
with SHA-256, and stamps that hash into the generated simulator world, so that a
result can always be traced to the exact geometry that produced it. Seeding is
derived from a SHA-256 digest of the layout identifier rather than from the
interpreter's string hash, so generation is byte-identical across processes and
machines; this was a defect found during development and is now covered by a
cross-process regression test.

Six layouts form a graded ladder of increasing dissimilarity from the training
layout (Fig. 4, Table 3). L00 reproduces the training layout exactly. L01
perturbs obstacle positions by up to ±0.3 m and yaw by up to ±10°. L02 is a
mirror of L00 about the aisle centreline: geometrically it shares only 7.4% of
L00's occupied area by intersection-over-union, with a mean obstacle
displacement of 3.00 m, yet its shortest path length and narrowest en-route
corridor are identical to L00's, so it is a same-difficulty, different-geometry
control. L03 shifts the aisle structure and swaps sides. L04 adds crate stacks to
increase clutter. L05 combines a narrow gate with an occluded goal approach.

This is a ladder of six seeded layouts, one seed per family, and is a pilot
rather than a sample from a distribution; it is not domain randomisation in the
sense of [11], and no distributional claim is made from it.

Every generated layout must pass four validators before it is admitted. A
geometric validator enforces minimum wall margins, a minimum inter-obstacle gap
and non-overlap of rotated footprints. A feasibility validator requires that A*
[12] on the inflated occupancy grid returns a path, and records its length and
tortuosity. A corridor validator computes the widest-path (max–min) bottleneck
clearance along the route; the en-route variant, which excludes the endpoint
discs, proved necessary because the first three layouts reported identical
corridor widths that turned out to be set by the goal's distance to the wall
rather than by any obstacle. Finally a reference-traversal validator flies a
non-learned tracker along the planned path and requires that it reach the goal,
which establishes that the layout is traversable by something before any learned
policy is asked to do it.

Five of the six layouts pass all four. **L04 does not**: the reference tracker
fails to reach the goal within the horizon at every speed and planning margin
tried. Because traversability is unconfirmed there, a policy failure on L04
cannot be attributed to the policy, and L04 is reported but excluded from any
generalisation conclusion.

**Table 3.** The six evaluation layouts. IoU is intersection-over-union of
occupied area against L00; corridor is the narrowest en-route bottleneck
clearance; traversal is the reference tracker's result.

| Layout | Variation | Obstacles | IoU vs L00 | Corridor (m) | A* path (m) | Tortuosity | Reference traversal |
|---|---|---|---|---|---|---|---|
| L00 | training layout | 6 | 1.000 | 2.08 | 9.00 | 1.085 | goal, 50 steps |
| L01 | ±0.3 m, ±10° perturbation | 6 | 0.589 | 1.87 | 8.96 | 1.080 | goal, 50 steps |
| L02 | mirrored about the centreline | 6 | 0.074 | 2.08 | 9.00 | 1.085 | goal, 51 steps |
| L03 | aisle shifted, sides swapped | 6 | 0.093 | 1.31 | 9.29 | 1.120 | goal, 52 steps |
| L04 | added clutter | 9 | 0.135 | 0.85 | 9.09 | 1.095 | **not reached** |
| L05 | narrow gate, occluded goal | 10 | 0.141 | 0.68 | 9.34 | 1.125 | goal, 56 steps |

**Figure 4.** The six-layout ladder, drawn to metric scale from the generated
scenario files, with configuration space, the A* reference path, the cart's
swept lane, start and goal.

### X.9 Verification

Seven checks were applied to the environment itself, independently of any
policy (Table 4, Fig. 5).

*Timing.* The interval between successive policy observations was measured over
a full episode as 0.200000 s, with a maximum deviation below 3 × 10⁻¹⁶ s — a
floating-point artefact, confirming that the control period is exactly 25
physics steps.

*World against scenario.* Every generated world was reloaded and its object
poses compared against the scenario file that produced it. The worst
disagreement across the six layouts was 4.0 × 10⁻⁶ m. This check is run as a
blocking gate before any episode, and it is what caught the third and fourth
defects below.

*Sensor against an independent ray caster.* The simulated LiDAR was compared
against an analytic ray caster operating on the same exact box geometry, beam by
beam, giving a mean absolute difference of 0.048 m per beam over a full scan.

*Sensor against the moving cart.* The sector ranges were compared against the
ray caster evaluated at the cart's *logged* pose for the same step, sampled at
the same 720 bearings and reduced by the same sector binning, giving a mean
absolute error of 0.0193 m over 650 comparisons. An earlier version of this
check sampled the reference at different bearings and reported roughly ten times
the error; the discrepancy was a defect in the check, not in the sensor.

*Interface contract.* Twenty-six assertions on the observation vector — dtype,
shape, finiteness, range bounds, sector indexing, action clipping and horizon
length — pass for all six layouts, 26/26 each.

*Reproduction of the earlier single-layout study.* Re-running the earlier
protocol on the regenerated L00 world reproduced all 10 of 10 cells with zero
path difference, **when the session is replayed in its original order**.

*Reset independence.* The qualifier above is load-bearing and is reported as a
finding rather than buried. Because the pre-episode settle terminates on a
tolerance rather than after a fixed number of steps, a small amount of state
from the preceding episode survives into the next: the attitude-loop integrator
and, through it, the observed sector ranges. A single episode replayed in
isolation can therefore diverge from its logged trajectory by up to 0.287 m,
while the same episodes replayed in session order agree to 4.99 × 10⁻⁶ m. A
fixed-step settle removes the leakage — the integrator and sector ranges then
match to exactly zero — and the full protocol was re-run under it, moving no
pooled cell by more than two episodes in forty-five and leaving the mirror
result in X.10 intact. It was nonetheless **not adopted here**, because it still
misses a 10⁻⁵ m order-independence threshold on one of five tested targets
(2.1 × 10⁻⁴ m). All results reported were therefore measured under one reset
mode in one fixed order, which that order reproduces; the consequence for
interval estimation is stated in X.11.

Verification was not a formality: it found six defects in the environment, five
of which are fixed and each of which is now a regression test. Besides the
bounding-circle error of X.5 and the reset coupling above, these were: a camera
generator that rotated about the vertical axis instead of tilting, so every
rendered frame was empty sky — invisible in headless runs; a world writer that
placed the cart at its sweep centre rather than its *t* = 0 pose, a 1.9 m
mismatch that corrupted a calibration reference by 1.34 m; a calibration tool
that loaded a fixed scenario, so compared one world's measurements against
another world's geometry; and a protocol file in the earlier release describing
a different run than its own results table, now prevented by per-run manifests
carrying scenario hashes.

**Table 4.** Environment verification results.

| Check | Measure | Result |
|---|---|---|
| Observation interval | deviation from 0.2 s | 0.200000 s, < 3 × 10⁻¹⁶ s |
| World vs scenario pose | worst error, 6 layouts | 4.0 × 10⁻⁶ m |
| LiDAR vs ray caster | mean absolute, per beam | 0.048 m |
| LiDAR vs moving cart | mean absolute, *n* = 650 | 0.0193 m |
| Interface contract | assertions passed | 26 / 26 per layout |
| Reproduction of the earlier study | cells matched, in order | 10 / 10, 0 m |
| Same episode in isolation | worst path divergence | 0.287 m |
| Airframe tilt | episodes above 0.50 rad | 3 of 540 |

**Figure 5.** Verification panel: timing, sensor-versus-ray-caster residuals,
the cart consistency check, and the bounding-circle false-contact geometry.

### X.10 Use in evaluation

The environment was exercised by a pilot evaluation of 540 episodes spanning the
six layouts, three policy checkpoints and a reactive non-learned baseline
built on artificial potential fields [13], of
which 480 are distinct configurations once deterministic repeats are counted
once (Fig. 6). Its purpose here is to demonstrate that the environment supports
the comparison it was built for, and interpretation of the policies is left to
Section [PPO]. Two observations bear on the environment rather than on the
policy. First, L00 and L02 — equal in path length and corridor width, sharing
only 7.4% of occupied area — returned identical pooled success, 24 of 45 each,
which is the behaviour a same-difficulty control is supposed to produce and
indicates the mirror construction worked as intended. Second, L05 returned 0 of
45 on a layout whose traversability the reference tracker confirms, locating a
limit that is a property of the task rather than an artefact of an infeasible
map; the potential-field baseline meanwhile shows the known
goals-nonreachable-with-obstacles-nearby stagnation [14] on the same layout. Proportions are reported with Wilson intervals [15], which assume
independent trials; X.11 explains why that assumption is only approximately met.

**Figure 6.** Pooled success per layout with approximate 95% Wilson intervals,
the mirrored layout highlighted, and L04 shown as excluded.

### X.11 Limitations and threats to validity

Seven limitations are stated explicitly.

*The moving obstacle is kinematic.* The cart neither accelerates nor responds to
contact (X.6), so cart-contact outcomes measure geometric interception and not
collision dynamics.

*The sensor mount is ideal.* The LiDAR is held perfectly level, so the scan
plane never tilts with the airframe. Real stabilisation is imperfect, and the
bracketing experiment — repeating the protocol with a body-fixed mount — is
specified but has not been run.

*A single scan layer.* One horizontal plane at 1.14 m cannot observe overhanging
or low geometry. The layouts are built so that no obstacle is shorter than the
scan plane, which keeps the current evaluation sound but also means the
evaluation does not test that failure mode.

*Episodes are coupled.* Under the reset used for these results, consecutive
episodes are not fully independent (X.9). Reported Wilson intervals are
therefore approximate; they are retained because they remain informative about
precision, but no significance test is performed on them.

*Six layouts, one seed per family.* This is a pilot, not a sample from a
distribution. No trend is fitted across the ladder and no claim of
generalisation beyond these six layouts is made.

*A fixed horizon.* The 18.0 s horizon is a hard cut-off. On L04 the reference
tracker also fails within it, which is precisely why no conclusion is drawn
there; a longer-horizon diagnostic remains outstanding.

*Verification in simulation is not validation.* No measurement in this section
was taken on physical hardware. The checks in X.9 establish that the implemented
environment matches its specification and agrees with independent geometric
computation; they say nothing about how either corresponds to a real warehouse,
and no such claim is made [2].

### X.12 Summary of contribution

The contribution of this section is a reproducible simulation environment and
the evidence that it can be believed. Specifically: a selection of Webots
against four stated requirements, with the alternatives and the reasons
recorded; a 10 m × 6 m navigation environment whose dimensions and obstacle
placement are justified by the comparison they must support rather than by
appearance; start and goal positions placing the shortest path in obstruction,
quantified at 0.20 m; exact rotated-rectangle geometry with swept inter-step
contact testing, replacing a bounding-circle treatment that misreported contact
by 0.50 m; a kinematic moving-obstacle model whose determinism is what makes
episodes comparable, with its cost stated; a seeded, hash-stamped generator
producing byte-identical layouts across machines, admitted by four validators
including an A* feasibility test and a non-learned reference traversal; and a
seven-part verification that agrees with an independent ray caster to 0.0193 m
on the moving obstacle, reproduces the earlier study exactly when replayed in
order, and found six real defects, five now fixed and each covered by a
regression test. The one open defect — residual state across
episode resets — is reported here with its magnitude, its fix and the reason the
fix was not adopted, rather than omitted.

---

## References

[1] J. Schulman, F. Wolski, P. Dhariwal, A. Radford, and O. Klimov, "Proximal
policy optimization algorithms," 2017, arXiv:1707.06347.

[2] R. G. Sargent, "Verification and validation of simulation models," *J.
Simulation*, vol. 7, no. 1, pp. 12–24, 2013, doi: 10.1057/jos.2012.20.

[3] O. Michel, "Webots: Professional mobile robot simulation," *Int. J. Adv.
Robot. Syst.*, vol. 1, no. 1, pp. 39–42, 2004, doi: 10.5772/5618.

[4] N. Koenig and A. Howard, "Design and use paradigms for Gazebo, an
open-source multi-robot simulator," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots
Syst.*, 2004, pp. 2149–2154, doi: 10.1109/IROS.2004.1389727.

[5] S. Shah, D. Dey, C. Lovett, and A. Kapoor, "AirSim: High-fidelity visual
and physical simulation for autonomous vehicles," in *Field and Service
Robotics*, 2017, arXiv:1705.05065.

[6] J. Panerati, H. Zheng, S. Zhou, J. Xu, A. Prorok, and A. P. Schoellig,
"Learning to fly—a gym environment with PyBullet physics for reinforcement
learning of multi-agent quadcopter control," in *Proc. IEEE/RSJ Int. Conf.
Intell. Robots Syst.*, 2021, arXiv:2103.02142.

[7] T. Lozano-Pérez, "Spatial planning: A configuration space approach," *IEEE
Trans. Comput.*, vol. C-32, no. 2, pp. 108–120, 1983,
doi: 10.1109/TC.1983.1676196.

[8] K. Cobbe, O. Klimov, C. Hesse, T. Kim, and J. Schulman, "Quantifying
generalization in reinforcement learning," in *Proc. Int. Conf. Mach. Learn.*,
2019, arXiv:1812.02341.

[9] R. Kirk, A. Zhang, E. Grefenstette, and T. Rocktäschel, "A survey of
zero-shot generalisation in deep reinforcement learning," *J. Artif. Intell.
Res.*, vol. 76, 2023, doi: 10.1613/jair.1.14174.

[10] P. Henderson, R. Islam, P. Bachman, J. Pineau, D. Precup, and D. Meger,
"Deep reinforcement learning that matters," in *Proc. AAAI Conf. Artif.
Intell.*, 2018, arXiv:1709.06560.

[11] J. Tobin, R. Fong, A. Ray, J. Schneider, W. Zaremba, and P. Abbeel, "Domain
randomization for transferring deep neural networks from simulation to the real
world," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst.*, 2017,
arXiv:1703.06907.

[12] P. E. Hart, N. J. Nilsson, and B. Raphael, "A formal basis for the heuristic
determination of minimum cost paths," *IEEE Trans. Syst. Sci. Cybern.*, vol. 4,
no. 2, pp. 100–107, 1968, doi: 10.1109/TSSC.1968.300136.

[13] O. Khatib, "Real-time obstacle avoidance for manipulators and mobile
robots," *Int. J. Robot. Res.*, vol. 5, no. 1, pp. 90–98, 1986,
doi: 10.1177/027836498600500106.

[14] S. S. Ge and Y. J. Cui, "New potential functions for mobile robot path
planning," *IEEE Trans. Robot. Autom.*, vol. 16, no. 5, pp. 615–620, 2000,
doi: 10.1109/70.880813.

[15] E. B. Wilson, "Probable inference, the law of succession, and statistical
inference," *J. Amer. Statist. Assoc.*, vol. 22, no. 158, pp. 209–212, 1927,
doi: 10.1080/01621459.1927.10502953.
