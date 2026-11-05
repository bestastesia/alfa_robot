# MOTION-386 B2 whole-group IK scoring prototype

## Objective

B1 selects contact, carry/recenter and placement IK sequentially. B2 changes the
ranking object to one complete group:

`g = (q_contact, q_carry, q_placement)`

The evaluated endpoint chain is:

`Home -> contact -> derived extraction -> carry -> placement -> Home`

Hard IK, pose, joint-limit, collision, extraction, placement, RRT edge and timed
trajectory validation remain unchanged. B2 only orders groups that pass the
available hard gates.

## Frozen prototype cost

`config/motion386_b2_group_smooth.json` defines equivalent-second weights:

```text
J_group = J_motion
        + 0.75 s * P_turn
        + 1.00 s * P_reverse
        + 0.50 s * P_redundancy
        + 2.00 s * P_collision
        + 1.00 s * P_limit
```

For adjacent endpoints, let

```text
T_i = max_j |q[i+1,j] - q[i,j]| / velocity_limit[j]
z[i,j] = (q[i+1,j] - q[i,j]) / (velocity_limit[j] * T_i)
```

The bounded smoothness terms are:

```text
P_turn = mean(|z[i+1,j] - z[i,j]| / 2)

P_reverse = mean(
  sign reversal(i,j) * min(|delta q[i,j]|, |delta q[i+1,j]|) / joint_span[j]
)

P_redundancy = mean(clip(
  |x_contact - 2*x_carry + x_place| / scale_x, 0, 1
))
```

`x` is Updown, left swivel or right swivel. Scales are 0.10 m and 30 degrees.
Collision uses the maximum invalid fraction over all group edges, with nine samples
per edge. The limit term reaches one at a joint limit and is zero at or above 5%
normalized margin. Actual limit violations are still invalid, not soft-scored.

The internal selector minimizes seconds. A reader-facing within-task score is:

`Score = 100 * exp(-(J - J_best) / 5 s)`

It is only comparable among groups for the same task.

## Implementation

- Core: `tools/motion306_learning/group_score.py`
- Hierarchical selector: `tools/motion306_learning/group_selector.py`
- GPU pool adapter: `tools/motion306_learning/group_backend.py`
- Config: `config/motion386_b2_group_smooth.json`
- Tests: `tests/test_core_motion386_group_score.py`
- Strict single-group smoke: `tools/experiments/motion386/b2_group_smoke.py`
- Historical diagnostic: `tools/experiments/motion386/group_smoothness_diagnostic.py`
- Diagnostic output: `repro_results/motion386-b2-selected-triple-diagnostic/`

The core accepts batches of full endpoint chains, audits every cost term, returns a
hard-valid mask and uses a stable relative score. It fails closed when redundancy
parameters or edge collision fractions are missing.

## Historical selected-triple diagnostic

Existing MOTION-386 results store the selected contact/carry/placement endpoints but
not the derived extraction endpoint or collision fractions for every group edge.
Therefore the diagnostic below includes only endpoint turn, reversal, redundancy
curvature and separately reported joint-limit robustness. It is not a B2 selection
result.

All seven arms have a selected triple on 76 common tasks:

| Function | Common final success | Smooth score P50 | Smooth cost P50 | Smoothest/tied | Mean rank |
|---|---:|---:|---:|---:|---:|
| B1 v2b | 74/76 | 98.49 | 0.832 s | 5 | 3.30 |
| Limit guard | 74/76 | 99.95 | 0.732 s | 34 | 2.80 |
| Updown stable | 74/76 | 98.68 | 0.851 s | 19 | 3.74 |
| Pool robust | 73/76 | 98.94 | 0.827 s | 12 | 3.83 |
| Home bias | 75/76 | 98.53 | 0.832 s | 5 | 4.88 |
| Return priority | 74/76 | 98.49 | 0.831 s | 5 | 5.82 |
| Balanced combo | 74/76 | 99.26 | 0.778 s | 14 | 3.63 |

Limit guard and balanced combo often produce smoother selected endpoints, but do not
improve final success on the common set. Several failed-task mean smoothness scores
are as high as successful-task scores. Smoothness is therefore useful as one group
preference, not as a feasibility predictor.

## Runtime probe

On this workstation, one synthetic vectorized batch of 131,072 six-endpoint groups
took 471.6 ms on CPU (3.598 microseconds/group). The actual candidate contract cannot
reuse one fixed placement pool: each placement target depends on its selected carry
state. The implemented hierarchical beam therefore keeps four contacts, then sixteen
contact/carry pairs, generates 64 placement candidates separately for every pair and
scores at most 1,024 complete groups. The synthetic probe is only an upper-scale
scorer measurement; it does not include IK generation, collision sampling, memory
transfer or final planning.

## Endpoint animation

`tools/experiments/motion386/group_ik_animation.py` renders the representative
all-success task `L07-R01-dev-v0-S30x40x40`. This task has the largest smoothness
cost spread among common all-success tasks. The animation compares all seven
functions, shows Updown and swivel interpolation, displays all 15 selected joint
values and outlines joints that reverse direction across the three IK endpoints.

Artifacts are under `repro_results/motion386-b2-visual/`:

- animated GIF SHA256: `df89fc2422f2507f437adb52ff0dc9fa2e8ebdc3600a1f98cd8f1829358dab29`
- poster PNG SHA256: `3bbea49fd21ac582a840bf50a0f60c0fc9f479dc0c004f4b4994c48224d7c563`
- metadata SHA256: `e62978810468b29c038ff2b474194a2667817863d1583787b0fd14b9b4d3815e`

The animation is linear endpoint interpolation only. It is not an RRT replay,
collision-validated motion or evidence that B2 has selected a new group.

The same representative sequence is also recorded with the real URDF/FK in Rerun:

- RRD: `repro_results/motion386-b2-visual/selected_ik_endpoint_motion.rrd`
- 1,260 frames on the `endpoint_motion` timeline; 180 frames per function
- RRD SHA256: `84215a3775968ed89e3b26850c4f5e08ae7dda0f33b8c008411af69cf856e081`
- Metadata SHA256: `f77f4206861557daa1542c3c3842e752bb998f7cd57e98b7438b2ecf5874be55`

The Rerun recording has the same endpoint-interpolation limitation. Persisted strict
evaluation rows contain the full-trajectory hash but not the trajectory frames, so
this recording must not be presented as recovered RRT motion.

## Next integration step

1. Hard-screen the contact pool and retain the best four contact/extraction endpoints.
2. Generate each contact's own 512-row carry pool and retain the best sixteen pairs globally.
3. Generate each retained pair's own 64-row placement pool, because placement targets depend on carry.
4. Score at most 1,024 complete groups with all five edge risks.
5. Execute the highest-ranked valid group once; do not fall back through groups.
6. Compare against frozen v2b on strict success, selection p95, planning wall time,
   executed T_lb, direction reversals, acceleration and jerk.

Current selected configuration remains B1 v2b. B2 is a tested scoring prototype,
not a promoted selector and not an independent-test result.

## Rebuilt runtime and real GPU smoke

A composed isolated entry point was created at
`/home/tim/.venvs/motion386-curobo-py311`. It references the existing light project
environment, Isaac Sim 5.1 Python 3.11 prebundle and frozen cuRobo source
`78fd485`. The actual local versions are Python 3.11.17, Torch 2.7.0+cu128,
CUDA 12.8, Warp 1.17.0, NumPy 1.26.4 and SciPy 1.11.4. The Torch version differs
from the old README's 2.8 claim and is recorded rather than silently upgraded.

The cuRobo connector GPU tests pass 3/3. The older fixed-pair core smoke currently
rejects its initial state, so it is not used as B2 evidence. The current mobile
`LookaheadEnv` B2 path is operational.

The first all-contact hard-screen implementation took 3,776 ms. Lazy contact
validation ranks cheap analytic extraction endpoints first and stops after four
hard-valid contacts; the same selected group then takes about 1,260 ms. A persistent
checker pollution bug found by strict execution was fixed by restoring both the
snapshot and empty/loaded checker geometry after conveyor previews.

## Four-mode paired smoke

Four same-environment, same-task, same-seed v2b/B2 pairs all succeed. B2 selection
stays below 2 seconds in all four modes:

| Mode | B2 selection | Executed T_lb delta | Duration delta | Jerk delta |
|---|---:|---:|---:|---:|
| dual-top | 1,260 ms | +1.175 s | +1.707 s | -0.00537 |
| dual-side | 1,576 ms | -0.491 s | -1.898 s | +0.00465 |
| single-left-side | 936 ms | -1.039 s | -2.225 s | +0.00009 |
| single-right-side | 931 ms | -1.611 s | -3.316 s | +0.00005 |

Median B2-minus-v2b is +863 ms selection, -420 ms planning, +291 ms combined
wall time, -0.765 s executed T_lb and -2.061 s motion duration. B2 improves executed
T_lb and duration in 3/4 tasks, but improves timed jerk in only 1/4. Normalized
joint variation improves in 1/4 and direction reversals improve in 2/4. The current
endpoint smoothness terms therefore do not consistently predict actual planned-path
smoothness.

The preregistered `near_optimal_smooth` smoke ablation first limits primary
motion/collision/limit cost to +0.5 s and then minimizes endpoint smoothness. It is
rejected: normalized joint variation increases in 4/4 and timed jerk still improves
in only 1/4. Do not expand either B2 arm to dev96 yet. Next work should replace or
demote endpoint smoothness and retain the useful hierarchical group feasibility/time
selection.

Evidence:

- Weighted four-pair report: `repro_results/motion386-b2-paired-smoke/report.json`
- Near-optimal rejection: `repro_results/motion386-b2-near-paired-smoke/report.json`
- Paired actual-trajectory Rerun: `repro_results/motion386-b2-smoke/L07-R01-dev-v0-v2b-vs-B2.rrd`
- Weighted report SHA256: `9df9ab2031f8d909a52ac694e98b02e0a7705daa6feaad6b712cdc87dd507c1f`
- Near-optimal report SHA256: `04c0f9109ceb6f6ef1dcdaaa13a86dd903f151433b54c7654c054114580fcf73`
- Paired RRD SHA256: `14a248d78220f85b5c20abb51fcbf23866a1ad6268f57ab1d87e6f0a31314de8`

## B2-time beam8 default-size dev96

Endpoint smoothness weights were set to zero after the four-mode evidence above.
Pair-beam 16, 12 and 8 selected identical groups on all four smoke tasks; beam8
reduced selection to 0.69-1.04 s and retained 4/4 strict success. Beam8 was frozen
before the full default-size dev96 run.

The current-source, same-model, same-registry, seed-101 strict comparison is:

| Metric | v2b | B2-time beam8 |
|---|---:|---:|
| Strict success | 75/96 | 76/96 |
| Selection P50 / P95 | 241 / 267 ms | 926 / 1059 ms |
| Selection over 2 s | 0 | 0 |
| Successful planning P50 | 3249 ms | 3158 ms |
| Successful selection + planning P50 | 3492 ms | 4136 ms |
| Executed T_lb P50 | 28.51 s | 28.64 s |
| Motion duration P50 | 57.03 s | 57.50 s |
| Timed jerk ratio P50 | 0.03127 | 0.03152 |

Paired outcomes are B2-only 3, v2b-only 2, both-success 73 and neither 18. The
exact two-sided discordant-pair p-value is 1.0. Among both-success tasks, median
B2-minus-v2b executed T_lb, duration and jerk are all zero; total selection plus
planning wall time increases by 691 ms. The only net mode gain is dual-side, 36/36
versus 35/36. This is a specific-scene development improvement with five outcome
swaps, not stable overall improvement.

Decision: keep v2b selected, do not open independent test, and do not continue
weight tuning on this dev set. Any future smoothness work should be a separate
planned-path-proxy task rather than another endpoint-cost sweep.

Evidence:

- Report: `repro_results/motion386-b2-time-beam8-dev96-paired/report.json`
- Per-task pairs: `repro_results/motion386-b2-time-beam8-dev96-paired/pairs.csv`
- Report SHA256: `024672799981959322fe9071e44175cf5db8d85c4ea94847d879383234a8edcf`
- Pairs SHA256: `cf176d84e9e9f717cc55d09800f09cea274af1fc99584ce6fc7bb23bbb86e137`

## Integrity

- Weighted config SHA256: `52cc04eb1bf8b03a7774e4fdac6be68ed28f4f86a2e9a7ea4bdd30c093bdd5df`
- Near-optimal config SHA256: `0e0bcf1242ca288dd4df1fe01b62462b81a70d62e4dd123e8a8d139a8edaa904`
- Diagnostic report SHA256: `5657953422ddfb3ce78f4047c57a5f99d3d322ef2b0bc607db6bddb13c0595f8`
- Diagnostic summary SHA256: `63d2f7d00413ca69634f36a7c14cbda690137e979ae3cbd8e4bc3b44566c7cd4`
- Per-task selected triples SHA256: `3d4228b8bdf34eeca83f0efca7bb5b813a72751fc6c02f3c4d512ff29b75e0ab`
