# MOTION-365 handoff analysis

## Purpose

MOTION-313 is the parent direction: select IK configurations with an explainable
custom cost instead of a neural network. MOTION-365 is the concrete B1 handoff:
under a strict single-shot contract, select three coupled IK configurations in no
more than two seconds and compare fairly with the MOTION-306 learning selector.

The three decisions are:

1. Contact IK, together with its 35 cm (or size-adjusted) extraction endpoint.
2. Carry/recenter IK after extraction.
3. Placement IK, which must also leave a valid path back Home.

The selector does not replace IK, collision checking, RRT, timing, or final
trajectory validation. A selected triple is successful only if the complete task
passes those existing checks.

## Existing baselines

### A: current planner policy

- Contact: rank near Home, then lazily try 8/16/32/64 batches under a shared budget.
- Carry: alternate distance/rotation preferences and search multiple goals.
- Placement: distance plus rotation ordering, then planner retries and round fallback.
- This has high operational success because it can try several candidates. It is not
  a strict single-choice baseline.

### B0: analytic T_lb with a frozen V3 table

`T_lb` is a joint-space time lower bound in seconds:

`max_j(abs(q_next[j] - q_prev[j]) / velocity_limit[j])` per sequential segment,
summed across the task.

MOTION-311 B0 used a precomputed downstream V3 value table. It improved path-time
cost versus A, but the table is tied to an old carry candidate pool and box geometry.
The handoff reports that only 1.3% of the new dual-side carry candidates match and
the median nearest-neighbor distance is 1.39 rad. It cannot be reused as a general
B1 feasibility predictor.

### MOTION-311 learning prototype

The v2 prototype trained three small models in reverse dependency order:

- N3 predicts placement plus Home feasibility/cost.
- N2 predicts carry connectivity.
- N1 predicts contact/extraction success and a downstream value.

It keeps all hard planning checks and falls back to the baseline. On 96 dev
instances the committed delivery reports:

- 80/93 strict first-triple successes after excluding three fixed no-contact scenes.
- 91/93 success with retries/fallback.
- Selection p50/p95 16.28/21.34 ms.
- Versus A, median path T_lb improved 7.09%.
- Versus analytic B0, T_lb was 0.13% worse and no attributable network gain was
  demonstrated.
- N1 remained seed-sensitive; held-out test was not opened.

The later four-route experiment changed to a strict per-stage action contract. The
handoff reports dev results R3 60.1%, R1 43.8%, R2 33.3%, R4 14.6%, and local
heuristic 6.2%. These runs were budget-capped and not converged. Their original
models and raw result tree are not committed in Git, so this machine cannot yet
reproduce those percentages.

## What MOTION-306 / MOTION-364 has implemented

The checked-out branch extends the strict environment in four commits:

1. Box-size parameterization: five size levels, size-dependent wall pitch,
   extraction distance and payload sphere model. The registry contains 4,055
   instances: 1,280 train, 480 dev, 510 test plus seen-test, reserve and probes.
   Two size levels are held out from training.
2. Exact selector-side screening: contact collision and extraction checks, batched
   carry collision gates, and an optional placement parking collision gate. These
   are the same deterministic checks used by execution, not learned substitutes.
3. Online lookahead features:
   - Contact: Home-to-contact T_lb, extraction T_lb, whether the extraction endpoint
     has an analytic solution, nearest/mean downstream carry T_lb and local carry
     density.
   - Carry: transport T_lb, gate pass flag, Home distance and updown change.
   - Placement: placement T_lb, return-Home T_lb and their sum.
4. A residual candidate-conditioned Double DQN with a feasibility head. It can keep
   the analytic cost as a fixed prior and use the network only as a veto: candidates
   whose predicted success probability is sufficiently worse are moved behind the
   analytic ordering. The latest commit can train only this success head and exclude
   scene groups known to be entirely infeasible.

Committed dev diagnostics before the latest training step report:

- Frozen R3 without screening: 57/96 strict successes.
- R3 plus exact screening: 70/96.
- Analytic heuristic plus screening: 78/96.
- Random plus screening: 68/96.

This establishes that deterministic screening is already a major contributor. It
does not yet prove that the new learned feasibility head improves on the screened
analytic heuristic.

## B1 principle suggested by the handoff

B1 should compute downstream cost online from the current candidate pools and
geometry instead of using the frozen V3 table:

1. Preserve exact hard constraints: IK residual, limits, target pose and collision.
2. Generate current contact, carry and placement candidate pools.
3. Compute analytic extraction endpoints and size-aware task geometry.
4. Apply the same deterministic selector-side gates used by the strict environment.
5. Rank surviving candidates by online downstream `T_lb`; use clearance, joint
   margin or rotation only as explicit normalized tie-breakers/soft terms.
6. Execute exactly one candidate per stage and report first-triple success separately
   from any operational fallback.

The original `lookahead_heuristic+screen` route was the B1 seed. This checkout now
contains a named `b1` route, a versioned seconds-based configuration, exact hard
screening, per-candidate cost audit and separate selection/planning timing. On the
default-size dev96 comparison it achieved 73/96 strict success versus 75/96 for
local T_lb B0. Its selection p95 improved from 337.8 ms to 260.2 ms, but it lost two
net successes. The current contact lookahead reaches only the nearest gated carry
goal; it does not predict approach or placement planning feasibility.

The preregistered v2 ablation subsequently selected placement + return-Home
direct-edge soft risk. It restored two v1 placement failures without regression,
matching B0 at 75/96 while keeping selection p95 at 266.5 ms. Frozen five-size dev
evaluation then achieved 268/480 overall and 95/192 on held-out sizes; all selection
calls stayed below 2 s. Large-box failures shift toward carry collision and transport
timeouts, outside the selected placement refinement's scope.

## Remaining work for MOTION-365

- Freeze the two-second definition: selection only, or selection plus one full
  planning/validation call. The existing planner p90 of 2.5 s cannot satisfy a
  two-second end-to-end p90 without separate planner work.
- Freeze candidate pools, information available at each stage, and whether B1 may
  enumerate all analytic triples before selecting.
- Define exact hard rejection versus soft/risk ordering. An approximate collision
  test must not silently become a stricter safety gate unless its soundness is shown.
- The placement feasibility-risk ablation is complete and selected. A future
  approach proxy must model the non-straight planner; the tested straight-edge risk
  was rejected.
- Run A, old B0, B1 and MOTION-306 with identical model, registry, seed and budgets.
- Report strict success, failure stage/type, selection/planning/total latency,
  T_lb_path, J_arc and motion duration; retain all failures.
- Continue optional margin/rotation soft terms only when a concrete failure
  hypothesis exists. Do not open the sealed test split until the acceptance rule is
  approved.
