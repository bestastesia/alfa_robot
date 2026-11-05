# MOTION-365 B1 v2 dev ablation preregistration

Frozen before running v2 results on 2026-10-28.

## Question

Can a cheap direct-edge collision-risk proxy recover strict single-shot failures
that B1 v1's joint-space lower bound cannot predict, without changing hard gates or
exceeding the 2 s selection budget?

## Risk definition

For every current candidate, sample 9 evenly spaced joint states from the current
state (exclusive) to the candidate (inclusive). Risk is the fraction rejected by
the existing state-validity checker:

- Stage 0: Home/current to contact with the empty-scene checker.
- Stage 1: extraction endpoint to carry with the loaded checker. Implemented and
  audited, but zero-weight in all three preregistered v2 arms because v1 had no
  stage-1 planning failures.
- Stage 2: carry to placement with the loaded checker at the real conveyor parking
  snapshot; placement to Home with the empty checker at that same snapshot.

Risk is a soft cost only. A nonzero fraction never rejects a candidate, because RRT
may find a non-straight path. Endpoint, collision, RRT dense and timing checks remain
unchanged hard acceptance rules.

## Arms and fixed weights

- v2a: approach risk x 2.0 s.
- v2b: placement risk x 2.0 s plus Home risk x 2.0 s.
- v2: v2a + v2b.

The 2.0 s scale equals one inherited stage search budget. No post-result weight
sweep is allowed in this experiment.

## Dataset and metrics

- Registry and candidate pools identical to B1 v1.
- Default S30x40x40, dev96, seed 101.
- Held-out size levels and test split remain sealed.
- Main: strict success and paired B0/B1-v1 differences.
- Secondary: selection p50/p95/max, budget violations, failure stage/type,
  executed T_lb and motion duration among paired successes.
- Every failure, empty pool, infrastructure error and OOM remains in the denominator.

## Decision rule

Advance a v2 arm only if strict success is at least B0's 75/96 and B1-v1's selection
p95 remains below 2 s. If no arm meets that rule, retain B0 as the analytic baseline
and record direct-edge risk as not effective under this design.
