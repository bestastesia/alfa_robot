# MOTION-365 B1 frozen size evaluation

The selected B1 v2b configuration was frozen before this run. Five dev size levels
were evaluated with seed 101; the independent test split remained sealed. No result
was used to retune B1.

## Strict success

| Size level | Training relation | Success | Selection p95 | Selection max |
|---|---|---:|---:|---:|
| 30 x 37 x 40 cm | seen | 58/96 (60.4%) | 397.7 ms | 428.9 ms |
| 30 x 40 x 40 cm | seen/default | 75/96 (78.1%) | 268.3 ms | 288.5 ms |
| 35 x 38 x 42 cm | held-out interpolation | 55/96 (57.3%) | 279.6 ms | 299.5 ms |
| 40 x 37 x 45 cm | held-out combination | 40/96 (41.7%) | 326.2 ms | 961.5 ms |
| 40 x 40 x 45 cm | seen | 40/96 (41.7%) | 332.1 ms | 415.2 ms |

- All sizes: 268/480 (55.8%).
- Seen sizes: 173/288 (60.1%).
- Held-out sizes: 95/192 (49.5%).
- Selection over 2,000 ms: 0/480.
- Infrastructure/OOM: 0/480.

Held-out size performance is lower than seen by 10.6 percentage points, but the two
40 cm deep / 45 cm high levels are equally difficult whether their exact dimension
combination was seen or held out. The dominant effect is geometric task feasibility,
not selection latency alone.

## Failure migration with size

### 30 x 37 x 40

The narrow width remains feasible for many central tasks, but contact and extraction
failures increase at specific columns. Failures include 26 contact collisions,
3 empty pools, 7 extraction failures, 1 approach failure and 1 timing failure.

### 35 x 38 x 42 held-out

The first held-out size has 55 successes. Failures are entirely before carry:
24 contact collisions, 6 empty pools and 11 approach failures. Every task reaching
placement succeeded, so the selected placement risk generalizes cleanly on this
subset.

### 40 cm deep / 45 cm high

Both 37 cm and 40 cm widths finish 40/96. New loaded-motion limits dominate:

- 40 x 37 x 45: 11 carry collisions, 1 envelope failure, 4 transport timeouts.
- 40 x 40 x 45: 11 carry collisions, 1 envelope failure, 9 transport timeouts,
  plus 3 placement RRT and 2 timing failures.

The single-arm side modes become infeasible at carry for these large payloads.
Single-arm top remains 0 across all sizes because its contact configurations collide.

## Mode aggregate

- Dual side: 139/180 (77.2%).
- Dual top: 93/180 (51.7%).
- Single left side: 18/30 (60.0%).
- Single right side: 18/30 (60.0%).
- Single left/right top: 0/30 each.

These mode rates mix sizes and should not be read as independent randomized trials;
they identify where future planner/candidate work belongs.

## Timing boundary

Selection remains comfortably within the 2 s contract: aggregate p50/p95 is
246.6/322.0 ms, direct-risk p95 8.17 ms. Complete planning is a different issue:
successful motion duration median rises from 57.0 s at the default size to about
67.5 s at both 40 cm deep levels, and one planning wall time reaches 37.6 s.

Therefore B1 selection latency generalizes, while complete task feasibility and
planning time do not. The next analytic work should target loaded carry feasibility
for large boxes, not add more placement weighting.

## Evidence

- Source fingerprint: `a9bfe1b8740ca9e9811c979c4fc6f814ee3f63fc196d77c9004b82c120c0e910`
- Selected config SHA256: `c166064514d8985cb635e555f6490766efa6bff59a8cb7edb3dc5f033476b65e`
- Aggregate report SHA256: `77489c9451dd1af1a262526376ef37e240883b8da1c3cee7046aaf3605492770`
- Per-instance outputs: 480 under `repro_results/b1-v2b-size-dev480/`.
- Test split opened: no.
