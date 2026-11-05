# MOTION-365 B1 v3 loaded-carry risk preregistration

Frozen before running v3 results.

## Hypothesis

At 40 cm depth / 45 cm height, B1 v2b exposes loaded `carry_collision` and
`transport_budget_exhausted` failures. Exact endpoint screening already rejects
colliding carry candidates, but it cannot distinguish the remaining endpoints by
how much of the direct loaded edge is blocked. A lower collision fraction may be a
useful soft proxy for RRT connectivity.

## Fixed change

Starting from selected v2b:

- sample 9 states from the extraction endpoint to every carry candidate;
- evaluate with the existing loaded validity checker;
- add `2.0 s * colliding_fraction` to the carry-stage cost;
- never reject a candidate from this proxy;
- keep contact and placement costs, hard gates, seeds and budgets unchanged.

The 2 s scale equals the inherited transport search budget. No weight/sample sweep
is allowed after results.

## Development comparison

- Default S30x40x40 dev96: non-regression check.
- Seen large S40x40x45 dev96: target subset.
- Seed 101, capped pools, strict one action per stage.
- Compare current-source v2b and v3 with the same registry and hard gates.
- Independent test remains sealed.

Advance v3 only if:

1. default strict success is at least v2b's result under the same source;
2. large-size strict success increases, or transport timeouts decrease without a
   net strict-success loss;
3. selection p95 remains below 2 s.

Previously observed held-out-size results motivated the engineering question, so
any later held-out recheck is a transfer probe, not a pristine held-out claim.
