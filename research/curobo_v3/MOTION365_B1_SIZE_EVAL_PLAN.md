# MOTION-365 B1 frozen size evaluation plan

Frozen after selecting B1 v2b and before running non-default size results.

- Selector: `config/motion365_b1_v2b_place.json`, unchanged.
- Registry: `config/motion306_registry.json`.
- Group: dev only, seed 101.
- Levels: all five size levels; 96 instances each, 480 total.
- Seen sizes: S30x40x40, S40x40x45, S30x37x40.
- Held-out sizes: S35x38x42, S40x37x45.
- Test split remains sealed.
- Exact collision, extraction, RRT, timing and target contracts remain unchanged.
- All empty pools, task failures, infrastructure errors and OOMs stay in the denominator.

Report strict success, failure stage/type, selection p50/p95/max and 2 s violations
per level, seen/held-out aggregate and mode. These results must not be used to
retune v2b while retaining the held-out-size claim.
