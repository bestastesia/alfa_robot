# MOTION-365 B1 v3 loaded-carry risk report

This experiment followed `MOTION365_B1_V3_PREREGISTRATION.md`. The v3 config added
only a 9-sample loaded direct-edge collision fraction at the carry stage, scaled by
the inherited 2 s transport budget. It remained a soft cost; all hard checks were
unchanged.

## Paired result

| Size | v2b | v3 | v2b-only | v3-only | Transport timeout v2b→v3 |
|---|---:|---:|---:|---:|---:|
| 30×40×40 cm | 75/96 | 74/96 | 2 | 1 | 0→0 |
| 40×40×45 cm | 40/96 | 43/96 | 1 | 4 | 9→9 |
| Combined | 115/192 | 117/192 | 3 | 5 | 9→9 |

Selection p95 increased from 320.7 ms to 328.3 ms; maximum remained below 422 ms.
No selection exceeded 2 s and no infrastructure error occurred.

For 112 tasks where both arms succeeded, median v3-minus-v2b is zero for raw T_lb,
executed T_lb and motion duration. Mean executed T_lb changes by -0.021 s, which is
not a material path-cost improvement.

## Outcome changes

v3 recovered five tasks: two former timing failures and three former placement-RRT
failures. It regressed three formerly successful tasks into placement-RRT failures.

Critically, none of the nine targeted `transport_budget_exhausted` tasks recovered.
The direct collision fraction changes the carry endpoint and can incidentally change
the downstream placement problem, but it does not measure the RRT connectivity that
caused the target failure.

## Decision

Reject v3 as the selected B1 configuration because:

1. default-size strict success regresses from 75/96 to 74/96;
2. the targeted large-size transport timeouts remain 9/96;
3. its net large-size gain comes from unrelated placement/timing changes;
4. it adds latency without solving its preregistered problem.

Selected B1 remains v2b (`config/motion365_b1_v2b_place.json`). The v3 implementation
and negative result remain available for audit; weights were not retuned after seeing
the result.

## Evidence

- Source fingerprint: `2dc00d09ede1262f20c0b0577e3f280b39743bbb4740855478eb704a7dbfd978`
- v3 config SHA256: `4b3b8ee8d493b26d4f30a3a6be674877e060d982913d8c17da0742477dc642aa`
- v2b result SHA256: `159fcfec74c2cfb1c38dd1a4c6dd593281b95844e631dda434872b56dab7cc00`
- v3 result SHA256: `e165c82e60b79584c2fc6f18ce5c4e21a148febfdbb45cc493ad215043e21a31`
- paired report SHA256: `b0930363ab1d497bebdc09b96f5b26c740d2f07247eccb5e4a1cde01c0a28be3`
