# MOTION-365 B1 v2 dev ablation report

The experiment followed `MOTION365_B1_V2_PREREGISTRATION.md`. No weight was changed
after results. Default-size dev96 and seed 101 were used; test and held-out sizes
remained sealed.

## Results

| Arm | Added risk | Strict success | Selection p50 | Selection p95 | Over 2 s |
|---|---|---:|---:|---:|---:|
| B0 | local T_lb | 75/96 | 237.4 ms | 336.4 ms | 0 |
| B1 v1 | nearest carry lookahead | 73/96 | 231.9 ms | 260.0 ms | 0 |
| B1 v2a | approach direct-edge | 72/96 | 234.7 ms | 268.8 ms | 0 |
| B1 v2b | placement + Home direct-edge | **75/96** | 241.4 ms | 266.5 ms | 0 |
| B1 v2 | v2a + v2b | 74/96 | 240.2 ms | 270.2 ms | 0 |

Only v2b met the preregistered advancement rule.

## Paired interpretation

Compared with B1 v1, v2b recovered two instances and regressed none:

- `L03-R01-dev-v2-S30x40x40`: conveyor placement RRT failure -> success.
- `L03-R02-dev-v2-S30x40x40`: conveyor placement RRT failure -> success.

Compared with B0, v2b has the same 75/96 total but a different success set:

- v2b-only success: `L02-R01-v0` and `L03-R02-v2`.
- B0-only success: `L02-R01-v1` and `L07-R01-v1`.
- Both success: 73; neither success: 19.

For the 73 both-success tasks, B0 minus v2b executed T_lb has median 0 and mean
+0.134 s. Thus v2b is not a broad path-cost improvement, but it preserves B0's
strict success total while retaining B1's faster candidate screening.

## Why v2a was rejected

The 9-sample Home-to-contact straight-line collision fraction did not recover any
of the four approach budget/validation failures. It changed one previously
successful choice (`L03-R07-v1`) into a timing-validation failure. v2a finished
72/96; combined v2 inherited that regression and finished 74/96.

This proxy is not a reliable representation of the non-straight approach planner.
It remains implemented as a zero-by-default optional ablation, not part of the
selected B1 configuration.

## Cost and latency

v2b adds placement and return-Home risk only. The direct-risk calculation p95 was
8.15 ms across recorded stages. Total selection p95 was 266.5 ms and maximum
287.1 ms, with no 2 s violation.

Successful-task medians for v2b:

- raw planned-path T_lb: 29.55 s;
- executed T_lb: 28.51 s;
- motion duration: 57.03 s.

Hard IK/Pose, joint limits, exact collision, extraction, RRT dense validation and
timing rules were unchanged. A positive direct-edge risk never rejected a candidate.

## Decision and evidence

Selected config: `config/motion365_b1_v2b_place.json`.

- Config SHA256: `c166064514d8985cb635e555f6490766efa6bff59a8cb7edb3dc5f033476b65e`
- Source fingerprint: `a9bfe1b8740ca9e9811c979c4fc6f814ee3f63fc196d77c9004b82c120c0e910`
- v2b result SHA256: `a9a7b463e2c11991414dd69c61194077e9bd3d59c129b81581421e1b4d0dba5e`
- Unified ablation SHA256: `7a149f559fb9ccdc9da48aa57ff6456c794707c45ea5c778bc74bb3a5ed69f75`

The next evaluation step is size generalization with this config frozen. Results
must not be used to retune v2b and still be called held-out size evaluation.
