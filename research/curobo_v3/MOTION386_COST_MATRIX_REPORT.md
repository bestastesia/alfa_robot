# MOTION-386 custom cost matrix comparison

Six additional explainable cost functions were frozen before results and evaluated
on the same default-size dev96, seed 101, candidate pools and hard constraints. This
is an exploratory development comparison; independent test remains sealed.

## Default-size result

| Function | Added preference | Success | Selection p95 | Executed T_lb median |
|---|---|---:|---:|---:|
| v2b | selected baseline | 75/96 | 263.3 ms | 28.51 s |
| limit_guard | 5% joint-limit margin, max 2 s | 75/96 | 269.0 ms | 29.98 s |
| updown_stable | 0.1 m lift = 1 s | 74/96 | 266.8 ms | 28.89 s |
| pool_robust | downstream mean/density | 74/96 | 273.9 ms | 28.43 s |
| **home_bias** | **0.25 × carry-to-Home T_lb** | **76/96** | **268.1 ms** | **28.55 s** |
| return_priority | 1.5 × place-to-Home | 75/96 | 270.1 ms | 28.51 s |
| balanced_combo | half-strength combination | 75/96 | 262.6 ms | 28.92 s |

No arm exceeded the 2 s selection budget.

## What each function taught us

- `limit_guard` changes failure type but gives no net success and raises executed
  T_lb median by 1.47 s. A generic margin penalty is too blunt.
- `updown_stable` loses one strict success and creates a transport timeout. Less lift
  motion is not automatically better downstream geometry.
- `pool_robust` loses one strict success. Several nearby carry options do not imply
  that the eventually selected carry endpoint is good for placement.
- `return_priority` is effectively redundant with v2b's explicit Home lower bound
  and Home collision risk.
- `balanced_combo` has the fastest p95 among these arms but raises paired executed
  T_lb median by 0.57 s and swaps two successes each way. Reasonable soft terms are
  not additive by default.
- `home_bias` is the only arm with a clean default-size gain: one additional success,
  no lost successes, and paired executed T_lb median difference zero.

## Home-bias size probe

| Size | v2b | home_bias | Net |
|---|---:|---:|---:|
| 30×37×40 | 58 | 57 | -1 |
| 30×40×40 | 75 | 76 | +1 |
| 35×38×42 | 55 | 54 | -1 |
| 40×37×45 | 40 | 41 | +1 |
| 40×40×45 | 40 | 41 | +1 |
| **All five** | **268** | **269** | **+1** |

Across five sizes, home-bias recovers six tasks and regresses five. Both-success
executed T_lb median difference is zero; mean increases by 0.173 s. Selection p95
decreases from 321.3 to 317.0 ms in the exploratory aggregate.

The three non-default size runs were compared with the earlier frozen v2b size run;
their source fingerprints differ because unrelated verification/report utilities
were added. Selector runtime logic and configuration contracts did not change, but
this is therefore an exploratory size comparison rather than the same-source proof
used for default and 40×40×45 cm.

## Decision

Keep v2b as the selected robust baseline. Mark home-bias as the best development
candidate, not the new final configuration:

- default and 40×40×45 same-source comparisons each net +1;
- five-size aggregate nets only +1/480 with 11 outcome swaps;
- the evidence is too unstable to claim broad improvement after testing six dev
  hypotheses.

Before promotion, home-bias needs a preregistered interleaved repeat or independent
test decision. No further weight tuning should use the same dev outcomes.

## Evidence

- Matrix definition: `config/motion386_cost_matrix.json`
- Three-IK plus final table: `MOTION386_THREE_IK_TABLE.md`
- Matrix report SHA256: `c49db216c1ba34ad8b72f45db4961bda5d16141db813f6e5c848f1d2b0d7bc94`
- Home-bias config SHA256: `3e8470ce17b2a8948c96f1011dd00a30efdffec055ac42509e322e89548f1a69`
- Large-size paired report SHA256: `fb75074534fe853d428dc0aa8cbf58b3e66cbe649fc9d76604c4b30991b355d7`
- Five-size exploratory report SHA256: `577c87cb91180ce70d171b08887bf6e32f0e8991a7b8b31895c36acf1a05ccf2`
