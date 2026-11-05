# MOTION-365 analytic B1 selector

B1 is the explainable, non-neural comparison arm for the strict three-IK contract.
It ranks the current contact, carry and placement IK pools online, applies the same
deterministic gates as execution, then executes exactly one candidate per stage.
There is no retry in the B1 result; operational fallback must be measured separately.

## Cost contract

The fact source is `config/motion365_b1_v1.json`. All non-dimensionless cost terms
are seconds based on the joint-space lower bound `max_j(abs(dq_j) / v_j)`.

- Contact: Home-to-contact + contact-to-extraction + extraction-to-nearest legal
  carry candidate.
- Carry: current extraction endpoint to carry candidate.
- Placement: carry-to-placement + placement-to-Home.
- Optional carry-pool spread/scarcity, updown and joint-limit-margin soft terms are
  implemented but have zero v1 weight. They need a dev-only ablation before use.

IK/Pose, joint limits, collision, complete extraction, single-arm envelope, parking
collision, RRT dense validation and timing validation remain hard constraints. B1
does not create a collision-clearance surrogate and does not change MOTION-303
rotation handling.

Candidate generation, feature preparation, online lookahead, ranking and screening
all count toward the 2,000 ms selection budget. Planning wall time is reported
separately because the inherited full planner already exceeds two seconds in some
tasks.

## Run a dev subset

```bash
cd /home/tim/alfa_robot-motion365-b1-repro/research/curobo_v3
export PYTHONPATH=/home/tim/alfa_robot-motion365-b1-repro/research/curobo_v3/tools:/home/tim/Applications/isaacsim-5.1.0/exts/omni.isaac.ml_archive/pip_prebundle:/home/tim/Applications/isaacsim-5.1.0/kit/python/lib/python3.11/site-packages:/home/tim/alfa_robot-alfa_v3-curobo-repro/third_party/curobo-main-78fd485
flock -w 120 /tmp/sevenova-curobo-gpu.lock env CUDA_VISIBLE_DEVICES=0 OPENBLAS_NUM_THREADS=4 PYTHONPATH="$PYTHONPATH" /home/tim/alfa_robot-motion365-b1-repro/.venv/bin/python tools/motion306_learning/evaluate.py --registry config/motion306_registry.json --output repro_results/b1-dev --route b1 --b1-config config/motion365_b1_v1.json --group dev --levels S30x40x40 --seed 101 --max-instances 1 --max-minutes 5 --threads 4 --contact-pool capped
```

`--route b1` always enables exact screening, the placement parking gate and the
lookahead environment. The held-out `test` split remains sealed unless explicitly
enabled after the acceptance rule is frozen.

## B1 v1 dev result on this machine

Default 30 x 40 x 40 cm, 96 dev instances, seed 101, capped candidate pools and
the same exact gates for both arms:

| Metric | Local T_lb B0 | Online-lookahead B1 v1 |
|---|---:|---:|
| Strict success | 75/96 (78.1%) | 73/96 (76.0%) |
| Reached placement | 78 | 77 |
| Placement-stage failure | 3 | 4 |
| Selection p50 | 238.2 ms | 232.1 ms |
| Selection p95 | 337.8 ms | 260.2 ms |
| Selection over 2 s | n/a | 0 |
| Successful executed T_lb median | 28.70 s | 28.34 s |

Paired outcomes: B1-only success 1, B0-only success 3, both success 72. Among
both-success tasks, median path-cost difference is zero; B0 minus B1 mean executed
T_lb is +0.135 s. B1 makes exact screening cheaper by ranking legal contact/carry
candidates earlier, but the nearest-carry lookahead does not predict approach or
placement planning feasibility and loses two net strict successes. This is a
retained non-improvement result; v1 weights were not retuned after seeing dev.

Final source-matched report: `repro_results/b1-v1-dev96-final/report.json`.

## Selected v2 continuation

The preregistered direct-edge ablation is complete. Placement + return-Home soft
risk (`config/motion365_b1_v2b_place.json`) recovered two v1 placement-RRT failures
without a strict-success regression, finishing 75/96 and matching local B0's total.
Approach direct-edge risk was rejected because it recovered no approach failure and
introduced one timing regression.

See `MOTION365_B1_V2_REPORT.md` and `config/motion365_b1_selection.json`. v2b is the
frozen analytic configuration for subsequent size evaluation; test remains sealed.

The frozen five-size dev evaluation is complete; see
`MOTION365_B1_SIZE_REPORT.md`. Selection stayed below 2 s for all 480 instances,
while strict success fell from 78.1% at the default size to 41.7% at both 40 cm
deep levels because contact, carry and transport feasibility degraded.

The preregistered v3 carry-risk ablation is also complete and rejected; see
`MOTION365_B1_V3_REPORT.md`. It improved the large-size total from 40 to 43 but did
not recover any of the nine targeted transport timeouts and regressed default-size
success from 75 to 74. The selected configuration remains v2b.

MOTION-386 then compared six further custom functions. `home_bias` (0.25 times the
carry candidate's Home lower bound) was the only default-size arm to improve strict
success, 76/96 versus 75/96 with no lost successes. Across five sizes it finishes
269/480 versus v2b's 268/480, but swaps 11 outcomes. It remains a provisional
candidate rather than replacing v2b; see `MOTION386_COST_MATRIX_REPORT.md`.
