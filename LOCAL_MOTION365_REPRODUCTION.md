# MOTION-365 / MOTION-313 local reproduction

Date: 2026-10-28

This checkout is an isolated handoff reproduction. It does not modify the user's
existing `alfa_v3_curobo` working tree.

## Source

- Worktree: `/home/tim/alfa_robot-motion365-b1-repro`
- Commit: `f7edabb8b06f9a05d703a0e89defeb546da68ccd`
- Source branch: `bestastesia/alfa_robot:feature/motion306-lookahead-ranker`
- Baseline inherited by MOTION-311: PR #65 head `a77abd525ace67edd4c397599aba661fcaa97b37`
- MOTION-337 frozen validation record: `research/curobo_v3/evidence/motion311_delivery_validation.json`

The checkout is detached intentionally. The original dirty working tree remains at
`/home/tim/alfa_robot-alfa_v3-curobo-repro`.

## Environment

- Python 3.11.17: local `.venv`
- PyTorch 2.7.0+cu128: reused from Isaac Sim 5.1
- CUDA driver 580.178.04, RTX 4070 12 GB
- Warp 1.17.0: reused from Isaac Sim 5.1
- cuRobo source commit `78fd485`, package version reports 0.7.8
- NumPy 1.26.4, SciPy 1.11.4, trimesh 4.10.1, yourdfpy 0.0.60
- cuda-core 1.2.2 with CUDA 12.9 runtime packages

The handoff machine used Python 3.11, PyTorch 2.9.1 and cuRobo 0.8.0. This machine
therefore reproduces source contracts and representative GPU behavior, but its
timings are not an exact environment-matched performance reproduction.

`prepare_checkout.py` was run successfully. It relocates generated URDF/YAML paths
to this checkout and builds:

`research/curobo_v3/generated/v3_analytic_071cb95/libv3_analytic_bridge.so`

## Environment variables

Run these commands in a new terminal before project commands:

```bash
cd /home/tim/alfa_robot-motion365-b1-repro/research/curobo_v3
export PYTHONPATH=/home/tim/alfa_robot-motion365-b1-repro/research/curobo_v3/tools:/home/tim/Applications/isaacsim-5.1.0/exts/omni.isaac.ml_archive/pip_prebundle:/home/tim/Applications/isaacsim-5.1.0/kit/python/lib/python3.11/site-packages:/home/tim/alfa_robot-alfa_v3-curobo-repro/third_party/curobo-main-78fd485
```

## Verified commands

CPU core suite:

```bash
cd /home/tim/alfa_robot-motion365-b1-repro/research/curobo_v3
export PYTHONPATH=/home/tim/alfa_robot-motion365-b1-repro/research/curobo_v3/tools:/home/tim/Applications/isaacsim-5.1.0/exts/omni.isaac.ml_archive/pip_prebundle:/home/tim/Applications/isaacsim-5.1.0/kit/python/lib/python3.11/site-packages:/home/tim/alfa_robot-alfa_v3-curobo-repro/third_party/curobo-main-78fd485
CUDA_VISIBLE_DEVICES='' /home/tim/alfa_robot-motion365-b1-repro/.venv/bin/python -m unittest discover -s tests -p 'test_core_*.py'
```

Result: 250 passed, 1 skipped.

Strict analytic-lookahead sample:

```bash
cd /home/tim/alfa_robot-motion365-b1-repro/research/curobo_v3
export PYTHONPATH=/home/tim/alfa_robot-motion365-b1-repro/research/curobo_v3/tools:/home/tim/Applications/isaacsim-5.1.0/exts/omni.isaac.ml_archive/pip_prebundle:/home/tim/Applications/isaacsim-5.1.0/kit/python/lib/python3.11/site-packages:/home/tim/alfa_robot-alfa_v3-curobo-repro/third_party/curobo-main-78fd485
flock -w 120 /tmp/sevenova-curobo-gpu.lock env CUDA_VISIBLE_DEVICES=0 OPENBLAS_NUM_THREADS=4 PYTHONPATH="$PYTHONPATH" /home/tim/alfa_robot-motion365-b1-repro/.venv/bin/python tools/motion306_learning/evaluate.py --registry config/motion306_registry.json --output repro_results/analytic-smoke-cuda --route lookahead_heuristic --group dev --levels S30x40x40 --seed 101 --max-instances 1 --max-minutes 5 --threads 4 --contact-pool capped --place-gate
```

Paired local-only heuristic sample:

```bash
cd /home/tim/alfa_robot-motion365-b1-repro/research/curobo_v3
export PYTHONPATH=/home/tim/alfa_robot-motion365-b1-repro/research/curobo_v3/tools:/home/tim/Applications/isaacsim-5.1.0/exts/omni.isaac.ml_archive/pip_prebundle:/home/tim/Applications/isaacsim-5.1.0/kit/python/lib/python3.11/site-packages:/home/tim/alfa_robot-alfa_v3-curobo-repro/third_party/curobo-main-78fd485
flock -w 120 /tmp/sevenova-curobo-gpu.lock env CUDA_VISIBLE_DEVICES=0 OPENBLAS_NUM_THREADS=4 PYTHONPATH="$PYTHONPATH" /home/tim/alfa_robot-motion365-b1-repro/.venv/bin/python tools/motion306_learning/evaluate.py --registry config/motion306_registry.json --output repro_results/analytic-smoke-cuda --route heuristic --group dev --levels S30x40x40 --seed 101 --max-instances 1 --max-minutes 5 --threads 4 --contact-pool capped --place-gate
```

Mini size-envelope scan:

```bash
cd /home/tim/alfa_robot-motion365-b1-repro/research/curobo_v3
export PYTHONPATH=/home/tim/alfa_robot-motion365-b1-repro/research/curobo_v3/tools:/home/tim/Applications/isaacsim-5.1.0/exts/omni.isaac.ml_archive/pip_prebundle:/home/tim/Applications/isaacsim-5.1.0/kit/python/lib/python3.11/site-packages:/home/tim/alfa_robot-alfa_v3-curobo-repro/third_party/curobo-main-78fd485
flock -w 120 /tmp/sevenova-curobo-gpu.lock env CUDA_VISIBLE_DEVICES=0 OPENBLAS_NUM_THREADS=4 PYTHONPATH="$PYTHONPATH" /home/tim/alfa_robot-motion365-b1-repro/.venv/bin/python tools/experiments/motion306/size_envelope.py --output repro_results/size-envelope-mini --widths 0.36,0.37 --heights 0.30,0.35,0.40 --depths 0.30 --extra ''
```

## Local results

### Paired strict single-action sample

Instance: `L02-R01-dev-v0-S30x40x40`, dual top suction, seed 101.

- `lookahead_heuristic+screen`: full success; all three selected candidates were
  rank 0; timing/collision validation passed; selection overhead 226.10 ms;
  total planning wall time 9.02 s.
- `heuristic+screen`: reached placement, then failed `timing_validation` because
  `conveyor_place` timed samples failed the collision gate; selection overhead
  224.75 ms; total wall time 8.38 s.

This is representative evidence only, not a success-rate conclusion.

### Five-size smoke

The first sorted dev instance of every size was retained, including failures:

- 30 x 40 x 40 cm: success.
- 30 x 37 x 40 cm: all 64 contact candidates rejected by collision.
- 40 x 40 x 45 cm: no initial contact candidates.
- 35 x 38 x 42 cm held-out: no initial contact candidates.
- 40 x 37 x 45 cm held-out: no initial contact candidates.

### Mini envelope

- Width 36 cm: dual side/top 0/64 for every tested height.
- Width 37 cm: dual side/top 64/64 for every tested height.
- Single-arm width 37 cm: height 30 cm 0/64; height 35 cm 8-9/64;
  height 40 cm 27/64.

Raw evidence is under `research/curobo_v3/repro_results/`.

## B1 implementation added after reproduction

The named analytic B1 route, versioned cost config and final source-matched dev96
comparison are documented in:

- `research/curobo_v3/MOTION365_B1.md`
- `research/curobo_v3/config/motion365_b1_v1.json`
- `research/curobo_v3/repro_results/b1-v1-dev96-final/REPORT.md`

Final dev result: B1 73/96 versus local T_lb B0 75/96; selection p95 260.2 ms
versus 337.8 ms. The held-out test split was not opened.

The subsequent preregistered v2 ablation selected placement + Home direct-edge
risk: 75/96 at 266.5 ms selection p95. Frozen five-size dev evaluation completed
480 instances with 268 successes; held-out sizes were 95/192. Independent test
remains sealed. See `MOTION365_B1_V2_REPORT.md` and `MOTION365_B1_SIZE_REPORT.md`.

## Assets not present in Git

The original MOTION-311 delivery package is not in this repository. In particular,
the following original paths and hashes are documented but their files are absent:

- `/home/astesia/Sevenova/motion311-acceptance/delivery-20261025`
- `/home/astesia/Sevenova/motion311-fourroutes/`
- P3/D3 labels and frozen N1/N2/N3 model files
- Frozen L0 frame corpus

Therefore the reported 96-instance MOTION-337 result and four-route training result
can be audited from committed summaries and code, but cannot yet be rerun byte for
byte on this machine. Held-out `test` remains sealed and was not run here.
