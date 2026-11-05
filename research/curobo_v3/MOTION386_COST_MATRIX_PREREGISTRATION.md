# MOTION-386 multi-cost comparison preregistration

Frozen before producing any result for the six new functions on 2026-11-03.

All arms inherit selected B1 v2b and change only the listed soft preferences. IK,
Pose, joint limits, collision, extraction, RRT budgets, dense validation, timing,
candidate pools and seed remain unchanged. Every arm runs strict default-size dev96.
Independent test remains sealed.

The matrix tests distinct engineering hypotheses rather than a post-result weight
sweep. Because six hypotheses share the same dev set, the output is an exploratory
development comparison, not independent proof of generalization.

Selection order is: strict success count, no 2 s violations, paired-success executed
T_lb, then selection p95. An arm may advance to the 40 cm deep size only if it does
not reduce default-size strict success relative to the same-source v2b baseline.

See `config/motion386_cost_matrix.json` for exact weights and rationales.
