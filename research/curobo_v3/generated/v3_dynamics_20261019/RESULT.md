> 最新基线调查说明（2026-10-27）：冻结代理模型和Home与PR #65 HEAD逐字节一致。第10轮在新增3mm输送线搜索间隙下，密化候选因左右附着箱代理球互碰（最大2.97mm）被拒。更换种子只会绕过搜索脆弱性，不作为产品通过证据；PR保持Draft，等待根因修复。


# V3 dynamics overlay + cuRobo RNEA torque gate

Date: 2026-10-19

## Source

- Bundle: `robot_description.zip`
- SHA-256: `4716d6bf732c458de1504931716ce7d27d7142e8ccada289341d52f243fe2c52`
- Imported dynamic links: 17
- Imported mass sum: 62.273523 kg
- Method: map mass, COM, and inertia into the active analytic-proxy link frames at zero pose; keep the existing planner kinematics and collision model.

The full new URDF cannot replace the planner model directly: its link frames and joint limits differ from the current analytic proxy, and the frozen 25-box trajectory violates its J2 limits in many frames.

## 25-box torque audit (corrected eRob models)

The exact models are J1/J2 eRob170H160I, J3 eRob142H120I, J4 eRob142H160I, J5 eRob110H120I(V6), J6 eRob90H100I, and J7 eRob70H100I. The sustained gate is the manufacturer average allowable output torque multiplied by 90%; start/stop peak torque is not used as a sustained limit.

| Payload | Max / average limit | Max / configured 90% gate | Bottleneck | Failed segments | Result |
|---:|---:|---:|---|---:|---|
| 0 kg | 29.87% | 33.19% | left_joint7 | 0 | pass |
| 5 kg | 37.05% | 41.17% | right_joint6 | 0 | pass |
| 10 kg | 67.09% | 74.55% | right_joint6 | 0 | pass |
| 15 kg | 97.53% | 108.37% | left_joint6 | 15 | reject |
| 20 kg | 128.15% | 142.39% | left_joint6 | 21 | reject |
| 25 kg | 158.88% | 176.54% | left_joint6 | 27 | reject |
| 30 kg | 189.68% | 210.76% | left_joint6 | 29 | reject |
| 35 kg | 220.48% | 244.98% | left_joint6 | 30 | reject |
| 40 kg | 251.28% | 279.20% | left_joint6 | 33 | reject |

J3/J4 were swapped in the previous URDF description: J3 is 120:1 with 459 Nm start/stop peak and 16.7 RPM peak output speed; J4 is 160:1 with 484 Nm and 12.5 RPM. The dynamics overlay now carries the corrected start/stop effort values, while the torque gate reads the separate catalog profile and enforces average allowable torque.

## Runtime

Integrated in-place 20 kg audit over all 15 rounds:

- total: 690.0 ms
- steady-state median per round: 23.0 ms
- CUDA RNEA plus gravity-only computations: 174.6 ms total
- cold/new attachment-model peaks: 149.7 ms in round 1 and 246.8 ms when the side-suction transform first appears in round 10

The standalone full-wall JSON comparison takes about 3.3 s because it also copies and rebuilds the 150 MB report. It is not the runtime of the integrated planner gate.

## Safety boundary

- Payload mass is not supplied by the bundle; 5–40 kg are sensitivity cases, not measured box masses.
- The bundle worktree is dirty relative to its embedded commit and marks the dynamics as not hardware verified.
- URDF effort limits still require drive/controller confirmation.
- `updown` effort `10` has unverified units and is reported but not enforced.
- This is an RNEA feed-forward torque audit and fail-closed retiming gate, not a deployed closed-loop motor torque controller.

## Physics cross-check (2026-10-20)

- Payload COM distance is included through the per-round `tool_to_box` transform: 0.15 m for top suction and 0.20 m for side suction.
- Payload dimensions are 0.3 x 0.4 x 0.4 m; cuboid rotational inertia is rotated into the Tool0 frame and merged with the suction-tool inertia.
- CUDA RNEA receives q, qdot, and qddot for every frame and includes inertia, Coriolis/centrifugal, and gravity terms with g = 9.81 m/s^2.
- Robot link and suction-tool self-weight are included. Jerk is not an input to rigid-body inverse dynamics.
- Independent payload-potential finite-difference checks matched the RNEA payload gravity increment within 0.088 Nm for top suction and 0.247 Nm for side suction.
- RNEA decomposition residuals were below 3.1e-5 Nm in the checked top/side samples.
- Not modeled: joint friction/damping, gearbox loss/backlash, reflected motor inertia, electrical/thermal limits, and suction compliance.

## Added payload sensitivity

The same frozen q/qdot/qddot trajectory was audited at 30, 35, and 40 kg per active hand. Using the eRob average-load maximum x 90% as the sustained-load criterion:

- 30 kg: J3/J4/J6 definitely exceed even the highest family member; right J1 is also slightly above the eRob170H upper sustained limit.
- 35 kg: J1/J3/J4/J6 definitely exceed; right J5 also exceeds.
- 40 kg: J1/J3/J4/J6 and right J5 exceed the applicable upper sustained limits; left J5 remains submodel-dependent.

Detailed per-joint values are in `visualizations/mentor-rebased-timed/joint-torque-overview-erob100.html` and its CSV export.

## Peak-event dual-limit visualization (2026-10-24)

Current runtime enforcement uses the manufacturer average allowable output torque multiplied by 90%; it does not use rated torque or start/stop peak torque as the denominator. A new interactive report provides two explicit views:

1. rated/continuous view: rated utilization, average-allowable utilization, and current 90% gate utilization;
2. start/stop view: utilization of the manufacturer start/stop peak torque.

For every payload mass and every left/right joint, the report records the maximum RNEA joint torque, payload-only gravity torque, payload effective gravity lever arm, round/task, phase, segment time, nominal full-sequence time, base pose, and chassis-front distance to the original box-wall plane.

Environment: initial chassis-front to box-wall distance is 0.900 m; lower-wall pickup rounds advance to 0.600 m; conveyor-side events are approximately 2.016 m from the original wall plane in X after navigation/rotation. The full-sequence time axis is the 897.260 s nominal arm-motion timeline and excludes planning time and zero-duration navigation teleports.
