# MOTION-386 three-IK plus final comparison table

This artifact exposes the selected contact, carry/recenter and placement IK for each
task, followed by the final whole-task result. It compares the frozen v2b baseline
with six preregistered custom-cost arms on the same default-size dev96.

## Shape

- 7 cost functions
- 96 tasks per function
- 4 rows per task: contact IK, carry/recenter IK, placement IK, final task
- 2,688 long-table rows
- all modes plus six mode-specific summary slices

The three IK rows include selected action index/rank, candidate count, Updown,
left/right swivel, q0-q14, every configured cost term, hard-screening counts,
selection timing components and stage planning result. The final row includes strict
success, failure stage/reason, all three selected action indices and Updown values,
total selection/planning time and raw/executed trajectory metrics.

## Default-size all-mode headline

| Function | Final success | Selection P50 | Selection P95 | Executed T_lb P50 |
|---|---:|---:|---:|---:|
| B1 v2b | 75/96 | 238.78 ms | 263.25 ms | 28.51 s |
| limit guard | 75/96 | 244.52 ms | 269.02 ms | 29.98 s |
| Updown stable | 74/96 | 240.67 ms | 266.80 ms | 28.89 s |
| pool robust | 74/96 | 242.90 ms | 273.92 ms | 28.43 s |
| Home bias | 76/96 | 241.34 ms | 268.05 ms | 28.55 s |
| return priority | 75/96 | 243.43 ms | 270.14 ms | 28.51 s |
| balanced combo | 75/96 | 241.69 ms | 262.57 ms | 28.92 s |

This is development evidence. Home bias remains a candidate rather than the selected
baseline because its five-size result is only +1/480 with eleven outcome swaps.

## Files

- `repro_results/motion386-three-ik-table/three_ik_plus_final_visual.html`
- `repro_results/motion386-three-ik-table/MOTION386_3IK加最终_函数参数对比.xlsx`
- `repro_results/motion386-three-ik-table/three_ik_plus_final_summary.csv`
- `repro_results/motion386-three-ik-table/three_ik_plus_final_long.csv`
- `repro_results/motion386-three-ik-table/three_ik_plus_final_data.json`

The standalone HTML first explains the shared lower-bound/risk mathematics and the
exact formula, weight, intent and observed limitation of all seven cost functions.
It then provides stage/function/mode/result filters. The workbook has one sheet per
IK stage and one final-task sheet; each sheet contains a seven-function summary,
charts and filterable task rows.

## Rebuild

Use `tools/experiments/motion386/three_ik_table.py`. Pass one
`--arm ID=LABEL:INSTANCES_DIR` argument for each frozen result directory and set
`--output-dir repro_results/motion386-three-ik-table`. The generator requires exactly
96 unique default-size instances for each arm and fails closed otherwise.

## Integrity

- HTML SHA256: `025abc0b7adee0c705a39c80e6de5c9db27c6f2a91f53c22598e7c43fcf05230`
- Summary CSV SHA256: `6ee27a9d71c492d9ba1b879532f7d83cd7734c288898b0c02cd2157d93599b16`
- Long CSV SHA256: `7109e1a368336bd74173d8a8167a5e8bfb7710a9c32ac99340e171f4ea074836`
- Workbook SHA256: `c4377497cf8b7bfe1b893ed24a0854423d40a22bacf861d7fe77c4c87934cb3d`
