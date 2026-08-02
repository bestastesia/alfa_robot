# Per-arm extraction policy selected from the external dual-grasp request.
string grasp_mode
bool require_full_detachment
# 侧吸脱离所使用的参考箱层：
# 1=保持原等高侧吸规则，相对自己的原箱位判断；
# 2=不等高任务的矮侧，相对正上方一层（高 0.40m）的箱位判断，不再同时要求原箱位。
# 顶吸填 0，此字段不参与顶吸判定。
uint8 front_clearance_levels
float64 retreat_priority
float64 lift_priority
float64 pitch_priority
