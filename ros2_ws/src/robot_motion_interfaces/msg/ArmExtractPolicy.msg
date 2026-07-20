# Per-arm extraction policy selected from the external dual-grasp request.
string grasp_mode
bool require_full_detachment
# 侧吸脱离时必须清空的箱墙高度层数：
# 1=保持原等高侧吸规则；2=矮侧还必须离开原位置及正上方一层的 x-z 侧面投影。
# 顶吸填 0，此字段不参与顶吸判定。
uint8 front_clearance_levels
float64 retreat_priority
float64 lift_priority
float64 pitch_priority
