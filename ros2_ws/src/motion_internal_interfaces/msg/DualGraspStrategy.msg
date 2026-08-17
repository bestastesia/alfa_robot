# Internal strategy classification for one external dual-grasp task.
uint8 DUAL_FRONT_EQUAL=1
uint8 DUAL_FRONT=1
uint8 DUAL_TOP_EQUAL=2
uint8 DUAL_TOP_LEFT_HIGH=3
uint8 DUAL_TOP_RIGHT_HIGH=4
uint8 MIXED_EQUAL=5
uint8 MIXED_DEGRADED_TO_FRONT_EQUAL=5
uint8 MIXED_DEGRADED_TO_FRONT=5
uint8 DUAL_FRONT_LEFT_HIGH=6
uint8 DUAL_FRONT_RIGHT_HIGH=7
uint8 MIXED_LEFT_HIGH=8
uint8 MIXED_RIGHT_HIGH=9
uint8 MIXED_DEGRADED_TO_FRONT_LEFT_HIGH=8
uint8 MIXED_DEGRADED_TO_FRONT_RIGHT_HIGH=9

uint8 task_type
string name
ArmExtractPolicy left
ArmExtractPolicy right
float64 height_difference_m
