# Zero is invalid so a default-constructed message cannot trigger motion.
uint8 STAGE_UNSPECIFIED=0
uint8 STAGE_TOP_SUCTION=1
uint8 STAGE_SIDE_SUCTION=2
uint8 STAGE_NO_MOVE=3

uint8 left_stage
geometry_msgs/Pose left_pose

uint8 right_stage
geometry_msgs/Pose right_pose
