# Zero is invalid so a default-constructed message cannot trigger motion.
uint8 STAGE_UNSPECIFIED=0
uint8 STAGE_TOP_SUCTION=1
uint8 STAGE_SIDE_SUCTION=2
uint8 STAGE_NO_MOVE=3

# Pose semantics depend on stage:
# - SIDE_SUCTION: actual front suction-surface center.
# - TOP_SUCTION: actual top suction-surface center.
# - NO_MOVE: pose is ignored. If exactly one arm uses NO_MOVE, Motion mirrors the
#   active target to the other arm by negating Y and executes a dual-arm fallback.
#   Both arms using NO_MOVE is invalid.
uint8 left_stage
geometry_msgs/Pose left_pose

uint8 right_stage
geometry_msgs/Pose right_pose
