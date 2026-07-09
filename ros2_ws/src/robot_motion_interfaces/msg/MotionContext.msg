# Request metadata shared by motion services.
# scene_id identifies the semantic/planning scene version used by the caller.
# state_id identifies the robot-state sample used by the caller.
# Services must prefer explicit request state fields over live /joint_states.
string request_id
builtin_interfaces/Time stamp
string frame_id
string scene_id
string state_id
