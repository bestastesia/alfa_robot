# One candidate trajectory/result produced by a planning stage.
uint32 source_candidate_index
uint32 rank
bool success
string failure_reason
float64 cost
float64 planning_time_ms
float64 trajectory_distance
sensor_msgs/JointState start_state
sensor_msgs/JointState goal_state
trajectory_msgs/JointTrajectory trajectory
