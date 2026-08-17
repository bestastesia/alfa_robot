# Authoritative robot state sample for motion planning/execution.
# Exactly one runtime node should publish authoritative=true for the active robot.
MotionContext context
string source
bool authoritative
sensor_msgs/JointState joint_state
