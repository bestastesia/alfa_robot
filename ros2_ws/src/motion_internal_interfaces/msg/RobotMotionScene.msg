# Authoritative planning scene sample for motion services.
# Simulation/mock mode can set this once before a task.
# Real runtime can publish scene updates from perception/world-model services.
MotionContext context
string source
bool authoritative
moveit_msgs/CollisionObject[] scene_objects
moveit_msgs/AttachedCollisionObject[] attached_collision_objects
