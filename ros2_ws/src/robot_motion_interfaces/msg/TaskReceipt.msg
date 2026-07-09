# External task receipt for whole-machine coordination.
# Core state values:
#   accepted, running, succeeded, failed, cancelled, stopped
string task_id
string state
builtin_interfaces/Time stamp
string message
