#!/usr/bin/env python3
from v3_task_full_cycle_demo import main
from v3_timed_single_arm_planner import TimedSingleArmPlanner


if __name__ == "__main__":
    main(planner_class=TimedSingleArmPlanner, default_port=8090, round_mode=True)
