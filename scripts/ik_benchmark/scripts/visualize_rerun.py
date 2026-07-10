#!/usr/bin/env python3
"""兼容入口；正式实现位于 alfa_robot_rerun 包。"""

from alfa_robot_rerun.visualize_rerun import *  # noqa: F401,F403
from alfa_robot_rerun.visualize_rerun import main


if __name__ == "__main__":
    main()
