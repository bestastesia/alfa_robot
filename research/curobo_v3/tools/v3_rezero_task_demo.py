import v3_visualized_task_demo as demo
from v3_rezero_task_planner import RezeroTaskPlanner, migrated_assets


def main():
    assets=migrated_assets()
    demo.FullCyclePlanner=RezeroTaskPlanner
    demo.DEFAULT_ROBOT_CONFIG=assets.robot_config
    demo.DEFAULT_MOBILE_ROBOT_CONFIG=assets.mobile_robot_config
    demo.DEFAULT_URDF=assets.urdf
    demo.DEFAULT_NAMED_POSES=assets.named_poses
    demo.main(default_port=8152)


if __name__=='__main__':
    main()
