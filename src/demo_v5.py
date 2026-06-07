import time
import math
import mujoco
from alfa_env_v5 import AlfaEnvV5


def main():
    env = AlfaEnvV5(model_path="E:\\qyx\\alfa_robot_v2_arm_v5\\scene_v5.xml", sim_dt=0.002, frame_skip=10)
    env.reset()

    # v5 joint mapping:
    #   躯干: pitch(Y hinge, 0~0.26), turn(Z hinge), updown(Z slide, 0~0.99)
    #   右臂: rightjoint1-6 (6 DOF, 全部铰链)
    #   左臂: leftjoint1-6  (6 DOF, 全部铰链)
    #   注意: v5 臂关节方向与 v2 不同, +pi/2 方向前伸
    ctrl_cmds = {
        "base_x":        0.0,
        "base_y":        0.0,
        "base_yaw":      0.0,
        "pitch":         0.0,
        "turn":          0.0,
        "updown":        0.3,

        "rightjoint1":   0.0,
        "rightjoint2":   math.pi/2,
        "rightjoint3":   0.0,
        "rightjoint4":  -math.pi/2,
        "rightjoint5":   0.0,
        "rightjoint6":   0.0,

        "leftjoint1":    0.0,
        "leftjoint2":    math.pi/2,
        "leftjoint3":    0.0,
        "leftjoint4":   -math.pi/2,
        "leftjoint5":    0.0,
        "leftjoint6":    0.0,

        "right_suction": 1.0,
        "left_suction":  1.0,
    }

    print("\n[ 测试启动 — v5 机器人 ]")
    print("  躯干: pitch / turn / updown")
    print("  左臂: leftjoint1-6 (6 DOF 全铰链)")
    print("  右臂: rightjoint1-6 (6 DOF 全铰链)")

    try:
        env.step(ctrl_cmds)

        while True:
            step_start = time.time()
            mujoco.mj_step(env.model, env.data)

            if not env.render():
                break

            elapsed = time.time() - step_start
            remaining = env.sim_dt - elapsed
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        env.close()


if __name__ == "__main__":
    main()
