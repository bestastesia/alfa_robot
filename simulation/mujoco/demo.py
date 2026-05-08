import time
import math
import mujoco
from alfa_env import AlfaEnv

def main():
    env = AlfaEnv(model_path="scene.xml", sim_dt=0.002, frame_skip=10)
    env.reset()

    # v2_arm_v4 joint mapping:
    #   基座: pitch, turn, updown
    #   双臂: leftjoint1-6 / rightjoint1-6
    ctrl_cmds = {
        "base_x":        0.0,
        "base_y":        0.0,
        "base_yaw":      0.0,
        "pitch":         0.0,
        "turn":          0.0,
        "updown":        0.3,

        "leftjoint1":    0.0,
        "leftjoint2":   -math.pi / 2.0,
        "leftjoint3":    0.0,
        "leftjoint4":    math.pi / 2.0,
        "leftjoint5":    0.0,
        "leftjoint6":    0.0,

        "rightjoint1":   0.0,
        "rightjoint2":   0.8,
        "rightjoint3":   0.0,
        "rightjoint4":   math.pi / 2.0,
        "rightjoint5":   0.0,
        "rightjoint6":   0.0,

        "right_suction": 1.0,
        "left_suction":  1.0,
    }

    print("\n[ 测试启动 — v2_arm_v4 新 URDF 机器人 ]")
    print("  基座: pitch/turn/updown")
    print("  双臂: leftjoint1-6 / rightjoint1-6")

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
