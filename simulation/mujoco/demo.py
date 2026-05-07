import time
import math
import mujoco
from alfa_env import AlfaEnv

def main():
    env = AlfaEnv(model_path="scene.xml", sim_dt=0.002, frame_skip=10)
    env.reset()

    # v2 joint mapping:
    #   右臂: rightarmbase(横移), rightjoint1(伸缩), rightjoint2-4(旋转)
    #   左臂: leftarmbase(横移), leftjoint1(伸缩), leftjoint2-5(旋转), leftjoint6(末端伸缩)
    ctrl_cmds = {
        "base_x":        0.0,
        "base_y":        0.0,
        "base_yaw":      0.0,
        "turn":          0.0,
        "updown":        0.3,
        "plate":         0.0,

        "rightarmbase":  0.0,
        "rightjoint1":   0.0,
        "rightjoint2":   0.8,
        "rightjoint3":   0.0,
        "rightjoint4":   math.pi / 2.0,

        "leftarmbase":   0.0,
        "leftjoint1":    0.0,
        "leftjoint2":   -math.pi/2,
        "leftjoint3":    0.0,
        "leftjoint4":    math.pi/2,
        "leftjoint5":    0.0,
        "leftjoint6":    0.0,

        "right_suction": 1.0,
        "left_suction":  1.0,
    }

    print("\n[ 测试启动 — v2 机器人 ]")
    print("  右臂: rightarmbase/rightjoint1-4")
    print("  左臂: leftarmbase/leftjoint1-6 (新增 leftjoint5/6)")

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
