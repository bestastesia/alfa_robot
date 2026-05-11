import time
import mujoco
from alfa_env import AlfaEnv


def main():
    env = AlfaEnv(model_path="scene.xml", sim_dt=0.002, frame_skip=10)
    env.reset()

    ctrl_cmds = {
        "right_suction": 1.0,
        "left_suction": 1.0,
    }

    print("\n[ 测试启动 — c1cf31b MuJoCo 机器人 ]")
    print("  初始姿态: 当前 URDF / MoveIt home 零位")
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
