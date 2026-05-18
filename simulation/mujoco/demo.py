import time
import mujoco
from alfa_env import AlfaEnv, MOVEIT_INITIAL_POSITIONS


def main():
    env = AlfaEnv(model_path="scene.xml", sim_dt=0.002, frame_skip=10)
    env.reset()

    ctrl_cmds = {
        "right_suction": 1.0,
        "left_suction": 1.0,
    }

    print("\n[ 测试启动 — current v5 MuJoCo 机器人 + 集装箱场景 ]")
    print(f"  初始姿态: {MOVEIT_INITIAL_POSITIONS}")
    print("  基座: pitch/turn/updown")
    print("  双臂: left_v5_joint1-6 / right_v5_joint1-6")
    print("  集装箱: 内宽 2.2m, 内高 2.4m, 5 排 × 10 个货物")

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
