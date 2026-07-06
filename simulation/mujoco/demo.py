import time
import mujoco
from alfa_env import AlfaEnv, MOVEIT_INITIAL_POSITIONS


REALTIME_TIMESTEP = 0.005
MAX_STEPS_PER_FRAME = 20


def main():
    env = AlfaEnv(model_path="scene.xml", sim_dt=REALTIME_TIMESTEP, frame_skip=1)
    env.reset()

    ctrl_cmds = {
        "right_suction": 1.0,
        "left_suction": 1.0,
    }

    print("\n[ 测试启动 — current MuJoCo 机器人 + 单排集装箱货物场景 ]")
    print(f"  初始姿态: {MOVEIT_INITIAL_POSITIONS}")
    print("  基座: pitch/turn/updown")
    print("  双臂: leftjoint1-6 / rightjoint1-6")
    print("  集装箱: 机器人身后约 1m, 地面放置, 内宽 2.2m, 内高 2.4m")
    print("  货物: 仅最前面一排，恢复完整碰撞，可推动")
    print(f"  实时模式: timestep={REALTIME_TIMESTEP}s, 每帧最多追赶 {MAX_STEPS_PER_FRAME} step")

    try:
        env.step(ctrl_cmds)

        next_sim_time = time.monotonic()

        while True:
            now = time.monotonic()
            steps = 0
            while next_sim_time <= now and steps < MAX_STEPS_PER_FRAME:
                mujoco.mj_step(env.model, env.data)
                next_sim_time += env.sim_dt
                steps += 1

            if not env.render():
                break

            remaining = next_sim_time - time.monotonic()
            if remaining > 0:
                time.sleep(min(remaining, env.sim_dt))
            elif steps >= MAX_STEPS_PER_FRAME:
                next_sim_time = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        env.close()


if __name__ == "__main__":
    main()
