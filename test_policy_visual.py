import argparse
import time

import numpy as np
import torch

from env import ContainerUnpackEnv
from model import A2CNetwork, collate_states


DEFAULT_BOX_RGBA = (0.76, 0.62, 0.33, 1.0)
DEFAULT_DROPPED_RGBA = (0.88, 0.45, 0.25, 1.0)
PICKABLE_RGBA = (0.15, 0.85, 0.35, 1.0)
LEFT_SELECTED_RGBA = (0.15, 0.45, 0.95, 1.0)
RIGHT_SELECTED_RGBA = (0.95, 0.2, 0.2, 1.0)
BOTH_SELECTED_RGBA = (0.85, 0.15, 0.85, 1.0)


def map_pickable_actions_to_global(local_left, local_right, pickable):
    global_left = -1
    global_right = -1
    if 0 <= local_left < len(pickable):
        global_left = int(pickable[local_left])
    if 0 <= local_right < len(pickable):
        global_right = int(pickable[local_right])
    return global_left, global_right


def load_model(model_path: str, device: torch.device, d_model: int, n_head: int, n_enc_layers: int, dim_ff: int, dropout: float, d_safe: float):
    net = A2CNetwork(
        d_model=d_model,
        n_head=n_head,
        n_enc_layers=n_enc_layers,
        dim_feedforward=dim_ff,
        dropout=dropout,
        d_safe=d_safe,
    ).to(device)
    checkpoint = torch.load(model_path, map_location=device)
    if isinstance(checkpoint, dict) and "model" in checkpoint:
        state_dict = checkpoint["model"]
    else:
        state_dict = checkpoint
    net.load_state_dict(state_dict)
    net.eval()
    return net


def update_colors(env: ContainerUnpackEnv, pickable, left_idx=-1, right_idx=-1):
    pickable_set = set(int(idx) for idx in pickable)
    for idx in range(env.n_max_boxes):
        geom_id = int(env._box_geom[idx])
        if not env.box_active[idx]:
            continue
        if idx == left_idx and idx == right_idx and idx >= 0:
            env._model.geom_rgba[geom_id] = BOTH_SELECTED_RGBA
        elif idx == left_idx and idx >= 0:
            env._model.geom_rgba[geom_id] = LEFT_SELECTED_RGBA
        elif idx == right_idx and idx >= 0:
            env._model.geom_rgba[geom_id] = RIGHT_SELECTED_RGBA
        elif idx in pickable_set:
            env._model.geom_rgba[geom_id] = PICKABLE_RGBA
        elif env.box_is_dropped[idx]:
            env._model.geom_rgba[geom_id] = DEFAULT_DROPPED_RGBA
        else:
            env._model.geom_rgba[geom_id] = DEFAULT_BOX_RGBA


def print_exclusion_report(env: ContainerUnpackEnv):
    diagnostics = env.diagnose_pickability()
    excluded = [d for d in diagnostics if not d["is_pickable"]]
    if not excluded:
        print("All active boxes are currently pickable.")
        return
    print("Excluded boxes:")
    for item in excluded:
        reasons = []
        if not item["top_clear"]:
            reasons.append("top_blocked")
        if not item["front_clear"]:
            reasons.append("front_blocked")
        reason_text = ",".join(reasons) if reasons else "unknown"
        base = (
            f"  id={item['idx']:>3d} reason={reason_text:<20} row={item['row_x']} col={item['col_y']} layer={item['layer_z']} "
            f"x={item['x']:.3f} y={item['y']:.3f} z={item['z']:.3f} dropped={int(item['is_dropped'])}"
        )
        extra = []
        if item["top_blocker"] is not None:
            tb = item["top_blocker"]
            extra.append(
                f"top_blocker={tb['idx']} bottom_z={tb['bottom_z']:.3f} target_top_z={tb['target_top_z']:.3f} "
                f"z_gap={tb['z_gap']:.3f}/{tb['z_gap_limit']:.3f} dx={tb['dx']:.3f}/{tb['x_overlap_limit']:.3f} "
                f"dy={tb['dy']:.3f}/{tb['y_overlap_limit']:.3f}"
            )
        if item["front_blocker"] is not None:
            fb = item["front_blocker"]
            extra.append(
                f"front_blocker={fb['idx']} x_max={fb['x_max_j']:.3f} target_front_x={fb['target_front_x']:.3f} "
                f"y_range=[{fb['y_min_j']:.3f},{fb['y_max_j']:.3f}] target_y=[{fb['target_y_low']:.3f},{fb['target_y_high']:.3f}] "
                f"z_gap={fb['z_gap']:.3f}/{fb['z_limit']:.3f}"
            )
        if not extra:
            extra.append("no blocker details")
        print(base + " | " + " | ".join(extra))


def run_episode(env: ContainerUnpackEnv, net: A2CNetwork, device: torch.device, max_steps: int, sleep_time: float):
    _, _, _, pickable = env.reset()
    print(f"Initial active boxes: {env.n_active}, initial pickable: {len(pickable)}")
    print_exclusion_report(env)

    def select_action():
        sB, sP, _, pickable_now = env._pack_obs(0.0, False, env._empty_info())[:4]
        s_B, s_P, mask_B, mask_P, pickable_mask, positions = collate_states([sB], [sP], device)
        with torch.no_grad():
            idx_l, idx_r, _, _, _, _, _, _ = net.sample_actions(s_B, s_P, mask_B, mask_P, pickable_mask, positions)
        local_left = int(idx_l.item())
        local_right = int(idx_r.item())
        exec_left, exec_right = map_pickable_actions_to_global(local_left, local_right, pickable_now)
        return pickable_now, exec_left, exec_right

    def execute_action(step_idx: int, exec_left: int, exec_right: int):
        _, _, _, next_pickable, reward, done, info = env.step(exec_left, exec_right)
        print(
            f"step={step_idx:02d} exec=({exec_left},{exec_right}) reward={reward:.2f} remaining={env.n_active} "
            f"next_pickable={len(next_pickable)} avg_x_penalty={info['avg_pick_x_penalty']:.3f} "
            f"motion_penalty={info['motion_distance_penalty']:.3f} xy_close_penalty={info['xy_close_penalty']:.3f}"
            f"far_pair_penalty={info['far_pair_penalty']:.3f} cross_hand_penalty={info['cross_hand_penalty']:.3f} "
            f"free_space_reward={info['free_space_reward']:.3f}"
        )
        return done

    try:
        from mujoco import viewer
        paused = {"value": True}

        def on_key(keycode):
            if keycode == 32:
                paused["value"] = not paused["value"]
                print("Paused" if paused["value"] else "Running")

        with viewer.launch_passive(env._model, env._data, key_callback=on_key) as v:
            print("Playback starts paused. Press Space to continue/pause.")
            step = 0
            update_colors(env, pickable)
            v.sync()
            while step < max_steps:
                if env.n_active == 0:
                    print("All boxes removed.")
                    break
                if paused["value"]:
                    v.sync()
                    time.sleep(0.03)
                    continue

                current_pickable, exec_left, exec_right = select_action()
                update_colors(env, current_pickable, exec_left, exec_right)
                print_exclusion_report(env)
                v.sync()
                time.sleep(max(0.15, sleep_time))

                done = execute_action(step, exec_left, exec_right)
                step += 1
                v.sync()
                time.sleep(sleep_time)
                if done:
                    print("Episode done.")
                    break
    except Exception as exc:
        print("Viewer launch failed:", exc)
        print("Falling back to non-visual playback.")
        for step in range(max_steps):
            if env.n_active == 0:
                print("All boxes removed.")
                break
            pickable_now, exec_left, exec_right = select_action()
            update_colors(env, pickable_now, exec_left, exec_right)
            print_exclusion_report(env)
            done = execute_action(step, exec_left, exec_right)
            if done:
                print("Episode done.")
                break


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Load a trained .pt model and visualize unloading")
    parser.add_argument("--model_path", type=str, required=True)
    parser.add_argument("--device", type=str, default="cuda")
    parser.add_argument("--steps", type=int, default=30)
    parser.add_argument("--sleep", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=12345)
    parser.add_argument("--d_model", type=int, default=128)
    parser.add_argument("--n_head", type=int, default=4)
    parser.add_argument("--n_enc_layers", type=int, default=3)
    parser.add_argument("--dim_ff", type=int, default=512)
    parser.add_argument("--dropout", type=float, default=0.1)
    parser.add_argument("--d_safe", type=float, default=0.5)
    parser.add_argument("--scatter_prob", type=float, default=1.0)
    args = parser.parse_args()

    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    env = ContainerUnpackEnv(d_safe=args.d_safe, noise_prob=args.scatter_prob)
    net = load_model(
        args.model_path,
        device,
        d_model=args.d_model,
        n_head=args.n_head,
        n_enc_layers=args.n_enc_layers,
        dim_ff=args.dim_ff,
        dropout=args.dropout,
        d_safe=args.d_safe,
    )
    run_episode(env, net, device, max_steps=args.steps, sleep_time=args.sleep)
