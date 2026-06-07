"""测试脚本：散落场景（随机位置、随机朝向、不紧密排列），加载模型进行可视化回放。"""
import argparse
import time
import numpy as np
import torch

from env import ContainerUnpackEnv
from model import A2CNetwork, collate_states

try:
    from mujoco import viewer
    HAS_VIEWER = True
except ImportError:
    HAS_VIEWER = False

DEFAULT_RGBA = (0.76, 0.62, 0.33, 1.0)
PICKABLE_RGBA = (0.15, 0.85, 0.35, 1.0)
LEFT_RGBA = (0.15, 0.45, 0.95, 1.0)
RIGHT_RGBA = (0.95, 0.2, 0.2, 1.0)
BOTH_RGBA = (0.85, 0.15, 0.85, 1.0)


def load_model(path, device, d_model, n_head, n_enc, dim_ff, dropout, d_safe):
    net = A2CNetwork(d_model=d_model, n_head=n_head, n_enc_layers=n_enc,
                     dim_feedforward=dim_ff, dropout=dropout, d_safe=d_safe).to(device)
    ckpt = torch.load(path, map_location=device)
    sd = ckpt["model"] if isinstance(ckpt, dict) and "model" in ckpt else ckpt
    net.load_state_dict(sd)
    net.eval()
    return net


def map_pickable_actions_to_global(local_left, local_right, pickable):
    gl, gr = -1, -1
    if 0 <= local_left < len(pickable):
        gl = int(pickable[local_left])
    if 0 <= local_right < len(pickable):
        gr = int(pickable[local_right])
    return gl, gr


def update_colors(env, pickable, left_idx=-1, right_idx=-1):
    ps = set(int(p) for p in pickable)
    for idx in range(env.n_max_boxes):
        if not env.box_active[idx]:
            continue
        gid = int(env._box_geom[idx])
        if idx == left_idx and idx == right_idx and idx >= 0:
            env._model.geom_rgba[gid] = BOTH_RGBA
        elif idx == left_idx and idx >= 0:
            env._model.geom_rgba[gid] = LEFT_RGBA
        elif idx == right_idx and idx >= 0:
            env._model.geom_rgba[gid] = RIGHT_RGBA
        elif idx in ps:
            env._model.geom_rgba[gid] = PICKABLE_RGBA
        else:
            env._model.geom_rgba[gid] = DEFAULT_RGBA


def main():
    p = argparse.ArgumentParser(description="散落场景可视化测试")
    p.add_argument("--model_path", type=str, required=True)
    p.add_argument("--device", type=str, default="cuda")
    p.add_argument("--steps", type=int, default=60)
    p.add_argument("--sleep", type=float, default=0.4)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--box_count", type=int, default=30)
    p.add_argument("--d_model", type=int, default=128)
    p.add_argument("--n_head", type=int, default=4)
    p.add_argument("--n_enc_layers", type=int, default=3)
    p.add_argument("--dim_ff", type=int, default=512)
    p.add_argument("--dropout", type=float, default=0.1)
    p.add_argument("--d_safe", type=float, default=0.5)
    args = p.parse_args()

    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")

    env = ContainerUnpackEnv()
    net = load_model(args.model_path, device, args.d_model, args.n_head,
                     args.n_enc_layers, args.dim_ff, args.dropout, args.d_safe)

    _, _, _, pickable = env.reset_scattered(box_count=args.box_count)
    total = int(env.box_active.sum())
    print(f"散落场景: {total} 个箱, pickable={len(pickable)}")

    step = 0
    paused = True

    def key_cb(keycode):
        nonlocal paused
        if keycode == 32:
            paused = not paused
            print("Run" if not paused else "Paused")

    viewer_ok = HAS_VIEWER
    v = None
    if viewer_ok:
        try:
            v = viewer.launch_passive(env._model, env._data, key_callback=key_cb)
        except Exception:
            viewer_ok = False

    print("按空格开始/暂停")
    try:
        while step < args.steps:
            if env.n_active == 0:
                print("全部清空")
                break

            if viewer_ok and v is not None:
                if paused:
                    v.sync()
                    time.sleep(0.03)
                    continue
                update_colors(env, pickable)
                v.sync()
            else:
                update_colors(env, pickable)

            sB, sP, _, pick_list = env._pack_obs(0.0, False, env._empty_info())[:4]
            s_B, s_P, mB, mP, pm, pos = collate_states([sB], [sP], device)
            with torch.no_grad():
                il, ir, lp_l, lp_r, _, _, _, value = net.sample_actions(s_B, s_P, mB, mP, pm, pos)
            gl, gr = map_pickable_actions_to_global(int(il.item()), int(ir.item()), pick_list)

            update_colors(env, pick_list, gl, gr)
            if viewer_ok and v is not None:
                v.sync()
                time.sleep(0.2)

            _, _, _, new_pickable, reward, done, _ = env.step(gl, gr)
            print(f"step={step:02d} 抓=({gl},{gr}) reward={reward:.2f} 剩余={env.n_active} pickable={len(new_pickable)}")
            pickable = new_pickable
            step += 1

            if viewer_ok and v is not None:
                v.sync()
                time.sleep(args.sleep)
            if done:
                print("结束")
                break
    finally:
        if viewer_ok and v is not None:
            v.close()


if __name__ == "__main__":
    main()
