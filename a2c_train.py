"""
A2C training for dual-arm container unpacking.

Optimizations over the original prototype:
  - correct action-index mapping from pickable-sequence space to global box ids,
  - correct fixed-action A2C updates,
  - richer environment metrics and logging,
  - improved state features and pickability approximation.
"""

import argparse
import json
import os
from collections import deque
from glob import glob

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    plt = None

try:
    from torch.utils.tensorboard import SummaryWriter
    HAS_TB = True
except ImportError:
    HAS_TB = False
    SummaryWriter = None

from env import ContainerUnpackEnv
from model import A2CNetwork, collate_states


class ParallelEnvRunner:
    def __init__(self, num_envs: int, env_kwargs: dict | None = None):
        env_kwargs = env_kwargs or {}
        self.num_envs = num_envs
        self.envs = [ContainerUnpackEnv(**env_kwargs) for _ in range(num_envs)]
        self.s_B = [None] * num_envs
        self.s_P = [None] * num_envs
        self.p_mask = [None] * num_envs
        self.pickable = [None] * num_envs
        self.episode_rewards = [0.0] * num_envs
        self.episode_lengths = [0] * num_envs
        self.finished_rewards = deque(maxlen=100)
        self.finished_remaining = deque(maxlen=100)
        self.finished_infos = deque(maxlen=100)
        self._reset_all()

    def _reset_all(self):
        for i in range(self.num_envs):
            self.s_B[i], self.s_P[i], self.p_mask[i], self.pickable[i] = self.envs[i].reset()
            self.episode_rewards[i] = 0.0
            self.episode_lengths[i] = 0

    def step(self, global_left, global_right):
        next_s_B = [None] * self.num_envs
        next_s_P = [None] * self.num_envs
        next_p_mask = [None] * self.num_envs
        next_pickable = [None] * self.num_envs
        rewards = np.zeros(self.num_envs, dtype=np.float32)
        dones = np.zeros(self.num_envs, dtype=bool)
        infos = []

        for i in range(self.num_envs):
            if self.s_B[i] is None:
                self.s_B[i], self.s_P[i], self.p_mask[i], self.pickable[i] = self.envs[i].reset()
                self.episode_rewards[i] = 0.0
                self.episode_lengths[i] = 0

            out = self.envs[i].step(int(global_left[i]), int(global_right[i]))
            next_s_B[i], next_s_P[i], next_p_mask[i], next_pickable[i], r, d, info = out
            rewards[i] = r
            dones[i] = d
            infos.append(info)
            self.episode_rewards[i] += r
            self.episode_lengths[i] += 1

            if d:
                self.finished_rewards.append(self.episode_rewards[i])
                self.finished_remaining.append(self.envs[i].n_active)
                self.finished_infos.append(info.copy())
                next_s_B[i], next_s_P[i], next_p_mask[i], next_pickable[i] = self.envs[i].reset()
                self.episode_rewards[i] = 0.0
                self.episode_lengths[i] = 0

        self.s_B, self.s_P, self.p_mask, self.pickable = next_s_B, next_s_P, next_p_mask, next_pickable
        return next_s_B, next_s_P, next_p_mask, next_pickable, rewards, dones, infos

    def get_current_states(self):
        return self.s_B, self.s_P, self.p_mask, self.pickable


def compute_gae(rewards, values, dones, gamma=0.99, lam=0.95):
    T = rewards.shape[0]
    advantages = torch.zeros(T, device=rewards.device)
    returns = torch.zeros(T, device=rewards.device)
    gae = 0.0
    for t in reversed(range(T)):
        mask = 1.0 - float(dones[t])
        delta = rewards[t] + gamma * values[t + 1] * mask - values[t]
        gae = delta + gamma * lam * mask * gae
        advantages[t] = gae
        returns[t] = advantages[t] + values[t]
    return advantages, returns


def export_training_curves(history: dict[str, list[tuple[int, float]]], output_path: str):
    if not HAS_MPL:
        return

    def smooth(values, window=10):
        arr = np.asarray(values, dtype=np.float32)
        if arr.size < 3:
            return arr
        window = min(window, arr.size)
        if window <= 1:
            return arr
        left = window // 2
        right = window - 1 - left
        padded = np.pad(arr, (left, right), mode="edge")
        kernel = np.ones(window, dtype=np.float32) / window
        return np.convolve(padded, kernel, mode="valid")

    panels = [
        ("episode/reward", "Episode Reward", "#4c78a8"),
        ("train/avg_pick_x_penalty", "Average X Penalty", "#e45756"),
        ("train/motion_distance_penalty", "Motion Distance Penalty", "#b279a2"),
        ("train/xy_close_penalty", "Y Close Penalty", "#72b7b2"),
        ("train/far_pair_penalty", "Far Pair Penalty", "#ff9da6"),
        ("train/cross_hand_penalty", "Cross-Hand Penalty", "#9d755d"),
        ("train/dropped_cleared", "Dropped Boxes Cleared", "#54a24b"),
        ("train/invalid_action_rate", "Invalid Action Rate", "#f58518"),
        ("train/free_space_reward", "Free Space Reward", "#a1dab4"),
    ]
    fig, axes = plt.subplots(3, 3, figsize=(16, 12), constrained_layout=True)
    axes = axes.ravel()
    for ax, (key, title, color) in zip(axes, panels):
        points = history.get(key, [])
        if points:
            xs = [s for s, _ in points]
            ys = [v for _, v in points]
            ax.plot(xs, ys, color=color, alpha=0.28, linewidth=1.0, label="raw")
            ax.plot(xs, smooth(ys), color=color, linewidth=2.1, label="smoothed")
            ax.legend()
        ax.set_title(title)
        ax.set_xlabel("Step")
        ax.grid(True, alpha=0.25)
    fig.suptitle("pick_a2c_2 RL Training Progress", fontsize=16)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def save_curve_history(history: dict[str, list[tuple[int, float]]], output_path: str):
    serializable = {k: [[int(step), float(value)] for step, value in points] for k, points in history.items()}
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(serializable, f)


def load_curve_history(path: str):
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    return {k: [(int(step), float(value)) for step, value in points] for k, points in raw.items()}


def resolve_resume_path(resume_arg: str, log_dir: str):
    if not resume_arg:
        return ""
    if resume_arg != "latest":
        return resume_arg
    candidates = glob(os.path.join(log_dir, "checkpoint_*.pt"))
    if not candidates:
        return ""
    candidates.sort(key=lambda p: int(os.path.splitext(os.path.basename(p))[0].split("_")[-1]))
    return candidates[-1]


def save_run_config(args, output_path: str):
    payload = {key: value for key, value in vars(args).items()}
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


def map_pickable_actions_to_global(local_left, local_right, pickable_lists):
    global_left = np.full_like(local_left, -1)
    global_right = np.full_like(local_right, -1)
    for i, pickable in enumerate(pickable_lists):
        if len(pickable) == 0:
            continue
        li = int(local_left[i])
        ri = int(local_right[i])
        if 0 <= li < len(pickable):
            global_left[i] = int(pickable[li])
        if 0 <= ri < len(pickable):
            global_right[i] = int(pickable[ri])
    return global_left, global_right


def train(args):
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    runner = ParallelEnvRunner(
        num_envs=args.num_envs,
        env_kwargs=dict(
            n_x=args.n_x,
            n_y=args.n_y,
            n_z=args.n_z,
            noise_prob=args.noise_prob,
            empty_ratio=args.empty_ratio,
            d_safe=args.d_safe,
        ),
    )
    print(f"Running {args.num_envs} parallel environments")

    net = A2CNetwork(
        d_model=args.d_model,
        n_head=args.n_head,
        n_enc_layers=args.n_enc_layers,
        dim_feedforward=args.dim_ff,
        dropout=args.dropout,
        d_safe=args.d_safe,
    ).to(device)

    optimizer = torch.optim.Adam(net.parameters(), lr=args.lr, eps=1e-5)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=max(args.total_steps, 1), eta_min=args.lr * 0.1)

    writer = SummaryWriter(log_dir=args.log_dir) if HAS_TB else None
    if writer is not None:
        print(f"TensorBoard log dir: {args.log_dir}")

    total_steps = 0
    best_reward = -float("inf")
    episode_reward_window = deque(maxlen=50)
    curve_path = os.path.join(args.log_dir, "training_curves.png")
    curve_history_path = os.path.join(args.log_dir, "curve_history.json")
    run_config_path = os.path.join(args.log_dir, "run_config.json")
    save_run_config(args, run_config_path)
    next_plot_step = args.plot_interval
    curve_history = {
        "episode/reward": [],
        "train/invalid_action_rate": [],
        "train/dropped_cleared": [],
        "train/dropped_severity_reward": [],
        "train/avg_pick_x_penalty": [],
        "train/motion_distance_penalty": [],
        "train/xy_close_penalty": [],
        "train/far_pair_penalty": [],
        "train/cross_hand_penalty": [],
        "train/free_space_reward": [],
    }

    resume_path = resolve_resume_path(args.resume, args.log_dir)
    if args.resume and not resume_path:
        print(f"Resume target not found for: {args.resume}")

    if resume_path:
        checkpoint = torch.load(resume_path, map_location=device)
        if isinstance(checkpoint, dict) and "model" in checkpoint:
            net.load_state_dict(checkpoint["model"])
            if "optimizer" in checkpoint:
                optimizer.load_state_dict(checkpoint["optimizer"])
            total_steps = int(checkpoint.get("step", 0))
            best_reward = float(checkpoint.get("best_reward", best_reward))
            next_plot_step = max(args.plot_interval, ((total_steps // args.plot_interval) + 1) * args.plot_interval)
            loaded_history = load_curve_history(curve_history_path)
            if loaded_history is not None:
                curve_history.update(loaded_history)
            print(f"Resumed from checkpoint: {resume_path} at step {total_steps}")
        else:
            net.load_state_dict(checkpoint)
            print(f"Loaded model weights from: {resume_path}")

    while total_steps < args.total_steps:
        buf_s_B, buf_s_P = [], []
        buf_action_l, buf_action_r = [], []
        buf_logp_l, buf_logp_r = [], []
        buf_reward, buf_done, buf_value = [], [], []
        rollout_infos = []

        for _ in range(args.rollout_steps):
            s_B_list, s_P_list, _, pick_list = runner.get_current_states()
            s_B_batch, s_P_batch, mask_B, mask_P, pickable_mask, positions = collate_states(s_B_list, s_P_list, device)

            with torch.no_grad():
                out = net.sample_actions(s_B_batch, s_P_batch, mask_B, mask_P, pickable_mask, positions)
            idx_l, idx_r, lp_l, lp_r, _, _, _, value = out

            local_left = idx_l.cpu().numpy()
            local_right = idx_r.cpu().numpy()
            global_left, global_right = map_pickable_actions_to_global(local_left, local_right, pick_list)
            _, _, _, _, rewards, dones, infos = runner.step(global_left, global_right)

            buf_s_B.append(s_B_list)
            buf_s_P.append(s_P_list)
            buf_action_l.append(local_left.copy())
            buf_action_r.append(local_right.copy())
            buf_logp_l.append(lp_l.detach())
            buf_logp_r.append(lp_r.detach())
            buf_reward.append(rewards)
            buf_done.append(dones)
            buf_value.append(value)
            rollout_infos.append(infos)

            for done in dones:
                if done and runner.finished_rewards:
                    episode_reward_window.append(runner.finished_rewards[-1])

            total_steps += 1
            if total_steps >= args.total_steps:
                break

        s_B_list, s_P_list, _, _ = runner.get_current_states()
        s_B_batch, s_P_batch, mask_B, mask_P, pickable_mask, positions = collate_states(s_B_list, s_P_list, device)
        with torch.no_grad():
            bootstrap_value = net.get_value(s_B_batch, s_P_batch, mask_B, mask_P, pickable_mask)

        T = len(buf_reward)
        rewards_t = torch.tensor(np.stack(buf_reward), dtype=torch.float32, device=device)
        dones_t = torch.tensor(np.stack(buf_done), dtype=torch.float32, device=device)
        values_t = torch.stack(buf_value)
        values_with_bootstrap = torch.cat([values_t, bootstrap_value.unsqueeze(0)], dim=0)

        all_advantages, all_returns = [], []
        for e in range(args.num_envs):
            adv, ret = compute_gae(rewards_t[:, e], values_with_bootstrap[:, e], dones_t[:, e], gamma=args.gamma, lam=args.lam)
            all_advantages.append(adv)
            all_returns.append(ret)
        advantages_t = torch.stack(all_advantages, dim=1)
        returns_t = torch.stack(all_returns, dim=1)

        advantages_flat = advantages_t.reshape(-1)
        advantages_flat = (advantages_flat - advantages_flat.mean()) / (advantages_flat.std() + 1e-8)
        advantages_t = advantages_flat.view_as(advantages_t)

        total_actor_loss = 0.0
        total_critic_loss = 0.0
        total_entropy = 0.0
        n_updates = 0

        for _ in range(args.epochs_per_rollout):
            indices = np.random.permutation(T * args.num_envs)
            for start in range(0, T * args.num_envs, args.batch_size):
                batch_idx = indices[start:start + args.batch_size]
                t_idx = [bi // args.num_envs for bi in batch_idx]
                e_idx = [bi % args.num_envs for bi in batch_idx]

                batch_s_B = [buf_s_B[t][e] for t, e in zip(t_idx, e_idx)]
                batch_s_P = [buf_s_P[t][e] for t, e in zip(t_idx, e_idx)]
                batch_adv = advantages_t[t_idx, e_idx]
                batch_ret = returns_t[t_idx, e_idx]
                batch_left = torch.tensor([buf_action_l[t][e] for t, e in zip(t_idx, e_idx)], dtype=torch.long, device=device)
                batch_right = torch.tensor([buf_action_r[t][e] for t, e in zip(t_idx, e_idx)], dtype=torch.long, device=device)

                batch_logp_old = torch.stack([
                    buf_logp_l[t][e] + buf_logp_r[t][e]
                    for t, e in zip(t_idx, e_idx)
                ]).to(device)

                s_B_b, s_P_b, mask_B_b, mask_P_b, pickable_b, pos_b = collate_states(batch_s_B, batch_s_P, device)
                lp_l, lp_r, entropy, values_new = net.evaluate_actions(s_B_b, s_P_b, mask_B_b, mask_P_b, pickable_b, pos_b, batch_left, batch_right)
                logp_new = lp_l + lp_r

                ratio = torch.exp(logp_new - batch_logp_old)
                clipped_ratio = torch.clamp(ratio, 1.0 - args.ppo_eps, 1.0 + args.ppo_eps)
                actor_loss = -torch.min(ratio * batch_adv, clipped_ratio * batch_adv).mean()
                critic_loss = F.mse_loss(values_new, batch_ret)
                entropy_loss = -entropy.mean()
                loss = actor_loss + args.value_coef * critic_loss + args.entropy_coef * entropy_loss

                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(net.parameters(), args.max_grad_norm)
                optimizer.step()
                scheduler.step()

                total_actor_loss += actor_loss.item()
                total_critic_loss += critic_loss.item()
                total_entropy += (-entropy_loss.item())
                n_updates += 1

        avg_actor = total_actor_loss / max(n_updates, 1)
        avg_critic = total_critic_loss / max(n_updates, 1)
        avg_entropy = total_entropy / max(n_updates, 1)

        flat_infos = [info for step_infos in rollout_infos for info in step_infos]
        avg_invalid = float(np.mean([info["invalid_picks"] for info in flat_infos])) if flat_infos else 0.0
        avg_dropped = float(np.mean([info["dropped_cleared"] for info in flat_infos])) if flat_infos else 0.0
        avg_drop_reward = float(np.mean([info["dropped_severity_reward"] for info in flat_infos])) if flat_infos else 0.0
        avg_x_penalty = float(np.mean([info["avg_pick_x_penalty"] for info in flat_infos])) if flat_infos else 0.0
        avg_motion_penalty = float(np.mean([info["motion_distance_penalty"] for info in flat_infos])) if flat_infos else 0.0
        avg_xy_close_penalty = float(np.mean([info["xy_close_penalty"] for info in flat_infos])) if flat_infos else 0.0
        avg_far_pair_penalty = float(np.mean([info["far_pair_penalty"] for info in flat_infos])) if flat_infos else 0.0
        avg_cross_hand_penalty = float(np.mean([info["cross_hand_penalty"] for info in flat_infos])) if flat_infos else 0.0
        avg_free_space = float(np.mean([info["free_space_reward"] for info in flat_infos])) if flat_infos else 0.0

        if writer is not None:
            writer.add_scalar("loss/actor", avg_actor, total_steps)
            writer.add_scalar("loss/critic", avg_critic, total_steps)
            writer.add_scalar("loss/entropy", avg_entropy, total_steps)
            writer.add_scalar("train/invalid_action_rate", avg_invalid, total_steps)
            writer.add_scalar("train/dropped_cleared", avg_dropped, total_steps)
            writer.add_scalar("train/avg_pick_x_penalty", avg_x_penalty, total_steps)
            writer.add_scalar("train/motion_distance_penalty", avg_motion_penalty, total_steps)
            writer.add_scalar("train/xy_close_penalty", avg_xy_close_penalty, total_steps)
            writer.add_scalar("train/far_pair_penalty", avg_far_pair_penalty, total_steps)
            writer.add_scalar("train/cross_hand_penalty", avg_cross_hand_penalty, total_steps)
            writer.add_scalar("train/free_space_reward", avg_free_space, total_steps)

        curve_history["train/invalid_action_rate"].append((total_steps, avg_invalid))
        curve_history["train/dropped_cleared"].append((total_steps, avg_dropped))
        curve_history["train/dropped_severity_reward"].append((total_steps, avg_drop_reward))
        curve_history["train/avg_pick_x_penalty"].append((total_steps, avg_x_penalty))
        curve_history["train/motion_distance_penalty"].append((total_steps, avg_motion_penalty))
        curve_history["train/xy_close_penalty"].append((total_steps, avg_xy_close_penalty))
        curve_history["train/far_pair_penalty"].append((total_steps, avg_far_pair_penalty))
        curve_history["train/cross_hand_penalty"].append((total_steps, avg_cross_hand_penalty))
        curve_history["train/free_space_reward"].append((total_steps, avg_free_space))

        if runner.finished_rewards:
            avg_ep_reward = float(np.mean(runner.finished_rewards))
            avg_remaining = float(np.mean(runner.finished_remaining))
            reward_window = float(np.mean(episode_reward_window)) if episode_reward_window else 0.0
            if writer is not None:
                writer.add_scalar("episode/reward", avg_ep_reward, total_steps)
                writer.add_scalar("episode/reward_window", reward_window, total_steps)
            curve_history["episode/reward"].append((total_steps, avg_ep_reward))
            if avg_ep_reward > best_reward:
                best_reward = avg_ep_reward
                torch.save(net.state_dict(), os.path.join(args.log_dir, "best_model.pt"))
            torch.save(net.state_dict(), os.path.join(args.log_dir, "last.pt"))

            print(
                f"Step {total_steps:7d} | Actor: {avg_actor:.4f} | Critic: {avg_critic:.4f} | Entropy: {avg_entropy:.4f} | "
                f"EpRew: {avg_ep_reward:.2f} | Remain: {avg_remaining:.1f} | Invalid: {avg_invalid:.2f} | "
                f"Dropped: {avg_dropped:.2f} | Xpen: {avg_x_penalty:.3f} | Motion: {avg_motion_penalty:.3f} | "
                f"Yclose: {avg_xy_close_penalty:.3f} | Far: {avg_far_pair_penalty:.3f} | Cross: {avg_cross_hand_penalty:.3f} | "
                f"FreeSpace: {avg_free_space:.3f}"
            )

        if total_steps >= next_plot_step:
            export_training_curves(curve_history, curve_path)
            save_curve_history(curve_history, curve_history_path)
            next_plot_step += args.plot_interval
        if total_steps % args.save_interval == 0:
            torch.save(
                {"model": net.state_dict(), "optimizer": optimizer.state_dict(), "step": total_steps, "best_reward": best_reward},
                os.path.join(args.log_dir, f"checkpoint_{total_steps}.pt"),
            )
            save_curve_history(curve_history, curve_history_path)

    export_training_curves(curve_history, curve_path)
    save_curve_history(curve_history, curve_history_path)
    if writer is not None:
        writer.close()
    print("Training complete.")


def main():
    parser = argparse.ArgumentParser(description="A2C Dual-Arm Unpacking")
    parser.add_argument("--n_x", type=int, default=6)
    parser.add_argument("--n_y", type=int, default=6)
    parser.add_argument("--n_z", type=int, default=5)
    parser.add_argument("--noise_prob", type=float, default=0.1)
    parser.add_argument("--empty_ratio", type=float, default=0.15)
    parser.add_argument("--d_safe", type=float, default=0.5)
    parser.add_argument("--d_model", type=int, default=128)
    parser.add_argument("--n_head", type=int, default=4)
    parser.add_argument("--n_enc_layers", type=int, default=3)
    parser.add_argument("--dim_ff", type=int, default=512)
    parser.add_argument("--dropout", type=float, default=0.1)
    parser.add_argument("--num_envs", type=int, default=8)
    parser.add_argument("--rollout_steps", type=int, default=64)
    parser.add_argument("--total_steps", type=int, default=200000)
    parser.add_argument("--batch_size", type=int, default=64)
    parser.add_argument("--epochs_per_rollout", type=int, default=4)
    parser.add_argument("--lr", type=float, default=5e-5)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--lam", type=float, default=0.95)
    parser.add_argument("--value_coef", type=float, default=0.5)
    parser.add_argument("--entropy_coef", type=float, default=0.05)
    parser.add_argument("--max_grad_norm", type=float, default=0.5)
    parser.add_argument("--ppo_eps", type=float, default=0.2)
    parser.add_argument("--device", type=str, default="cuda")
    parser.add_argument("--log_dir", type=str, default="./runs/a2c_unpack_continuous_v1")
    parser.add_argument("--save_interval", type=int, default=20000)
    parser.add_argument("--plot_interval", type=int, default=500)
    parser.add_argument("--resume", type=str, default="")
    args = parser.parse_args()
    base_dir = args.log_dir
    candidate = base_dir
    idx = 1
    while os.path.exists(candidate):
        idx += 1
        candidate = f"{base_dir}_v{idx}"
    args.log_dir = candidate
    os.makedirs(args.log_dir, exist_ok=True)
    train(args)


if __name__ == "__main__":
    main()
