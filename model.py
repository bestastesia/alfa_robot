"""
Transformer-based Actor-Critic network for dual-arm container unpacking.

The network consumes variable-length box feature sequences. The actor samples
in pickable-sequence index space during rollout and can also evaluate fixed
actions during training updates.
"""

import math
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Categorical


FEATURE_DIM = 16


class PositionalEncoding(nn.Module):
    def __init__(self, d_model: int, max_len: int = 512):
        super().__init__()
        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float32).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2, dtype=torch.float32) * (-math.log(10000.0) / d_model))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        self.register_buffer("pe", pe.unsqueeze(0))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return x + self.pe[:, : x.size(1), :]


class BoxEncoder(nn.Module):
    def __init__(
        self,
        input_dim: int = FEATURE_DIM,
        d_model: int = 128,
        n_head: int = 4,
        n_layers: int = 3,
        dim_feedforward: int = 512,
        dropout: float = 0.1,
        max_len: int = 512,
    ):
        super().__init__()
        self.d_model = d_model
        self.embed = nn.Sequential(
            nn.Linear(input_dim, d_model),
            nn.ReLU(inplace=True),
            nn.Linear(d_model, d_model),
        )
        self.pos_enc = PositionalEncoding(d_model, max_len)
        layer = nn.TransformerEncoderLayer(
            d_model=d_model,
            nhead=n_head,
            dim_feedforward=dim_feedforward,
            dropout=dropout,
            activation="gelu",
            batch_first=True,
        )
        self.encoder = nn.TransformerEncoder(layer, num_layers=n_layers)

    def forward(self, x: torch.Tensor, padding_mask: torch.Tensor | None = None) -> torch.Tensor:
        x = self.embed(x) * math.sqrt(self.d_model)
        x = self.pos_enc(x)
        return self.encoder(x, src_key_padding_mask=padding_mask)


class AutoregressiveActor(nn.Module):
    def __init__(self, d_model: int = 128, dropout: float = 0.1):
        super().__init__()
        self.left_scorer = nn.Sequential(
            nn.Linear(d_model, d_model),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(d_model, 1),
        )
        self.right_condition = nn.Sequential(
            nn.Linear(d_model * 2, d_model),
            nn.GELU(),
            nn.Dropout(dropout),
        )
        self.right_scorer = nn.Sequential(
            nn.Linear(d_model * 2, d_model),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(d_model, 1),
        )

    def _masked_distribution(self, logits: torch.Tensor, valid_mask: torch.Tensor) -> tuple[Categorical, torch.Tensor]:
        masked_logits = logits.masked_fill(~valid_mask, -1e9)
        probs = F.softmax(masked_logits, dim=-1)
        invalid_rows = valid_mask.sum(dim=-1) == 0
        if invalid_rows.any():
            probs = probs.clone()
            probs[invalid_rows] = 1.0 / probs.size(-1)
        return Categorical(probs=probs), probs

    def _build_right_context(self, fused, idx_left, pickable_mask, box_positions, d_safe):
        bsz, n_pick, _ = fused.shape
        device = fused.device
        idx_left = idx_left.clamp(0, max(n_pick - 1, 0))
        global_ctx = (fused * pickable_mask.unsqueeze(-1)).sum(dim=1) / pickable_mask.sum(dim=1, keepdim=True).clamp(min=1)
        left_feat = fused[torch.arange(bsz, device=device), idx_left]
        right_ctx = self.right_condition(torch.cat([global_ctx, left_feat], dim=-1))
        left_pos = box_positions[torch.arange(bsz, device=device), idx_left]
        dists = torch.norm(box_positions - left_pos.unsqueeze(1), dim=-1)
        picked_mask = F.one_hot(idx_left, n_pick).bool()
        proximity_mask = dists < d_safe
        action_mask = picked_mask | proximity_mask | (~pickable_mask)
        return right_ctx, action_mask

    def sample(self, fused, pickable_mask, box_positions, d_safe):
        left_logits = self.left_scorer(fused).squeeze(-1)
        left_dist, left_probs = self._masked_distribution(left_logits, pickable_mask)
        idx_left = left_dist.sample()
        left_log_prob = left_dist.log_prob(idx_left)
        left_entropy = left_dist.entropy()

        right_ctx, action_mask = self._build_right_context(fused, idx_left, pickable_mask, box_positions, d_safe)
        right_input = torch.cat([fused, right_ctx.unsqueeze(1).expand(-1, fused.size(1), -1)], dim=-1)
        right_logits = self.right_scorer(right_input).squeeze(-1)
        right_dist, right_probs = self._masked_distribution(right_logits, ~action_mask)
        idx_right = right_dist.sample()
        right_log_prob = right_dist.log_prob(idx_right)
        right_entropy = right_dist.entropy()
        return idx_left, idx_right, left_log_prob, right_log_prob, left_entropy + right_entropy, left_probs, right_probs

    def evaluate(self, fused, pickable_mask, box_positions, idx_left, idx_right, d_safe):
        left_logits = self.left_scorer(fused).squeeze(-1)
        left_dist, _ = self._masked_distribution(left_logits, pickable_mask)
        idx_left = idx_left.clamp(0, max(fused.size(1) - 1, 0))
        left_log_prob = left_dist.log_prob(idx_left)
        left_entropy = left_dist.entropy()

        right_ctx, action_mask = self._build_right_context(fused, idx_left, pickable_mask, box_positions, d_safe)
        right_input = torch.cat([fused, right_ctx.unsqueeze(1).expand(-1, fused.size(1), -1)], dim=-1)
        right_logits = self.right_scorer(right_input).squeeze(-1)
        right_dist, _ = self._masked_distribution(right_logits, ~action_mask)
        idx_right = idx_right.clamp(0, max(fused.size(1) - 1, 0))
        right_log_prob = right_dist.log_prob(idx_right)
        right_entropy = right_dist.entropy()
        return left_log_prob, right_log_prob, left_entropy + right_entropy


class Critic(nn.Module):
    def __init__(self, d_model: int = 128, hidden: int = 256, dropout: float = 0.1):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(d_model, hidden),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(hidden, hidden),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(hidden, 1),
        )

    def forward(self, fused: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
        pooled = (fused * mask.unsqueeze(-1)).sum(dim=1) / mask.sum(dim=1, keepdim=True).clamp(min=1)
        return self.net(pooled).squeeze(-1)


class A2CNetwork(nn.Module):
    def __init__(self, input_dim: int = FEATURE_DIM, d_model: int = 128, n_head: int = 4, n_enc_layers: int = 3, dim_feedforward: int = 512, dropout: float = 0.1, d_safe: float = 0.5):
        super().__init__()
        self.d_safe = d_safe
        self.pickable_encoder = BoxEncoder(input_dim=input_dim, d_model=d_model, n_head=n_head, n_layers=n_enc_layers, dim_feedforward=dim_feedforward, dropout=dropout)
        self.global_encoder = BoxEncoder(input_dim=input_dim, d_model=d_model, n_head=n_head, n_layers=n_enc_layers, dim_feedforward=dim_feedforward, dropout=dropout)
        self.cross_attn = nn.MultiheadAttention(embed_dim=d_model, num_heads=n_head, dropout=dropout, batch_first=True)
        self.fusion_norm = nn.LayerNorm(d_model)
        self.actor = AutoregressiveActor(d_model=d_model, dropout=dropout)
        self.critic = Critic(d_model=d_model, hidden=d_model * 2, dropout=dropout)

    def encode(self, s_B, s_P, mask_B, mask_P):
        enc_P = self.pickable_encoder(s_P, padding_mask=mask_P)
        enc_B = self.global_encoder(s_B, padding_mask=mask_B)
        fused, _ = self.cross_attn(query=enc_P, key=enc_B, value=enc_B, key_padding_mask=mask_B)
        return self.fusion_norm(enc_P + fused)

    def sample_actions(self, s_B, s_P, mask_B, mask_P, pickable_mask, box_positions):
        fused = self.encode(s_B, s_P, mask_B, mask_P)
        idx_l, idx_r, lp_l, lp_r, entropy, probs_l, probs_r = self.actor.sample(fused, pickable_mask, box_positions, self.d_safe)
        value = self.critic(fused, pickable_mask)
        return idx_l, idx_r, lp_l, lp_r, entropy, probs_l, probs_r, value

    def evaluate_actions(self, s_B, s_P, mask_B, mask_P, pickable_mask, box_positions, idx_left, idx_right):
        fused = self.encode(s_B, s_P, mask_B, mask_P)
        lp_l, lp_r, entropy = self.actor.evaluate(fused, pickable_mask, box_positions, idx_left, idx_right, self.d_safe)
        value = self.critic(fused, pickable_mask)
        return lp_l, lp_r, entropy, value

    def get_value(self, s_B, s_P, mask_B, mask_P, pickable_mask):
        fused = self.encode(s_B, s_P, mask_B, mask_P)
        return self.critic(fused, pickable_mask)


def collate_states(states_B: list[np.ndarray], states_P: list[np.ndarray], device: torch.device):
    batch_size = len(states_B)
    max_B = max(max(s.shape[0] for s in states_B), 1)
    max_P = max(max(s.shape[0] for s in states_P), 1)

    s_B = torch.zeros(batch_size, max_B, FEATURE_DIM, device=device)
    s_P = torch.zeros(batch_size, max_P, FEATURE_DIM, device=device)
    mask_B = torch.ones(batch_size, max_B, dtype=torch.bool, device=device)
    mask_P = torch.ones(batch_size, max_P, dtype=torch.bool, device=device)
    positions = torch.zeros(batch_size, max_P, 3, device=device)

    for i in range(batch_size):
        nB = states_B[i].shape[0]
        nP = states_P[i].shape[0]
        if nB > 0:
            s_B[i, :nB] = torch.from_numpy(states_B[i]).float().to(device)
            mask_B[i, :nB] = False
        if nP > 0:
            s_P[i, :nP] = torch.from_numpy(states_P[i]).float().to(device)
            mask_P[i, :nP] = False
            positions[i, :nP] = s_P[i, :nP, :3]

    pickable = ~mask_P
    return s_B, s_P, mask_B, mask_P, pickable, positions
