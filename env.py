"""
MuJoCo-based container unloading environment for dual-arm RL.

Continuous environment: bad grasps, collapse, and messy states do not end the
episode early. The agent must keep unloading from the new state after every
physical update.
"""

import numpy as np

try:
    import mujoco
    HAS_MUJOCO = True
except ImportError:
    HAS_MUJOCO = False


_WALL_THICKNESS = 0.02
_FEATURE_DIM = 16


class ContainerUnpackEnv:
    def __init__(
        self,
        n_x: int = 6,
        n_y: int = 6,
        n_z: int = 5,
        box_size: float = 0.4,
        noise_prob: float = 0.1,
        empty_ratio: float = 0.15,
        max_noise_boxes: int = 2,
        d_safe: float = 0.5,
        collapse_threshold: float = 0.05,
        settle_steps: int = 40,
        sim_steps_per_action: int = 20,
        top_clearance_margin: float = 0.02,
        top_block_xy_relax: float = 0.03,
        front_clearance_margin: float = 0.03,
        near_x_coef: float = 1.8,
        motion_penalty_coef: float = 0.7,
        cross_hand_penalty_coef: float = 1.5,
        xy_close_penalty_coef: float = 2.0,
        xy_close_y_threshold: float = 0.05,
        far_pair_penalty_coef: float = 3.0,
        far_pair_y_threshold: float = 0.10,
        free_space_radius_coef: float = 1.5,
        free_space_reward_coef: float = 0.3,
    ):
        if not HAS_MUJOCO:
            raise ImportError("mujoco is required. Install via: pip install mujoco")

        self.n_x = n_x
        self.n_y = n_y
        self.n_z = n_z
        self.box_size = box_size
        self.noise_prob = noise_prob
        self.empty_ratio = empty_ratio
        self.max_noise_boxes = max_noise_boxes
        self.d_safe = d_safe
        self.collapse_threshold = collapse_threshold
        self.settle_steps = settle_steps
        self.sim_steps_per_action = sim_steps_per_action
        self.top_clearance_margin = top_clearance_margin
        self.top_block_xy_relax = top_block_xy_relax
        self.front_clearance_margin = front_clearance_margin
        self.near_x_coef = near_x_coef
        self.motion_penalty_coef = motion_penalty_coef
        self.cross_hand_penalty_coef = cross_hand_penalty_coef
        self.xy_close_penalty_coef = xy_close_penalty_coef
        self.xy_close_y_threshold = xy_close_y_threshold
        self.far_pair_penalty_coef = far_pair_penalty_coef
        self.far_pair_y_threshold = far_pair_y_threshold
        self.free_space_radius_coef = free_space_radius_coef
        self.free_space_reward_coef = free_space_reward_coef

        self.depth = n_x * box_size
        self.width = n_y * box_size
        self.height = n_z * box_size
        self.opening_x = 0.0

        self.n_grid = n_x * n_y * n_z
        self.n_max_boxes = self.n_grid + max_noise_boxes

        self._model = mujoco.MjModel.from_xml_string(self._build_xml())
        self._data = mujoco.MjData(self._model)
        self._cache_ids()

        self.box_active = np.zeros(self.n_max_boxes, dtype=bool)
        self.box_is_dropped = np.zeros(self.n_max_boxes, dtype=bool)
        self.box_sizes = np.zeros((self.n_max_boxes, 3), dtype=np.float32)
        self.box_drop_severity = np.zeros(self.n_max_boxes, dtype=np.float32)
        self.last_pick_position = None
        self.step_count = 0

    def _build_xml(self) -> str:
        depth, width, height = self.depth, self.width, self.height
        w = _WALL_THICKNESS
        lines = [
            '<mujoco model="container_unpack">',
            '  <size memory="300M"/>',
            '  <option gravity="0 0 -9.81" timestep="0.002"/>',
            '  <visual><map fogstart="3" fogend="5"/></visual>',
            '  <asset>',
            '    <texture name="t_wall" type="cube" builtin="flat" mark="cross" width="128" height="128" rgb1="0.55 0.55 0.55" rgb2="0.35 0.35 0.35" markrgb="0.15 0.15 0.15" random="0.01"/>',
            '    <texture name="t_box" type="cube" builtin="flat" mark="cross" width="64" height="64" rgb1="0.85 0.68 0.35" rgb2="0.60 0.48 0.25" markrgb="0.25 0.18 0.08" random="0.01"/>',
            '    <texture name="t_noise" type="cube" builtin="flat" mark="cross" width="64" height="64" rgb1="0.88 0.45 0.25" rgb2="0.65 0.32 0.18" markrgb="0.35 0.18 0.08" random="0.01"/>',
            '    <material name="m_wall" texture="t_wall" specular="0.1" shininess="0.3"/>',
            '    <material name="m_box" texture="t_box" specular="0.05" shininess="0.2"/>',
            '    <material name="m_noise" texture="t_noise" specular="0.05" shininess="0.2"/>',
            '  </asset>',
            '  <worldbody>',
            f'    <geom name="ground" type="plane" size="8 8 0.1" pos="0 0 0" material="m_wall"/>',
            '    <body name="container" pos="0 0 0">',
            f'      <geom name="w_bottom" type="box" pos="{depth/2} 0 {-w/2}" size="{depth/2+w} {width/2+w} {w/2}" material="m_wall"/>',
            f'      <geom name="w_top" type="box" pos="{depth/2} 0 {height+w/2}" size="{depth/2+w} {width/2+w} {w/2}" material="m_wall"/>',
            f'      <geom name="w_left" type="box" pos="{depth/2} {-width/2-w/2} {height/2}" size="{depth/2+w} {w/2} {height/2+w}" material="m_wall"/>',
            f'      <geom name="w_right" type="box" pos="{depth/2} {width/2+w/2} {height/2}" size="{depth/2+w} {w/2} {height/2+w}" material="m_wall"/>',
            f'      <geom name="w_back" type="box" pos="{depth+w/2} 0 {height/2}" size="{w/2} {width/2+w} {height/2+w}" material="m_wall"/>',
            '    </body>',
        ]
        for i in range(self.n_max_boxes):
            lines.append(f'    <body name="box_{i}" pos="100 100 100">')
            lines.append(f'      <freejoint name="box_{i}_j"/>')
            lines.append(f'      <geom name="box_{i}_g" type="box" size="0.2 0.2 0.2" material="m_box" density="300"/>')
            lines.append('    </body>')
        lines.append('  </worldbody>')
        lines.append('</mujoco>')
        return '\n'.join(lines)

    def _cache_ids(self):
        self._box_joint = np.zeros(self.n_max_boxes, dtype=np.int32)
        self._box_geom = np.zeros(self.n_max_boxes, dtype=np.int32)
        self._box_qpos = np.zeros(self.n_max_boxes, dtype=np.int32)
        for i in range(self.n_max_boxes):
            self._box_joint[i] = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, f"box_{i}_j")
            self._box_geom[i] = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_GEOM, f"box_{i}_g")
            self._box_qpos[i] = self._model.jnt_qposadr[self._box_joint[i]]

    def reset(self):
        mujoco.mj_resetData(self._model, self._data)
        self.box_active.fill(False)
        self.box_is_dropped.fill(False)
        self.box_drop_severity.fill(0.0)
        self.last_pick_position = None
        self.step_count = 0

        idx = 0
        height_map = self._sample_height_map()
        for k in range(self.n_x):
            for j in range(self.n_y):
                stack_h = int(height_map[k, j])
                for i in range(stack_h):
                    x = self.box_size / 2 + k * self.box_size + np.random.uniform(-0.003, 0.003)
                    y = -self.width / 2 + self.box_size / 2 + j * self.box_size + np.random.uniform(-0.003, 0.003)
                    z = self.box_size / 2 + i * self.box_size
                    self._place_box(idx, x, y, z, self.box_size, self.box_size, self.box_size, 1.0, 0.0, 0.0, 0.0, dropped=False)
                    idx += 1

        if np.random.random() < self.noise_prob and idx < self.n_max_boxes:
            n_noise = np.random.randint(1, min(self.max_noise_boxes + 1, self.n_max_boxes - idx + 1))
            existing = list(np.flatnonzero(self.box_active))
            for _ in range(n_noise):
                if idx >= self.n_max_boxes:
                    break
                sx = self.box_size * np.random.uniform(0.85, 1.05)
                sy = self.box_size * np.random.uniform(0.85, 1.05)
                sz = self.box_size * np.random.uniform(0.70, 0.95)
                sampled = self._sample_non_overlapping_dropped_box(existing, sx, sy, sz)
                if sampled is None:
                    continue
                x, y, z = sampled
                yaw = np.random.uniform(0, 2 * np.pi)
                qw = np.cos(yaw / 2)
                qz = np.sin(yaw / 2)
                self._place_box(idx, x, y, z, sx, sy, sz, qw, 0.0, 0.0, qz, dropped=True)
                existing.append(idx)
                idx += 1

        for _ in range(self.settle_steps):
            mujoco.mj_step(self._model, self._data)

        return self._pack_obs(0.0, False, self._empty_info())[:4]

    def _sample_height_map(self):
        heights = np.zeros((self.n_x, self.n_y), dtype=np.int32)
        front_clear_cols = max(1, self.n_x // 3)
        for k in range(front_clear_cols):
            heights[k, :] = 0
        for k in range(front_clear_cols, self.n_x):
            for j in range(self.n_y):
                max_h = self.n_z
                if k > front_clear_cols:
                    max_h = min(max_h, heights[k - 1, j] + 1)
                    max_h = max(1, max_h)
                min_h = max(1, max_h - 2)
                base_h = np.random.randint(min_h, max_h + 1)
                if np.random.random() < self.empty_ratio:
                    base_h = max(1, base_h - 1)
                heights[k, j] = int(np.clip(base_h, 1, self.n_z))
        return heights

    def _sample_non_overlapping_dropped_box(self, existing_indices, sx, sy, sz):
        x_hi = min(self.depth * 0.22, self.depth - sx / 2)
        if x_hi <= sx / 2:
            return None
        for _ in range(40):
            x = np.random.uniform(sx / 2, x_hi)
            y = np.random.uniform(-self.width / 2 + sy / 2, self.width / 2 - sy / 2)
            z = np.random.uniform(sz / 2, min(self.height * 0.35, self.height - sz / 2))
            overlapped = False
            for idx in existing_indices:
                p = self._get_position(idx)
                s = self.box_sizes[idx]
                if abs(x - p[0]) < (sx + s[0]) / 2 and abs(y - p[1]) < (sy + s[1]) / 2 and abs(z - p[2]) < (sz + s[2]) / 2:
                    overlapped = True
                    break
            if not overlapped:
                return x, y, z
        return None

    def reset_clean_grid(self):
        """整齐满箱场景：所有 n_x*n_y*n_z 格子填满标准箱，无抖动无散落。"""
        return self.reset_clean_grid_custom(self.box_size, self.n_x, self.n_y, self.n_z)

    def reset_clean_grid_custom(self, box_size, n_x, n_y, n_z):
        """整齐满箱可变规格：自定义箱体边长和网格数，填满无抖动无散落。"""
        total = n_x * n_y * n_z
        if total > self.n_max_boxes:
            raise ValueError(f"网格 {n_x}x{n_y}x{n_z}={total} 超出最大箱数 {self.n_max_boxes}")
        mujoco.mj_resetData(self._model, self._data)
        self.box_active.fill(False)
        self.box_is_dropped.fill(False)
        self.box_drop_severity.fill(0.0)
        self.last_pick_position = None
        self.step_count = 0

        idx = 0
        for k in range(n_x):
            for j in range(n_y):
                for i in range(n_z):
                    x = box_size / 2 + k * box_size
                    y = -self.width / 2 + box_size / 2 + j * box_size
                    z = box_size / 2 + i * box_size
                    self._place_box(idx, x, y, z, box_size, box_size, box_size, 1.0, 0.0, 0.0, 0.0, dropped=False)
                    idx += 1

        for _ in range(self.settle_steps):
            mujoco.mj_step(self._model, self._data)

        return self._pack_obs(0.0, False, self._empty_info())[:4]

    def reset_scattered(self, box_count: int = 30, box_size: float = 0.4):
        """散落场景：随机位置随机朝向，每柱随机1-3层，各层yaw可对齐或扰动。"""
        mujoco.mj_resetData(self._model, self._data)
        self.box_active.fill(False)
        self.box_is_dropped.fill(False)
        self.box_drop_severity.fill(0.0)
        self.last_pick_position = None
        self.step_count = 0

        idx = 0
        base_count = max(1, box_count // 2)  # 底座数量
        bases = []
        gap_xy = box_size * 0.12

        # 底座：随机散布
        attempts = 0
        while len(bases) < base_count and attempts < base_count * 20:
            sx = box_size * np.random.uniform(0.8, 1.1)
            sy = box_size * np.random.uniform(0.8, 1.1)
            sz = box_size * np.random.uniform(0.7, 1.0)
            x = np.random.uniform(sx / 2 + 0.05, self.depth - sx / 2 - 0.05)
            y = np.random.uniform(-self.width / 2 + sy / 2 + 0.05, self.width / 2 - sy / 2 - 0.05)
            yaw = np.random.uniform(0, 2 * np.pi)
            overlap = False
            for bx, by, _, bsx, bsy, _, _ in bases:
                if (abs(x - bx) < (sx + bsx) / 2 + gap_xy and
                    abs(y - by) < (sy + bsy) / 2 + gap_xy):
                    overlap = True; break
            if not overlap:
                qw = np.cos(yaw / 2); qz = np.sin(yaw / 2)
                z = sz / 2
                self._place_box(idx, x, y, z, sx, sy, sz, qw, 0.0, 0.0, qz, dropped=True)
                bases.append((x, y, z, sx, sy, sz, yaw))
                idx += 1
            attempts += 1

        # 每柱随机 1-3 层
        np.random.shuffle(bases)
        for bx, by, bz, bsx, bsy, bsz, byaw in bases:
            if idx >= self.n_max_boxes:
                break
            n_layers = np.random.choice([1, 2, 3], p=[0.4, 0.35, 0.25])  # 总层数
            cx, cy, cz, csz, cyaw = bx, by, bz, bsz, byaw
            for _ in range(n_layers - 1):
                if idx >= self.n_max_boxes:
                    break
                sx = box_size * np.random.uniform(0.8, 1.1)
                sy = box_size * np.random.uniform(0.8, 1.1)
                sz = box_size * np.random.uniform(0.7, 1.0)
                x = cx + np.random.uniform(-0.05, 0.05)
                y = cy + np.random.uniform(-0.05, 0.05)
                z = cz + csz / 2 + sz / 2 + np.random.uniform(0.0, 0.02)
                if np.random.random() < 0.4:
                    yaw = cyaw
                else:
                    yaw = cyaw + np.random.uniform(-0.5, 0.5)
                qw = np.cos(yaw / 2); qz = np.sin(yaw / 2)
                self._place_box(idx, x, y, z, sx, sy, sz, qw, 0.0, 0.0, qz, dropped=True)
                cx, cy, cz, csz, cyaw = x, y, z, sz, yaw
                idx += 1

        for _ in range(5):
            mujoco.mj_step(self._model, self._data)

        return self._pack_obs(0.0, False, self._empty_info())[:4]

    def step(self, idx_left, idx_right):
        old_pickable = self._compute_pickable()
        pickable_set = set(old_pickable)
        old_active = int(self.box_active.sum())

        reward = 0.0
        done = False
        info = self._empty_info()
        self.step_count += 1

        lv = idx_left in pickable_set and self.box_active[idx_left]
        rv = idx_right in pickable_set and self.box_active[idx_right]
        info["invalid_picks"] = int(not lv) + int(not rv)

        old_z = {i: float(self._data.qpos[self._box_qpos[i] + 2]) for i in np.flatnonzero(self.box_active)}
        active_positions = [self._get_position(i) for i in np.flatnonzero(self.box_active)]
        if active_positions:
            x_values = [float(p[0]) for p in active_positions]
            current_x_min = min(x_values)
            current_x_max = max(x_values)
        else:
            current_x_min = 0.0
            current_x_max = 1.0
        x_span = max(current_x_max - current_x_min, 1e-6)

        if idx_left == idx_right and idx_left >= 0 and lv and rv:
            reward -= 10.0
            info["same_target"] = True

        if lv and rv and idx_left != idx_right:
            reward += 2.0
        else:
            reward -= 4.0
            info["dual_pick_failed"] = 1.0

        selected = []
        if lv:
            selected.append(idx_left)
        if rv and idx_right != idx_left:
            selected.append(idx_right)

        dropped_priority = 0.0
        dropped_severity_reward = 0.0
        avg_pick_x_penalty = 0.0
        motion_distance_penalty = 0.0
        xy_close_penalty = 0.0
        far_pair_penalty = 0.0
        cross_hand_penalty = 0.0
        free_space_reward = 0.0
        picked_positions = []

        for vid in selected:
            p = self._get_position(vid)
            picked_positions.append(p)
            relative_x = (float(p[0]) - current_x_min) / x_span
            avg_pick_x_penalty += relative_x
            reward -= self.near_x_coef * relative_x
            reward += 0.15 * float(p[2]) / max(self.height, 1e-6)
            free_radius = self.free_space_radius_coef * self.box_size
            nearby_count = 0
            for j in np.flatnonzero(self.box_active):
                if j == vid:
                    continue
                if float(np.linalg.norm(self._get_position(j) - p)) < free_radius:
                    nearby_count += 1
            free_space_reward += 1.0 / max(float(nearby_count) + 1.0, 1.0)
            if self.box_is_dropped[vid]:
                severity = float(self.box_drop_severity[vid])
                early_bonus = (1.0 / max(self.step_count, 1)) * (0.5 + 0.5 * severity)
                reward += early_bonus
                dropped_priority += 1.0
                dropped_severity_reward += early_bonus

        if len(selected) == 2:
            p0, p1 = picked_positions[0], picked_positions[1]
            dx = abs(float(p0[0] - p1[0]))
            dy = abs(float(p0[1] - p1[1]))
            dz = abs(float(p0[2] - p1[2]))
            if dy < self.xy_close_y_threshold:
                xy_close_penalty = 1.0 - dy / max(self.xy_close_y_threshold, 1e-6)
                reward -= self.xy_close_penalty_coef * xy_close_penalty
            if dy > self.far_pair_y_threshold:
                far_pair_penalty = (
                    dx / max(self.depth, 1e-6)
                    + dy / max(self.width, 1e-6)
                    + dz / max(self.height, 1e-6)
                )
                reward -= self.far_pair_penalty_coef * far_pair_penalty
            if float(p0[1]) > float(p1[1]):
                cross_hand_penalty = float(p0[1] - p1[1]) / max(self.width, 1e-6)
                reward -= self.cross_hand_penalty_coef * cross_hand_penalty

        reward += self.free_space_reward_coef * free_space_reward

        if selected:
            avg_pick_x_penalty /= max(len(selected), 1)
            current_pick_center = np.mean(np.stack(picked_positions, axis=0), axis=0)
            if self.last_pick_position is not None:
                motion_distance = float(np.linalg.norm(current_pick_center - self.last_pick_position))
                motion_distance_penalty = motion_distance / max(np.linalg.norm([self.depth, self.width, self.height]), 1e-6)
                reward -= self.motion_penalty_coef * motion_distance_penalty
            self.last_pick_position = current_pick_center.copy()

        for vid in selected:
            self._deactivate_box(vid)

        for _ in range(self.sim_steps_per_action):
            mujoco.mj_step(self._model, self._data)

        collapse_events = 0
        for i in np.flatnonzero(self.box_active):
            if i in old_z:
                new_z = float(self._data.qpos[self._box_qpos[i] + 2])
                if old_z[i] - new_z > self.collapse_threshold:
                    collapse_events += 1
        if collapse_events > 0:
            reward -= 2.0 * collapse_events
            info["collapse"] = True

        removed_count = max(0, old_active - int(self.box_active.sum()))
        reward += 0.1 * removed_count

        info["removed_count"] = float(removed_count)
        info["dropped_cleared"] = float(dropped_priority)
        info["dropped_severity_reward"] = float(dropped_severity_reward)
        info["avg_pick_x_penalty"] = float(avg_pick_x_penalty)
        info["motion_distance_penalty"] = float(motion_distance_penalty)
        info["xy_close_penalty"] = float(xy_close_penalty)
        info["far_pair_penalty"] = float(far_pair_penalty)
        info["cross_hand_penalty"] = float(cross_hand_penalty)
        info["free_space_reward"] = float(free_space_reward)

        if self.box_active.sum() == 0:
            reward += 5.0
            done = True

        return self._pack_obs(reward, done, info)

    def _pack_obs(self, reward, done, info):
        pickable = self._compute_pickable()
        mask = np.zeros(self.n_max_boxes, dtype=bool)
        mask[pickable] = True
        return (
            self._get_state(np.flatnonzero(self.box_active), pickable),
            self._get_state(pickable, pickable),
            mask,
            np.array(pickable, dtype=np.int32),
            reward,
            done,
            info,
        )

    def _place_box(self, idx, x, y, z, sx, sy, sz, qw, qx, qy, qz, dropped):
        addr = self._box_qpos[idx]
        self._data.qpos[addr: addr + 7] = [x, y, z, qw, qx, qy, qz]
        gid = self._box_geom[idx]
        self._model.geom_size[gid] = [sx / 2, sy / 2, sz / 2]
        self._model.geom_contype[gid] = 1
        self._model.geom_conaffinity[gid] = 1
        self._model.geom_matid[gid] = 2 if dropped else 1
        self.box_active[idx] = True
        self.box_is_dropped[idx] = dropped
        self.box_sizes[idx] = [sx, sy, sz]
        self.box_drop_severity[idx] = self._compute_drop_severity(x, y, z, sx, sy, sz) if dropped else 0.0

    def _deactivate_box(self, idx):
        addr = self._box_qpos[idx]
        self._data.qpos[addr: addr + 3] = [100, 100, 100]
        self._data.qpos[addr + 3: addr + 7] = [1, 0, 0, 0]
        gid = self._box_geom[idx]
        self._model.geom_contype[gid] = 0
        self._model.geom_conaffinity[gid] = 0
        self.box_active[idx] = False

    def _get_position(self, idx):
        addr = self._box_qpos[idx]
        return np.array(self._data.qpos[addr: addr + 3], dtype=np.float32).copy()

    def _compute_pickable(self):
        active = np.flatnonzero(self.box_active)
        pickable = []
        for i in active:
            top_clear = self._has_top_clearance(i, active)
            front_clear = self._has_front_clearance(i, active)
            if top_clear and front_clear:
                pickable.append(i)
        pickable.sort(key=self._pickable_priority)
        return pickable

    def _pickable_priority(self, idx):
        pos = self._get_position(idx)
        front_priority = pos[0]
        dropped_bonus = -0.22 * float(self.box_is_dropped[idx]) - 0.28 * float(self.box_drop_severity[idx])
        motion_bonus = 0.0
        if self.last_pick_position is not None:
            motion_bonus = 0.18 * float(np.linalg.norm(pos - self.last_pick_position))
        z_bonus = -0.02 * pos[2]
        return front_priority + motion_bonus + dropped_bonus + z_bonus

    def _top_blocker_info(self, idx, active):
        pi = self._get_position(idx)
        si = self.box_sizes[idx]
        top_z = pi[2] + si[2] / 2
        for j in active:
            if j == idx:
                continue
            pj = self._get_position(j)
            sj = self.box_sizes[j]
            bottom_z = pj[2] - sj[2] / 2
            z_gap = bottom_z - top_z
            if bottom_z < top_z - self.top_clearance_margin:
                continue
            dx = abs(pi[0] - pj[0])
            dy = abs(pi[1] - pj[1])
            x_overlap_limit = max(0.0, (si[0] + sj[0]) / 2 - self.top_block_xy_relax)
            y_overlap_limit = max(0.0, (si[1] + sj[1]) / 2 - self.top_block_xy_relax)
            if dx >= x_overlap_limit:
                continue
            if dy >= y_overlap_limit:
                continue
            return {
                "idx": int(j),
                "bottom_z": float(bottom_z),
                "target_top_z": float(top_z),
                "z_gap": float(z_gap),
                "z_gap_limit": float(self.top_clearance_margin),
                "dx": float(dx),
                "dy": float(dy),
                "x_overlap_limit": float(x_overlap_limit),
                "y_overlap_limit": float(y_overlap_limit),
            }
        return None

    def _has_top_clearance(self, idx, active):
        return self._top_blocker_info(idx, active) is None

    def _front_blocker_info(self, idx, active):
        pi = self._get_position(idx)
        si = self.box_sizes[idx]
        x_min_i = pi[0] - si[0] / 2
        y_half = max(0.0, si[1] / 2 - self.front_clearance_margin)
        y_low_i = pi[1] - y_half
        y_high_i = pi[1] + y_half
        z_center_i = pi[2]
        for j in active:
            if j == idx:
                continue
            pj = self._get_position(j)
            sj = self.box_sizes[j]
            x_max_j = pj[0] + sj[0] / 2
            if x_max_j >= x_min_i:
                continue
            y_min_j = pj[1] - sj[1] / 2
            y_max_j = pj[1] + sj[1] / 2
            if y_max_j < y_low_i or y_min_j > y_high_i:
                continue
            z_gap = abs(pj[2] - z_center_i)
            z_limit = max(si[2], sj[2]) * 0.45
            if z_gap > z_limit:
                continue
            return {
                "idx": int(j),
                "x_max_j": float(x_max_j),
                "target_front_x": float(x_min_i),
                "y_min_j": float(y_min_j),
                "y_max_j": float(y_max_j),
                "target_y_low": float(y_low_i),
                "target_y_high": float(y_high_i),
                "z_gap": float(z_gap),
                "z_limit": float(z_limit),
            }
        return None

    def _has_front_clearance(self, idx, active):
        return self._front_blocker_info(idx, active) is None

    def _top_clearance_value(self, idx):
        pi = self._get_position(idx)
        si = self.box_sizes[idx]
        top_z = pi[2] + si[2] / 2
        clearance = self.height - top_z
        for j in np.flatnonzero(self.box_active):
            if j == idx:
                continue
            pj = self._get_position(j)
            sj = self.box_sizes[j]
            if abs(pi[0] - pj[0]) >= (si[0] + sj[0]) / 2 - 1e-4:
                continue
            if abs(pi[1] - pj[1]) >= (si[1] + sj[1]) / 2 - 1e-4:
                continue
            clearance = min(clearance, max(0.0, pj[2] - sj[2] / 2 - top_z))
        return float(max(0.0, clearance))

    def _front_clearance_value(self, idx):
        pi = self._get_position(idx)
        si = self.box_sizes[idx]
        front_face = pi[0] - si[0] / 2
        blocker_face = self.opening_x
        y_half = max(0.0, si[1] / 2 - self.front_clearance_margin)
        y_low_i = pi[1] - y_half
        y_high_i = pi[1] + y_half
        for j in np.flatnonzero(self.box_active):
            if j == idx:
                continue
            pj = self._get_position(j)
            sj = self.box_sizes[j]
            if pj[0] >= pi[0]:
                continue
            y_min_j = pj[1] - sj[1] / 2
            y_max_j = pj[1] + sj[1] / 2
            if y_max_j < y_low_i or y_min_j > y_high_i:
                continue
            if abs(pi[2] - pj[2]) >= max(si[2], sj[2]) * 0.45:
                continue
            blocker_face = max(blocker_face, pj[0] + sj[0] / 2)
        return float(max(0.0, front_face - blocker_face))

    def _support_count(self, idx):
        pi = self._get_position(idx)
        si = self.box_sizes[idx]
        top_z = pi[2] + si[2] / 2
        count = 0
        for j in np.flatnonzero(self.box_active):
            if j == idx:
                continue
            pj = self._get_position(j)
            sj = self.box_sizes[j]
            bottom_z = pj[2] - sj[2] / 2
            if abs(bottom_z - top_z) > max(0.04, 0.2 * self.box_size):
                continue
            if abs(pi[0] - pj[0]) < (si[0] + sj[0]) / 2 - 1e-4 and abs(pi[1] - pj[1]) < (si[1] + sj[1]) / 2 - 1e-4:
                count += 1
        return float(count)

    def _compute_drop_severity(self, x, y, z, sx, sy, sz):
        front_term = 1.0 - min(max(x / max(self.depth, 1e-6), 0.0), 1.0)
        height_term = 1.0 - min(max(z / max(self.height, 1e-6), 0.0), 1.0)
        size_term = min(1.0, abs(sx - self.box_size) / max(self.box_size, 1e-6))
        lateral_term = min(1.0, abs(y) / max(self.width / 2, 1e-6))
        severity = 0.35 * front_term + 0.3 * height_term + 0.2 * size_term + 0.15 * lateral_term
        return float(np.clip(severity, 0.0, 1.0))

    def _grid_location(self, idx):
        p = self._get_position(idx)
        col_x = int(np.clip(np.floor(p[0] / self.box_size), 0, self.n_x - 1))
        col_y = int(np.clip(np.floor((p[1] + self.width / 2) / self.box_size), 0, self.n_y - 1))
        layer_z = int(np.clip(np.floor(p[2] / self.box_size), 0, self.n_z - 1))
        return col_x + 1, col_y + 1, layer_z + 1

    def _get_state(self, indices, pickable_indices):
        n = len(indices)
        if n == 0:
            return np.zeros((0, _FEATURE_DIM), dtype=np.float32)
        pickable_set = set(int(i) for i in pickable_indices)
        state = np.zeros((n, _FEATURE_DIM), dtype=np.float32)
        for k, idx in enumerate(indices):
            pos = self._get_position(idx)
            size = self.box_sizes[idx]
            top_clear = self._top_clearance_value(idx) / max(self.height, 1e-6)
            front_clear = self._front_clearance_value(idx) / max(self.depth, 1e-6)
            support_load = self._support_count(idx) / max(self.n_z, 1)
            dx = dy = dz = 0.0
            if self.last_pick_position is not None:
                delta = pos - self.last_pick_position
                dx = delta[0] / max(self.depth, 1e-6)
                dy = delta[1] / max(self.width, 1e-6)
                dz = delta[2] / max(self.height, 1e-6)
            state[k, 0:3] = pos / np.array([max(self.depth, 1e-6), max(self.width / 2, 1e-6), max(self.height, 1e-6)], dtype=np.float32)
            state[k, 3:6] = size / max(self.box_size, 1e-6)
            state[k, 6] = 1.0 - pos[0] / max(self.depth, 1e-6)
            state[k, 7] = top_clear
            state[k, 8] = front_clear
            state[k, 9] = support_load
            state[k, 10] = float(self.box_is_dropped[idx])
            state[k, 11] = float(self.box_drop_severity[idx])
            state[k, 12] = float(idx in pickable_set)
            state[k, 13] = dx
            state[k, 14] = dy
            state[k, 15] = dz
        return state.astype(np.float32)

    def diagnose_pickability(self):
        active = np.flatnonzero(self.box_active)
        diagnostics = []
        for idx in active:
            top_info = self._top_blocker_info(idx, active)
            front_info = self._front_blocker_info(idx, active)
            top_clear = top_info is None
            front_clear = front_info is None
            pos = self._get_position(idx)
            row_x, col_y, layer_z = self._grid_location(idx)
            diagnostics.append(
                {
                    "idx": int(idx),
                    "top_clear": bool(top_clear),
                    "front_clear": bool(front_clear),
                    "is_pickable": bool(top_clear and front_clear),
                    "x": float(pos[0]),
                    "y": float(pos[1]),
                    "z": float(pos[2]),
                    "front_clearance": float(self._front_clearance_value(idx)),
                    "top_clearance": float(self._top_clearance_value(idx)),
                    "is_dropped": bool(self.box_is_dropped[idx]),
                    "row_x": int(row_x),
                    "col_y": int(col_y),
                    "layer_z": int(layer_z),
                    "top_blocker": top_info,
                    "front_blocker": front_info,
                }
            )
        diagnostics.sort(key=lambda item: item["x"])
        return diagnostics

    def _empty_info(self):
        return {
            "collapse": False,
            "same_target": False,
            "proximity": False,
            "cross_arm": False,
            "dual_pick_failed": 0.0,
            "invalid_picks": 0.0,
            "removed_count": 0.0,
            "dropped_cleared": 0.0,
            "dropped_severity_reward": 0.0,
            "avg_pick_x_penalty": 0.0,
            "motion_distance_penalty": 0.0,
            "xy_close_penalty": 0.0,
            "far_pair_penalty": 0.0,
            "cross_hand_penalty": 0.0,
            "free_space_reward": 0.0,
        }

    @property
    def n_active(self):
        return int(self.box_active.sum())

    def export_xml(self, output_path: str):
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(self._build_xml())


if __name__ == "__main__":
    env = ContainerUnpackEnv()
    sB, sP, mask, pickable = env.reset()
    print(f"Reset: {env.n_active} active, {len(pickable)} pickable")
    print(f"s_B shape: {sB.shape}, s_P shape: {sP.shape}")
