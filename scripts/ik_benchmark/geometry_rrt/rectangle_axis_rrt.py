#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import random
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class State:
    x: float
    y: float
    r: float


@dataclass
class Node:
    state: State
    parent: int
    cost: float


@dataclass(frozen=True)
class Aabb:
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass
class PlanResult:
    success: bool
    nodes: list[Node]
    path: list[State]
    elapsed_ms: float
    iterations: int
    collision_checks: int
    reason: str
    random_rejections: int = 0


class PoseRejectField:
    """Deterministic sparse validity field over discretized (x, y, r) poses.

    This models "a pose has P probability of being rejected" instead of
    "an expansion attempt has P probability of being rejected". Once a pose
    cell is rejected, every future check of that cell is rejected too.
    """

    def __init__(self, rate: float, cell_xy: float, cell_r: float, seed: int) -> None:
        self.rate = max(0.0, min(1.0, rate))
        self.cell_xy = cell_xy
        self.cell_r = cell_r
        self.seed = seed
        self._cache: dict[tuple[int, int, int], bool] = {}
        self._forced_valid: set[tuple[int, int, int]] = set()
        if self.cell_xy <= 0.0:
            raise ValueError("reject cell xy must be positive")
        if self.cell_r <= 0.0:
            raise ValueError("reject cell r must be positive")

    def enabled(self) -> bool:
        return self.rate > 0.0

    def key(self, state: State) -> tuple[int, int, int]:
        return (
            math.floor(state.x / self.cell_xy),
            math.floor(state.y / self.cell_xy),
            math.floor((angle_wrap(state.r) + math.pi) / self.cell_r),
        )

    def force_valid(self, state: State) -> None:
        self._forced_valid.add(self.key(state))

    def rejected(self, state: State) -> bool:
        if self.rate <= 0.0:
            return False
        key = self.key(state)
        if key in self._forced_valid:
            return False
        if key not in self._cache:
            self._cache[key] = self._hash01(key) < self.rate
        return self._cache[key]

    def _hash01(self, key: tuple[int, int, int]) -> float:
        # SplitMix64-style deterministic hash; avoids Python's salted hash().
        kx, ky, kr = key
        mask = (1 << 64) - 1
        value = (
            (kx * 0x9E3779B185EBCA87)
            ^ (ky * 0xC2B2AE3D27D4EB4F)
            ^ (kr * 0x165667B19E3779F9)
            ^ (self.seed * 0xD6E8FEB86659FD93)
        ) & mask
        value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9 & mask
        value = (value ^ (value >> 27)) * 0x94D049BB133111EB & mask
        value = value ^ (value >> 31)
        return ((value >> 11) & ((1 << 53) - 1)) / float(1 << 53)


def angle_wrap(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def angle_diff(a: float, b: float) -> float:
    return angle_wrap(a - b)


def rect_corners(state: State, width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    c = math.cos(state.r)
    s = math.sin(state.r)
    local = [(-half_w, -half_h), (half_w, -half_h), (half_w, half_h), (-half_w, half_h)]
    return [(state.x + c * lx - s * ly, state.y + s * lx + c * ly) for lx, ly in local]


def point_in_aabb(point: tuple[float, float], box: Aabb) -> bool:
    x, y = point
    return box.min_x <= x <= box.max_x and box.min_y <= y <= box.max_y


def segments_intersect(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float], d: tuple[float, float]) -> bool:
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    def on_segment(p, q, r):
        return min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and min(p[1], r[1]) <= q[1] <= max(p[1], r[1])

    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)
    if o1 == 0 and on_segment(a, c, b):
        return True
    if o2 == 0 and on_segment(a, d, b):
        return True
    if o3 == 0 and on_segment(c, a, d):
        return True
    if o4 == 0 and on_segment(c, b, d):
        return True
    return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)


def polygon_intersects_aabb(points: list[tuple[float, float]], box: Aabb) -> bool:
    if any(point_in_aabb(point, box) for point in points):
        return True
    box_points = [(box.min_x, box.min_y), (box.max_x, box.min_y), (box.max_x, box.max_y), (box.min_x, box.max_y)]
    if any(point_in_aabb(point, Aabb(min(x for x, _ in points), max(x for x, _ in points), min(y for _, y in points), max(y for _, y in points))) for point in box_points):
        # The AABB corner can be in the polygon bounding box but outside the polygon. Keep this only as broad phase.
        pass
    poly_edges = list(zip(points, points[1:] + points[:1]))
    box_edges = list(zip(box_points, box_points[1:] + box_points[:1]))
    for edge_a in poly_edges:
        for edge_b in box_edges:
            if segments_intersect(edge_a[0], edge_a[1], edge_b[0], edge_b[1]):
                return True
    # Convex rectangle contains an AABB corner. Use winding sign.
    def inside_convex(point):
        signs = []
        for p, q in poly_edges:
            signs.append((q[0] - p[0]) * (point[1] - p[1]) - (q[1] - p[1]) * (point[0] - p[0]))
        return all(v >= 0 for v in signs) or all(v <= 0 for v in signs)
    return any(inside_convex(point) for point in box_points)


def axis_collision_free(state: State, width: float, height: float, clearance: float = 0.0, obstacles: list[Aabb] | None = None) -> bool:
    corners = rect_corners(state, width, height)
    if not all(x >= clearance and y >= clearance for x, y in corners):
        return False
    for obstacle in obstacles or []:
        if polygon_intersects_aabb(corners, obstacle):
            return False
    return True


def state_collision_free(
    state: State,
    width: float,
    height: float,
    clearance: float = 0.0,
    obstacles: list[Aabb] | None = None,
    reject_field: PoseRejectField | None = None,
) -> tuple[bool, bool]:
    if not axis_collision_free(state, width, height, clearance, obstacles):
        return False, False
    if reject_field is not None and reject_field.rejected(state):
        return False, True
    return True, False


def state_distance(lhs: State, rhs: State, r_weight: float) -> float:
    return math.hypot(lhs.x - rhs.x, lhs.y - rhs.y) + r_weight * abs(angle_diff(lhs.r, rhs.r))


def interpolate(lhs: State, rhs: State, ratio: float) -> State:
    ratio = max(0.0, min(1.0, ratio))
    return State(
        lhs.x + (rhs.x - lhs.x) * ratio,
        lhs.y + (rhs.y - lhs.y) * ratio,
        angle_wrap(lhs.r + angle_diff(rhs.r, lhs.r) * ratio),
    )


def steer(src: State, dst: State, step_xy: float, step_r: float) -> State:
    dx = dst.x - src.x
    dy = dst.y - src.y
    dist_xy = math.hypot(dx, dy)
    if dist_xy > step_xy and dist_xy > 1e-12:
        scale = step_xy / dist_xy
        x = src.x + dx * scale
        y = src.y + dy * scale
    else:
        x = dst.x
        y = dst.y
    dr = angle_diff(dst.r, src.r)
    if abs(dr) > step_r:
        dr = math.copysign(step_r, dr)
    return State(x, y, angle_wrap(src.r + dr))


def edge_collision_free(
    a: State,
    b: State,
    width: float,
    height: float,
    edge_resolution: float,
    r_resolution: float,
    clearance: float,
    obstacles: list[Aabb] | None = None,
    reject_field: PoseRejectField | None = None,
) -> tuple[bool, int, int]:
    dist = math.hypot(b.x - a.x, b.y - a.y)
    rot = abs(angle_diff(b.r, a.r))
    samples = max(2, int(math.ceil(max(dist / edge_resolution, rot / r_resolution))))
    checks = 0
    random_rejections = 0
    for index in range(samples + 1):
        checks += 1
        ok, rejected = state_collision_free(
            interpolate(a, b, index / samples),
            width,
            height,
            clearance,
            obstacles,
            reject_field,
        )
        if not ok:
            if rejected:
                random_rejections += 1
            return False, checks, random_rejections
    return True, checks, random_rejections


def reconstruct(nodes: list[Node], index: int) -> list[State]:
    path: list[State] = []
    while index >= 0:
        path.append(nodes[index].state)
        index = nodes[index].parent
    path.reverse()
    return path


def path_smooth(
    path: list[State],
    width: float,
    height: float,
    edge_resolution: float,
    r_resolution: float,
    clearance: float,
    attempts: int,
    rng: random.Random,
    obstacles: list[Aabb] | None = None,
    reject_field: PoseRejectField | None = None,
) -> tuple[list[State], int, int]:
    if len(path) <= 2:
        return path, 0, 0
    checks = 0
    random_rejections = 0
    path = list(path)
    for _ in range(attempts):
        if len(path) <= 2:
            break
        i = rng.randrange(0, len(path) - 2)
        j = rng.randrange(i + 2, len(path))
        ok, used, rejected = edge_collision_free(
            path[i],
            path[j],
            width,
            height,
            edge_resolution,
            r_resolution,
            clearance,
            obstacles,
            reject_field,
        )
        checks += used
        random_rejections += rejected
        if ok:
            path = path[: i + 1] + path[j:]
    return path, checks, random_rejections


def plan_rrt(
    start: State,
    goal: State,
    width: float,
    height: float,
    bounds: tuple[float, float, float, float],
    max_iterations: int,
    step_xy: float,
    step_r: float,
    edge_resolution: float,
    r_resolution: float,
    goal_tolerance_xy: float,
    goal_tolerance_r: float,
    goal_sample_rate: float,
    r_weight: float,
    clearance: float,
    smooth_attempts: int,
    seed: int,
    obstacles: list[Aabb] | None = None,
    reject_field: PoseRejectField | None = None,
) -> PlanResult:
    rng = random.Random(seed)
    start_time = time.perf_counter()
    collision_checks = 0
    random_rejections = 0
    nodes = [Node(start, -1, 0.0)]
    if reject_field is not None:
        reject_field.force_valid(start)
        reject_field.force_valid(goal)
    start_ok, start_rejected = state_collision_free(start, width, height, clearance, obstacles, reject_field)
    if start_rejected:
        random_rejections += 1
    if not start_ok:
        return PlanResult(False, nodes, [], 0.0, 0, 1, "start_in_collision", random_rejections)
    goal_ok, goal_rejected = state_collision_free(goal, width, height, clearance, obstacles, reject_field)
    if goal_rejected:
        random_rejections += 1
    if not goal_ok:
        return PlanResult(False, nodes, [], 0.0, 0, 2, "goal_in_collision", random_rejections)

    min_x, max_x, min_y, max_y = bounds
    for iteration in range(1, max_iterations + 1):
        sample = goal if rng.random() < goal_sample_rate else State(
            rng.uniform(min_x, max_x), rng.uniform(min_y, max_y), rng.uniform(-math.pi, math.pi)
        )
        nearest_index = min(range(len(nodes)), key=lambda i: state_distance(nodes[i].state, sample, r_weight))
        nearest = nodes[nearest_index].state
        new_state = steer(nearest, sample, step_xy, step_r)
        ok, used_checks, used_rejections = edge_collision_free(
            nearest,
            new_state,
            width,
            height,
            edge_resolution,
            r_resolution,
            clearance,
            obstacles,
            reject_field,
        )
        collision_checks += used_checks
        random_rejections += used_rejections
        if not ok:
            continue
        edge_cost = state_distance(nearest, new_state, r_weight)
        nodes.append(Node(new_state, nearest_index, nodes[nearest_index].cost + edge_cost))
        new_index = len(nodes) - 1
        if (math.hypot(new_state.x - goal.x, new_state.y - goal.y) <= goal_tolerance_xy and
                abs(angle_diff(new_state.r, goal.r)) <= goal_tolerance_r):
            ok_goal, used_goal_checks, used_goal_rejections = edge_collision_free(
                new_state,
                goal,
                width,
                height,
                edge_resolution,
                r_resolution,
                clearance,
                obstacles,
                reject_field,
            )
            collision_checks += used_goal_checks
            random_rejections += used_goal_rejections
            if ok_goal:
                nodes.append(Node(goal, new_index, nodes[new_index].cost + state_distance(new_state, goal, r_weight)))
                path = reconstruct(nodes, len(nodes) - 1)
                path, smooth_checks, smooth_rejections = path_smooth(
                    path,
                    width,
                    height,
                    edge_resolution,
                    r_resolution,
                    clearance,
                    smooth_attempts,
                    rng,
                    obstacles,
                    reject_field,
                )
                collision_checks += smooth_checks
                random_rejections += smooth_rejections
                return PlanResult(True, nodes, path, (time.perf_counter() - start_time) * 1000.0, iteration, collision_checks, "", random_rejections)
    return PlanResult(False, nodes, [], (time.perf_counter() - start_time) * 1000.0, max_iterations, collision_checks, "max_iterations", random_rejections)


def densify_path(path: list[State], step_xy: float, step_r: float) -> list[State]:
    if not path:
        return []
    dense = [path[0]]
    for a, b in zip(path, path[1:]):
        dist = math.hypot(b.x - a.x, b.y - a.y)
        rot = abs(angle_diff(b.r, a.r))
        count = max(1, int(math.ceil(max(dist / step_xy, rot / step_r))))
        for index in range(1, count + 1):
            dense.append(interpolate(a, b, index / count))
    return dense


class SvgMapper:
    def __init__(self, bounds: tuple[float, float, float, float], pixel: int = 900, margin: int = 60) -> None:
        self.min_x, self.max_x, self.min_y, self.max_y = bounds
        self.pixel = pixel
        self.margin = margin
        self.scale = min(
            (pixel - 2 * margin) / (self.max_x - self.min_x),
            (pixel - 2 * margin) / (self.max_y - self.min_y),
        )

    def p(self, x: float, y: float) -> tuple[float, float]:
        px = self.margin + (x - self.min_x) * self.scale
        py = self.pixel - self.margin - (y - self.min_y) * self.scale
        return px, py

    def points_attr(self, points: list[tuple[float, float]]) -> str:
        return " ".join(f"{x:.2f},{y:.2f}" for x, y in (self.p(px, py) for px, py in points))


def svg_poly(mapper: SvgMapper, points: list[tuple[float, float]], stroke: str, fill: str = "none", opacity: float = 1.0, width: float = 1.0) -> str:
    return f'<polygon points="{mapper.points_attr(points)}" fill="{fill}" stroke="{stroke}" stroke-width="{width}" opacity="{opacity}" />'


def svg_line(mapper: SvgMapper, a: tuple[float, float], b: tuple[float, float], stroke: str, width: float = 1.0, opacity: float = 1.0) -> str:
    x1, y1 = mapper.p(*a)
    x2, y2 = mapper.p(*b)
    return f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{stroke}" stroke-width="{width}" opacity="{opacity}" />'


def write_svg(
    result: PlanResult,
    start: State,
    goal: State,
    width: float,
    height: float,
    bounds: tuple[float, float, float, float],
    title: str,
    output: Path,
    obstacles: list[Aabb] | None = None,
    reject_field: PoseRejectField | None = None,
    reject_slice_r: float | None = None,
) -> None:
    mapper = SvgMapper(bounds)
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{mapper.pixel}" height="{mapper.pixel}" viewBox="0 0 {mapper.pixel} {mapper.pixel}">',
        '<rect width="100%" height="100%" fill="white" />',
        f'<text x="20" y="28" font-family="monospace" font-size="16">{title}</text>',
    ]
    min_x, max_x, min_y, max_y = bounds
    parts.append(svg_poly(mapper, [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)], "#bbbbbb", width=1.0))
    parts.append(svg_line(mapper, (0, min_y), (0, max_y), "#cc2222", width=4.0))
    parts.append(svg_line(mapper, (min_x, 0), (max_x, 0), "#cc2222", width=4.0))
    if reject_field is not None and reject_field.enabled():
        slice_r = reject_slice_r if reject_slice_r is not None else goal.r
        cell = reject_field.cell_xy
        x = math.floor(min_x / cell) * cell
        while x <= max_x:
            y = math.floor(min_y / cell) * cell
            while y <= max_y:
                center = State(x + cell * 0.5, y + cell * 0.5, slice_r)
                if reject_field.rejected(center):
                    px, py = mapper.p(x, y + cell)
                    size = cell * mapper.scale
                    parts.append(f'<rect x="{px:.2f}" y="{py:.2f}" width="{size:.2f}" height="{size:.2f}" fill="#ff3355" opacity="0.13" />')
                y += cell
            x += cell
    for obstacle in obstacles or []:
        parts.append(svg_poly(mapper, [(obstacle.min_x, obstacle.min_y), (obstacle.max_x, obstacle.min_y), (obstacle.max_x, obstacle.max_y), (obstacle.min_x, obstacle.max_y)], "#7722cc", fill="#aa66dd", opacity=0.35, width=2.0))
    step = max(1, len(result.nodes) // 800)
    for node in result.nodes[1::step]:
        parent = result.nodes[node.parent]
        parts.append(svg_line(mapper, (parent.state.x, parent.state.y), (node.state.x, node.state.y), "#d0d0d0", width=0.6, opacity=0.45))
    if result.path:
        path_points = [(state.x, state.y) for state in result.path]
        points = " ".join(f"{x:.2f},{y:.2f}" for x, y in (mapper.p(px, py) for px, py in path_points))
        parts.append(f'<polyline points="{points}" fill="none" stroke="#0066ff" stroke-width="3" />')
        for state in result.path:
            parts.append(svg_poly(mapper, rect_corners(state, width, height), "#0066ff", opacity=0.28, width=1.0))
    parts.append(svg_poly(mapper, rect_corners(start, width, height), "#00aa44", opacity=1.0, width=3.0))
    parts.append(svg_poly(mapper, rect_corners(goal, width, height), "#ff8800", opacity=1.0, width=3.0))
    parts.append('</svg>')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(parts))


def write_html_animation(
    path: list[State],
    start: State,
    goal: State,
    width: float,
    height: float,
    bounds: tuple[float, float, float, float],
    title: str,
    output: Path,
    reject_field: PoseRejectField | None = None,
    reject_slice_r: float | None = None,
) -> None:
    dense = densify_path(path, 1.0, math.radians(4.0))
    mapper = SvgMapper(bounds)
    frames = [rect_corners(state, width, height) for state in dense]
    frame_points = [mapper.points_attr(frame) for frame in frames]
    min_x, max_x, min_y, max_y = bounds
    start_poly = mapper.points_attr(rect_corners(start, width, height))
    goal_poly = mapper.points_attr(rect_corners(goal, width, height))
    path_points = " ".join(f"{x:.2f},{y:.2f}" for x, y in (mapper.p(s.x, s.y) for s in dense))
    reject_rects = []
    if reject_field is not None and reject_field.enabled():
        slice_r = reject_slice_r if reject_slice_r is not None else goal.r
        cell = reject_field.cell_xy
        x = math.floor(min_x / cell) * cell
        while x <= max_x:
            y = math.floor(min_y / cell) * cell
            while y <= max_y:
                center = State(x + cell * 0.5, y + cell * 0.5, slice_r)
                if reject_field.rejected(center):
                    px, py = mapper.p(x, y + cell)
                    size = cell * mapper.scale
                    reject_rects.append(f"<rect x='{px:.2f}' y='{py:.2f}' width='{size:.2f}' height='{size:.2f}' fill='#ff3355' opacity='0.13'/>")
                y += cell
            x += cell
    reject_svg = "\n  ".join(reject_rects)
    html = f"""<!doctype html>
<html><head><meta charset='utf-8'><title>{title}</title></head>
<body style='font-family: sans-serif'>
<h3>{title}</h3>
<svg width='{mapper.pixel}' height='{mapper.pixel}' viewBox='0 0 {mapper.pixel} {mapper.pixel}' style='border:1px solid #ddd'>
  <rect width='100%' height='100%' fill='white'/>
  {svg_poly(mapper, [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)], '#bbbbbb')}
  {svg_line(mapper, (0, min_y), (0, max_y), '#cc2222', 4.0)}
  {svg_line(mapper, (min_x, 0), (max_x, 0), '#cc2222', 4.0)}
  {reject_svg}
  <polyline points='{path_points}' fill='none' stroke='#0066ff' stroke-width='3' opacity='0.5'/>
  <polygon points='{start_poly}' fill='none' stroke='#00aa44' stroke-width='3'/>
  <polygon points='{goal_poly}' fill='none' stroke='#ff8800' stroke-width='3'/>
  <polygon id='moving_rect' points='{frame_points[0] if frame_points else start_poly}' fill='#4488ff' fill-opacity='0.35' stroke='#0044cc' stroke-width='2'/>
</svg>
<p id='frame'></p>
<script>
const frames = {json.dumps(frame_points)};
let i = 0;
function tick() {{
  if (frames.length > 0) {{
    document.getElementById('moving_rect').setAttribute('points', frames[i]);
    document.getElementById('frame').innerText = `frame ${{i+1}}/${{frames.length}}`;
    i = (i + 1) % frames.length;
  }}
}}
setInterval(tick, 40);
tick();
</script>
</body></html>"""
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html)


def parse_angles(value: str) -> list[float]:
    return [math.radians(float(part.strip())) for part in value.split(",") if part.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description="RRT for a planar rectangle moving in (x,y,r) while avoiding x/y axes")
    parser.add_argument("--start-x", type=float, default=15.0)
    parser.add_argument("--start-y", type=float, default=20.0)
    parser.add_argument("--goal-x", type=float, default=35.0)
    parser.add_argument("--goal-y", type=float, default=45.0)
    parser.add_argument("--width", type=float, default=30.0)
    parser.add_argument("--height", type=float, default=40.0)
    parser.add_argument("--start-r-deg", type=float, default=0.0)
    parser.add_argument("--goal-r-deg", default="0,15,30,-20")
    parser.add_argument("--bounds", default="0,80,0,90", help="min_x,max_x,min_y,max_y")
    parser.add_argument("--max-iterations", type=int, default=8000)
    parser.add_argument("--step-xy", type=float, default=4.0)
    parser.add_argument("--step-r-deg", type=float, default=8.0)
    parser.add_argument("--edge-resolution", type=float, default=1.0)
    parser.add_argument("--r-resolution-deg", type=float, default=2.0)
    parser.add_argument("--goal-tolerance-xy", type=float, default=4.0)
    parser.add_argument("--goal-tolerance-r-deg", type=float, default=8.0)
    parser.add_argument("--goal-sample-rate", type=float, default=0.18)
    parser.add_argument("--r-weight", type=float, default=8.0)
    parser.add_argument("--clearance", type=float, default=0.0)
    parser.add_argument("--smooth-attempts", type=int, default=200)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--random-reject-rate", type=float, default=0.0, help="姿态格子的持久随机拒绝概率，例如0.3；同一(x,y,r)格子每次检查结果一致")
    parser.add_argument("--reject-cell-xy", type=float, default=2.0, help="随机姿态拒绝场的xy格子尺寸")
    parser.add_argument("--reject-cell-r-deg", type=float, default=8.0, help="随机姿态拒绝场的r格子尺寸")
    parser.add_argument("--output-dir", type=Path, default=Path("data/geometry_rrt/rectangle_axis_rrt"))
    parser.add_argument("--obstacle", action="append", default=[], help="额外AABB障碍，格式 min_x,max_x,min_y,max_y；可重复")
    args = parser.parse_args()

    bounds_values = [float(part.strip()) for part in args.bounds.split(",")]
    if len(bounds_values) != 4:
        raise ValueError("--bounds must be min_x,max_x,min_y,max_y")
    bounds = (bounds_values[0], bounds_values[1], bounds_values[2], bounds_values[3])
    start = State(args.start_x, args.start_y, math.radians(args.start_r_deg))
    obstacles: list[Aabb] = []
    for item in args.obstacle:
        values = [float(part.strip()) for part in item.split(",")]
        if len(values) != 4:
            raise ValueError("--obstacle must be min_x,max_x,min_y,max_y")
        obstacles.append(Aabb(values[0], values[1], values[2], values[3]))
    goal_angles = parse_angles(args.goal_r_deg)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    summary = []
    for index, goal_r in enumerate(goal_angles, start=1):
        goal = State(args.goal_x, args.goal_y, goal_r)
        reject_field = PoseRejectField(
            rate=args.random_reject_rate,
            cell_xy=args.reject_cell_xy,
            cell_r=math.radians(args.reject_cell_r_deg),
            seed=args.seed * 1009 + index,
        )
        result = plan_rrt(
            start=start,
            goal=goal,
            width=args.width,
            height=args.height,
            bounds=bounds,
            max_iterations=args.max_iterations,
            step_xy=args.step_xy,
            step_r=math.radians(args.step_r_deg),
            edge_resolution=args.edge_resolution,
            r_resolution=math.radians(args.r_resolution_deg),
            goal_tolerance_xy=args.goal_tolerance_xy,
            goal_tolerance_r=math.radians(args.goal_tolerance_r_deg),
            goal_sample_rate=args.goal_sample_rate,
            r_weight=args.r_weight,
            clearance=args.clearance,
            smooth_attempts=args.smooth_attempts,
            seed=args.seed + index,
            obstacles=obstacles,
            reject_field=reject_field,
        )
        path_length = 0.0
        rotation_total = 0.0
        for a, b in zip(result.path, result.path[1:]):
            path_length += math.hypot(b.x - a.x, b.y - a.y)
            rotation_total += abs(angle_diff(b.r, a.r))
        title = f"goal r={math.degrees(goal_r):.1f}deg success={result.success} iter={result.iterations} checks={result.collision_checks} {result.elapsed_ms:.2f}ms"
        svg_path = args.output_dir / f"case_{index:02d}_r{int(round(math.degrees(goal_r))):+d}.svg"
        html_path = args.output_dir / f"case_{index:02d}_r{int(round(math.degrees(goal_r))):+d}.html"
        write_svg(result, start, goal, args.width, args.height, bounds, title, svg_path, obstacles, reject_field, goal.r)
        if result.success:
            write_html_animation(result.path, start, goal, args.width, args.height, bounds, title, html_path, reject_field, goal.r)
        summary.append({
            "case": index,
            "goal_r_deg": math.degrees(goal_r),
            "success": result.success,
            "reason": result.reason,
            "elapsed_ms": result.elapsed_ms,
            "iterations": result.iterations,
            "node_count": len(result.nodes),
            "path_waypoint_count": len(result.path),
            "path_xy_length": path_length,
            "path_rotation_deg": math.degrees(rotation_total),
            "collision_checks": result.collision_checks,
            "random_rejections": result.random_rejections,
            "svg": str(svg_path),
            "html": str(html_path) if result.success else "",
            "path": [state.__dict__ for state in result.path],
        })
        print(f"case {index}: r={math.degrees(goal_r):.1f} success={result.success} time={result.elapsed_ms:.2f}ms iter={result.iterations} nodes={len(result.nodes)} checks={result.collision_checks} rejects={result.random_rejections} svg={svg_path}")
    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"summary={summary_path}")
    return 0 if all(item["success"] for item in summary) else 1


if __name__ == "__main__":
    raise SystemExit(main())
