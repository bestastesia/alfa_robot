"""Render an honest selected-IK endpoint animation; this is not an RRT replay."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


WIDTH, HEIGHT = 1280, 760
BACKGROUND = (22, 27, 34)
PANEL = (31, 38, 48)
PANEL_ALT = (35, 43, 54)
TEXT = (235, 240, 246)
MUTED = (157, 170, 185)
GRID = (66, 78, 92)
BLUE = (56, 128, 214)
CORAL = (221, 99, 72)
GREEN = (73, 174, 122)
AMBER = (235, 168, 62)
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"
FONT_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FUNCTION_COLORS = {
    "v2b": (70, 137, 213), "limit_guard": (75, 177, 126), "updown_stable": (224, 164, 69),
    "pool_robust": (186, 112, 181), "home_bias": (73, 176, 181),
    "return_priority": (217, 105, 87), "balanced_combo": (151, 160, 171),
}


def font(size, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT, size)


def endpoint_rows(data, task):
    result = {}
    for row in data["rows"]:
        if row["instance"] != task or row["stage"] >= 3 or not row.get("reached"):
            continue
        entry = result.setdefault(row["function_id"], {"label": row["function_label"], "rows": {}})
        entry["rows"][row["stage"]] = row
    functions = [row["id"] for row in data["functions"]]
    if any(set(result.get(function, {}).get("rows", {})) != {0, 1, 2} for function in functions):
        raise ValueError(f"task {task} does not have three selected IKs for every function")
    return functions, result


def diagnostic_rows(path, task):
    with path.open(encoding="utf-8-sig") as stream:
        rows = [row for row in csv.DictReader(stream) if row["instance"] == task]
    return {row["function_id"]: row for row in rows}


def interpolate_schedule(frame, total):
    hold = max(4, total // 10)
    travel = (total - 3 * hold) // 2
    boundaries = (hold, hold + travel, 2 * hold + travel, 2 * (hold + travel))
    if frame < boundaries[0]:
        return 0, 0, 0.
    if frame < boundaries[1]:
        alpha = (frame - boundaries[0]) / max(boundaries[1] - boundaries[0] - 1, 1)
        return 0, 1, alpha
    if frame < boundaries[2]:
        return 1, 1, 0.
    if frame < boundaries[3]:
        alpha = (frame - boundaries[2]) / max(boundaries[3] - boundaries[2] - 1, 1)
        return 1, 2, alpha
    return 2, 2, 0.


def circular_lerp(a, b, alpha):
    delta = (b - a + 180.) % 360. - 180.
    return a + alpha * delta


def mix(a, b, alpha):
    return tuple(round((1. - alpha) * x + alpha * y) for x, y in zip(a, b))


def heat(value, index):
    normalized = np.clip((value + .5) * 2. if index == 0 else value / math.pi, -1., 1.)
    return mix((55, 64, 76), CORAL if normalized >= 0. else BLUE, .18 + .82 * abs(float(normalized)))


def line(draw, xy, fill, width=1):
    draw.line(xy, fill=fill, width=width)


def render_frame(data, diagnostics, functions, entries, task, frame, total):
    image = Image.new("RGB", (WIDTH, HEIGHT), BACKGROUND)
    draw = ImageDraw.Draw(image)
    draw.text((32, 20), "三 IK 端点顺畅度对比", font=font(28, True), fill=TEXT)
    draw.text((32, 58), f"代表任务 {task} · 7 种函数均完整成功", font=font(15), fill=MUTED)
    draw.rounded_rectangle((890, 22, 1248, 70), radius=5, fill=(55, 42, 29), outline=AMBER)
    draw.text((906, 35), "端点插值示意，不是 RRT 验收轨迹", font=font(14, True), fill=(250, 205, 128))

    start_x, end_x, timeline_y = 470, 1190, 98
    line(draw, (start_x, timeline_y, end_x, timeline_y), GRID, 4)
    labels = ("接触 IK", "回中/携箱 IK", "放置 IK")
    for index, label in enumerate(labels):
        x = start_x + index * (end_x - start_x) / 2
        draw.ellipse((x - 8, timeline_y - 8, x + 8, timeline_y + 8), fill=(86, 99, 115), outline=TEXT)
        box = draw.textbbox((0, 0), label, font=font(14))
        draw.text((x - (box[2] - box[0]) / 2, timeline_y - 31), label, font=font(14), fill=TEXT)
    source, target, alpha = interpolate_schedule(frame, total)
    progress = (source + alpha) / 2.
    playhead = start_x + progress * (end_x - start_x)
    draw.ellipse((playhead - 9, timeline_y - 9, playhead + 9, timeline_y + 9), fill=GREEN)
    transition = labels[source] if source == target else f"{labels[source]} → {labels[target]}"
    draw.text((32, 91), transition, font=font(17, True), fill=GREEN)

    headers = ["函数 / 顺畅分", "冗余参数", *(["U"] + [f"L{i}" for i in range(1, 8)] + [f"R{i}" for i in range(1, 8)])]
    draw.text((32, 125), headers[0], font=font(13), fill=MUTED)
    draw.text((260, 125), headers[1], font=font(13), fill=MUTED)
    cell_x, cell_w = 590, 41
    for index, label in enumerate(headers[2:]):
        draw.text((cell_x + index * cell_w + 10, 125), label, font=font(12), fill=MUTED)

    score_values = [float(diagnostics[f]["smoothness_score"]) for f in functions]
    min_score = min(score_values)
    for row_index, function in enumerate(functions):
        y = 148 + row_index * 76
        draw.rounded_rectangle((24, y, 1254, y + 64), radius=5,
                               fill=PANEL if row_index % 2 == 0 else PANEL_ALT)
        color = FUNCTION_COLORS[function]
        draw.rectangle((24, y, 31, y + 64), fill=color)
        entry = entries[function]
        rows = [entry["rows"][stage] for stage in range(3)]
        q = np.asarray([[row[f"q{i}"] for i in range(15)] for row in rows], dtype=float)
        current = q[source] if source == target else (1. - alpha) * q[source] + alpha * q[target]
        redundant = np.asarray([[row["updown_m"], row["left_swivel_deg"], row["right_swivel_deg"]]
                                for row in rows], dtype=float)
        if source == target:
            current_redundant = redundant[source]
        else:
            current_redundant = np.array([(1. - alpha) * redundant[source, 0] + alpha * redundant[target, 0],
                                          circular_lerp(redundant[source, 1], redundant[target, 1], alpha),
                                          circular_lerp(redundant[source, 2], redundant[target, 2], alpha)])
        reverse = (q[1] - q[0]) * (q[2] - q[1]) < 0.
        diagnostic = diagnostics[function]
        score = float(diagnostic["smoothness_score"])
        cost = float(diagnostic["smoothness_cost_s"])
        rank = int(diagnostic["smoothness_rank"])
        draw.text((42, y + 8), entry["label"], font=font(16, True), fill=TEXT)
        draw.text((42, y + 35), f"{score:5.2f} 分  排名 {rank}  代价 {cost:.3f}s", font=font(12), fill=color)
        bar_x, bar_y, bar_w = 205, y + 43, 40
        draw.rectangle((bar_x, bar_y, bar_x + bar_w, bar_y + 7), fill=GRID)
        fraction = (score - min_score) / max(100. - min_score, 1e-6)
        draw.rectangle((bar_x, bar_y, bar_x + bar_w * fraction, bar_y + 7), fill=color)
        draw.text((260, y + 8), f"Updown {current_redundant[0]:+.2f} m", font=font(13), fill=TEXT)
        draw.text((260, y + 34), f"左ψ {current_redundant[1]:+6.1f}°   右ψ {current_redundant[2]:+6.1f}°",
                  font=font(13), fill=MUTED)
        for joint in range(15):
            x = cell_x + joint * cell_w
            draw.rounded_rectangle((x, y + 8, x + 35, y + 55), radius=3, fill=heat(current[joint], joint),
                                   outline=AMBER if reverse[joint] else GRID, width=3 if reverse[joint] else 1)
            shown = f"{current[joint]:+.2f}" if joint == 0 else f"{math.degrees(current[joint]):+.0f}"
            box = draw.textbbox((0, 0), shown, font=font(10))
            draw.text((x + (35 - box[2] + box[0]) / 2, y + 24), shown, font=font(10), fill=TEXT)

    footer_y = 704
    draw.rectangle((24, footer_y, 1254, 744), fill=(27, 33, 41))
    draw.text((38, footer_y + 10), "色带：蓝=负角度，红=正角度；U 单位 m，其余关节显示角度；橙框=三个 IK 间发生方向反转",
              font=font(13), fill=MUTED)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--diagnostic-csv", type=Path, required=True)
    parser.add_argument("--task", default="L07-R01-dev-v0-S30x40x40")
    parser.add_argument("--frames", type=int, default=70)
    parser.add_argument("--duration-ms", type=int, default=100)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.frames < 20 or args.duration_ms < 20:
        raise ValueError("animation requires at least 20 frames and 20 ms duration")
    data = json.loads(args.data.read_text())
    functions, entries = endpoint_rows(data, args.task)
    diagnostics = diagnostic_rows(args.diagnostic_csv, args.task)
    if set(diagnostics) != set(functions):
        raise ValueError("diagnostic rows do not cover every function")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    frames = [render_frame(data, diagnostics, functions, entries, args.task, index, args.frames)
              for index in range(args.frames)]
    poster = frames[len(frames) // 2]
    poster.save(args.output_dir / "selected_ik_endpoint_comparison.png")
    palette = [frame.quantize(colors=128, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
               for frame in frames]
    palette[0].save(args.output_dir / "selected_ik_endpoint_comparison.gif", save_all=True,
                    append_images=palette[1:], duration=args.duration_ms, loop=0, disposal=2, optimize=False)
    gif_path = args.output_dir / "selected_ik_endpoint_comparison.gif"
    with Image.open(gif_path) as encoded:
        encoded_durations = []
        for index in range(encoded.n_frames):
            encoded.seek(index)
            encoded_durations.append(int(encoded.info.get("duration", 0)))
        encoded_frames = encoded.n_frames
    metadata = {
        "schema": "motion386-selected-ik-animation-v1",
        "task": args.task,
        "source_frames": args.frames,
        "encoded_frames": encoded_frames,
        "encoded_duration_ms": sum(encoded_durations),
        "source_frame_duration_ms": args.duration_ms,
        "functions": functions,
        "scope": "linear interpolation between selected contact/carry/placement IK endpoints",
        "not_claimed": ["RRT trajectory", "collision-validated interpolation", "B2 candidate-group selection"],
    }
    (args.output_dir / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(metadata, ensure_ascii=False))


if __name__ == "__main__":
    main()
