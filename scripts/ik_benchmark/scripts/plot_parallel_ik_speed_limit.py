#!/usr/bin/env python3
"""Plot/tabulate parallel IK speed-limit benchmark JSONL files."""

from __future__ import annotations

import argparse
import csv
import contextlib
import io
import json
from pathlib import Path

try:
    with contextlib.redirect_stderr(io.StringIO()):
        import matplotlib.pyplot as plt
    HAS_MPL = True
except Exception as exc:  # pragma: no cover
    HAS_MPL = False
    MPL_ERROR = exc

VARIANT_LABELS = {
    "serial_50ms": "Control 1: Serial IK / 50ms",
    "parallel8_50ms": "Control 2: Parallel Wide IK / 50ms",
    "parallel8_10ms": "Experiment: Parallel Narrow IK / 10ms",
}

VARIANT_LABELS_ZH = {
    "serial_50ms": "对照组1：串行IK / 50ms",
    "parallel8_50ms": "对照组2：并行宽时域IK / 50ms",
    "parallel8_10ms": "实验组：并行窄时域IK / 10ms",
}


def read_jsonl(path: Path) -> list[dict]:
    rows: list[dict] = []
    with path.open() as file:
        for line in file:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def load_variant(path: Path, variant_id: str) -> tuple[list[dict], dict]:
    rows = read_jsonl(path)
    stages = [row for row in rows if row.get("type") == "stage"]
    summaries = [row for row in rows if row.get("type") == "summary"]
    if not summaries:
        raise ValueError(f"no summary record in {path}")
    summary = summaries[-1]
    for index, stage in enumerate(stages, start=1):
        stage["variant_id"] = variant_id
        stage["variant_label"] = VARIANT_LABELS.get(variant_id, variant_id)
        stage["variant_label_zh"] = VARIANT_LABELS_ZH.get(variant_id, variant_id)
        stage["stage_no"] = index
    summary["variant_id"] = variant_id
    summary["variant_label"] = VARIANT_LABELS.get(variant_id, variant_id)
    summary["variant_label_zh"] = VARIANT_LABELS_ZH.get(variant_id, variant_id)
    return stages, summary


def write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def plot_grouped_bars(stage_rows: list[dict], output: Path, value_key: str, ylabel: str, title: str) -> None:
    variants = list(VARIANT_LABELS.keys())
    by_variant = {variant: [row for row in stage_rows if row["variant_id"] == variant] for variant in variants}
    stage_count = max((len(rows) for rows in by_variant.values()), default=0)
    x = list(range(1, stage_count + 1))
    width = 0.25
    fig, ax = plt.subplots(figsize=(15, 6))
    offsets = [-width, 0.0, width]
    for offset, variant in zip(offsets, variants):
        values = [float(row.get(value_key) or 0.0) for row in by_variant.get(variant, [])]
        ax.bar([i + offset for i in x[:len(values)]], values, width=width, label=VARIANT_LABELS[variant])
    ax.set_title(title)
    ax.set_xlabel("阶段序号")
    ax.set_ylabel(ylabel)
    ax.set_xticks(x)
    ax.grid(axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output.with_suffix(".png"), dpi=160)
    fig.savefig(output.with_suffix(".svg"))
    plt.close(fig)


def plot_summary(summary_rows: list[dict], output: Path) -> None:
    labels = [row["variant_label"] for row in summary_rows]
    wall = [float(row.get("wall_ms") or 0.0) / 1000.0 for row in summary_rows]
    trials_per_sec = [float(row.get("trials_per_sec") or 0.0) for row in summary_rows]
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    axes[0].bar(labels, wall, color=["#1f77b4", "#ff7f0e", "#2ca02c"])
    axes[0].set_title("15阶段总 wall 耗时")
    axes[0].set_ylabel("秒")
    axes[0].tick_params(axis="x", rotation=15)
    axes[0].grid(axis="y", alpha=0.3)
    axes[1].bar(labels, trials_per_sec, color=["#1f77b4", "#ff7f0e", "#2ca02c"])
    axes[1].set_title("吞吐量")
    axes[1].set_ylabel("trials / second")
    axes[1].tick_params(axis="x", rotation=15)
    axes[1].grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output.with_suffix(".png"), dpi=160)
    fig.savefig(output.with_suffix(".svg"))
    plt.close(fig)


def svg_escape(text: object) -> str:
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def write_simple_svg_bar(path: Path, labels: list[str], values: list[float], title: str, ylabel: str) -> None:
    width, height = 920, 520
    left, top, bottom = 100, 70, 90
    chart_w = width - left - 40
    chart_h = height - top - bottom
    max_value = max(values) if values else 1.0
    if max_value <= 0:
        max_value = 1.0
    bar_gap = 28
    bar_w = max(30.0, (chart_w - bar_gap * (len(values) + 1)) / max(1, len(values)))
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd", "#d62728"]
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2}" y="32" text-anchor="middle" font-size="22" font-family="sans-serif">{svg_escape(title)}</text>',
        f'<text x="24" y="{top + chart_h/2}" text-anchor="middle" transform="rotate(-90 24 {top + chart_h/2})" font-size="14" font-family="sans-serif">{svg_escape(ylabel)}</text>',
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top+chart_h}" stroke="#333"/>',
        f'<line x1="{left}" y1="{top+chart_h}" x2="{left+chart_w}" y2="{top+chart_h}" stroke="#333"/>',
    ]
    for tick in range(6):
        value = max_value * tick / 5.0
        y = top + chart_h - chart_h * tick / 5.0
        parts.append(f'<line x1="{left}" y1="{y:.1f}" x2="{left+chart_w}" y2="{y:.1f}" stroke="#ddd"/>')
        parts.append(f'<text x="{left-8}" y="{y+4:.1f}" text-anchor="end" font-size="11" font-family="sans-serif">{value:.1f}</text>')
    for index, (label, value) in enumerate(zip(labels, values)):
        x = left + bar_gap + index * (bar_w + bar_gap)
        bar_h = chart_h * value / max_value
        y = top + chart_h - bar_h
        parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w:.1f}" height="{bar_h:.1f}" fill="{colors[index % len(colors)]}"/>')
        parts.append(f'<text x="{x+bar_w/2:.1f}" y="{y-6:.1f}" text-anchor="middle" font-size="12" font-family="sans-serif">{value:.2f}</text>')
        parts.append(f'<text x="{x+bar_w/2:.1f}" y="{top+chart_h+22}" text-anchor="middle" font-size="12" font-family="sans-serif">{svg_escape(label)}</text>')
    parts.append('</svg>')
    path.write_text("\n".join(parts) + "\n")


def write_fallback_svgs(output_dir: Path, summaries: list[dict]) -> None:
    labels = [row["variant_label"] for row in summaries]
    write_simple_svg_bar(
        output_dir / "04a_total_wall_time_dependency_free.svg",
        labels,
        [float(row.get("wall_ms") or 0.0) / 1000.0 for row in summaries],
        "Total Wall Time for 15 Stages",
        "seconds",
    )
    write_simple_svg_bar(
        output_dir / "04b_throughput_dependency_free.svg",
        labels,
        [float(row.get("trials_per_sec") or 0.0) for row in summaries],
        "IK Throughput",
        "trials / second",
    )
    write_simple_svg_bar(
        output_dir / "04c_average_wall_per_trial_dependency_free.svg",
        labels,
        [float(row.get("wall_ms_per_trial") or 0.0) for row in summaries],
        "Average Wall Time per IK Trial",
        "ms / trial",
    )

def main() -> int:
    parser = argparse.ArgumentParser(description="Plot parallel IK speed-limit benchmark")
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path("/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/parallel_ik_speed_limit"),
    )
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args()

    output_dir = args.output_dir or (args.input_dir / "charts")
    output_dir.mkdir(parents=True, exist_ok=True)

    all_stages: list[dict] = []
    summaries: list[dict] = []
    for variant_id in VARIANT_LABELS:
        path = args.input_dir / f"{variant_id}.jsonl"
        stages, summary = load_variant(path, variant_id)
        for stage in stages:
            stage["wall_ms_per_trial"] = float(stage.get("wall_ms") or 0.0) / max(1, int(stage.get("trial_count") or 0))
            stage["sum_solve_ms_per_trial"] = float(stage.get("sum_solve_ms") or 0.0) / max(1, int(stage.get("trial_count") or 0))
        summary["trials_per_sec"] = 1000.0 * float(summary.get("trial_count") or 0.0) / max(1e-9, float(summary.get("wall_ms") or 0.0))
        summary["wall_ms_per_trial"] = float(summary.get("wall_ms") or 0.0) / max(1, int(summary.get("trial_count") or 0))
        all_stages.extend(stages)
        summaries.append(summary)

    stage_fields = [
        "variant_label", "variant_label_zh", "stage_no", "difficulty", "episode_id", "stage_name", "workers",
        "success", "trial_count", "legal_count", "timeout_like_count",
        "wall_ms", "wall_ms_per_trial", "sum_solve_ms", "sum_solve_ms_per_trial",
    ]
    summary_fields = [
        "variant_label", "variant_label_zh", "workers", "stage_count", "success_stage_count", "failed_stage_count",
        "trial_count", "legal_count", "timeout_like_count", "wall_ms", "wall_ms_per_trial",
        "sum_solve_ms", "trials_per_sec",
    ]
    write_csv(output_dir / "阶段耗时明细.csv", all_stages, stage_fields)
    write_csv(output_dir / "策略汇总.csv", summaries, summary_fields)

    md_lines = ["# 每阶段 256 次 IK 求解耗时对比", "", "|组别|阶段数|IK候选数|成功阶段|总wall(s)|平均wall/候选(ms)|吞吐量(trials/s)|", "|---|---:|---:|---:|---:|---:|---:|"]
    for row in summaries:
        md_lines.append(
            f"|{row.get('variant_label_zh', row['variant_label'])}|{row.get('stage_count', 0)}|{row.get('trial_count', 0)}|"
            f"{row.get('success_stage_count', 0)}|{float(row.get('wall_ms') or 0.0)/1000.0:.3f}|"
            f"{float(row.get('wall_ms_per_trial') or 0.0):.3f}|{float(row.get('trials_per_sec') or 0.0):.1f}|"
        )
    (output_dir / "实验汇总.md").write_text("\n".join(md_lines) + "\n")

    html_rows = []
    for row in summaries:
        html_rows.append(
            "<tr>"
            f"<td>{svg_escape(row.get('variant_label_zh', row['variant_label']))}</td>"
            f"<td>{row.get('stage_count', 0)}</td>"
            f"<td>{row.get('trial_count', 0)}</td>"
            f"<td>{row.get('success_stage_count', 0)}</td>"
            f"<td>{float(row.get('wall_ms') or 0.0)/1000.0:.3f}</td>"
            f"<td>{float(row.get('wall_ms_per_trial') or 0.0):.3f}</td>"
            f"<td>{float(row.get('trials_per_sec') or 0.0):.1f}</td>"
            "</tr>"
        )
    html = """<!doctype html>
<html lang="zh-CN">
<meta charset="utf-8">
<style>
body { font-family: 'Noto Sans CJK SC', 'Microsoft YaHei', 'SimHei', sans-serif; margin: 24px; }
table { border-collapse: collapse; }
th, td { border: 1px solid #ccc; padding: 8px 12px; }
th { background: #f3f3f3; }
img { max-width: 920px; display: block; margin: 24px 0; }
</style>
<h1>每阶段 256 次 IK 求解耗时对比</h1>
<table>
<tr><th>组别</th><th>阶段数</th><th>IK候选数</th><th>成功阶段</th><th>总wall(s)</th><th>平均wall/候选(ms)</th><th>吞吐量(trials/s)</th></tr>
""" + "\n".join(html_rows) + """
</table>
<h2>SVG 图表（英文标签，避免中文字体缺失）</h2>
<img src="04a_total_wall_time_dependency_free.svg">
<img src="04b_throughput_dependency_free.svg">
<img src="04c_average_wall_per_trial_dependency_free.svg">
</html>
"""
    (output_dir / "实验汇总.html").write_text(html, encoding="utf-8")


    write_fallback_svgs(output_dir, summaries)

    if HAS_MPL:
        plot_grouped_bars(all_stages, output_dir / "01_每阶段256次IK总耗时", "wall_ms", "每阶段256次IK总耗时 / ms", "每阶段 256 次 IK 求解总耗时对比")
        plot_grouped_bars(all_stages, output_dir / "02_每阶段平均单次IK耗时", "wall_ms_per_trial", "平均单次IK wall耗时 / ms", "每阶段平均单次 IK 耗时对比")
        plot_grouped_bars(all_stages, output_dir / "03_每阶段timeout数量", "timeout_like_count", "timeout-like 数量", "每阶段 timeout-like 数量对比")
        plot_summary(summaries, output_dir / "04_总体耗时与吞吐")
    else:
        print(f"跳过图表生成：matplotlib 不可用：{MPL_ERROR}")

    print(f"saved: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
