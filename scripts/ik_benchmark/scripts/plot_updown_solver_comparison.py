#!/usr/bin/env python3
"""Generate Chinese charts/tables for updown solver comparison JSONL."""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
from pathlib import Path
from typing import Any

import pandas as pd

plt: Any = None
font_manager: Any = None

STRATEGY_LABELS = {
    "unlimited_bioik_until_collision_free": "对照组1：不限updown",
    "lookup_like_fixed_h_with_fallback": "对照组2：查表+降级",
    "parallel_updown_aware_solver": "实验组：多h多seed",
}

STRATEGY_ORDER = [
    "unlimited_bioik_until_collision_free",
    "lookup_like_fixed_h_with_fallback",
    "parallel_updown_aware_solver",
]


def load_matplotlib() -> bool:
    global plt, font_manager
    if plt is not None and font_manager is not None:
        return True
    try:
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            import matplotlib.pyplot as pyplot
            from matplotlib import font_manager as matplotlib_font_manager
    except Exception as exc:
        print(f"跳过图表生成：matplotlib 无法导入：{exc}")
        return False
    plt = pyplot
    font_manager = matplotlib_font_manager
    return True

def configure_fonts() -> None:
    font_candidates = [
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
        Path("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"),
        Path("/usr/share/fonts/truetype/arphic/uming.ttc"),
    ]
    for font_path in font_candidates:
        if font_path.exists():
            font_manager.fontManager.addfont(str(font_path))
            font_name = font_manager.FontProperties(fname=str(font_path)).get_name()
            plt.rcParams["font.family"] = font_name
            plt.rcParams["font.sans-serif"] = [font_name]
            break
    plt.rcParams["axes.unicode_minus"] = False


def load_records(path: Path) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    header: dict[str, Any] = {}
    stages: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            rec_type = rec.get("type")
            if rec_type == "header":
                header = rec
            elif rec_type == "stage":
                stages.append(rec)
            elif rec_type == "summary":
                summaries.append(rec)
            else:
                raise ValueError(f"Unsupported record type at line {line_no}: {rec_type}")
    if not stages:
        raise ValueError(f"No stage records found: {path}")
    return header, stages, summaries


def stage_sort_key(stage: str) -> tuple[int, int, str]:
    stage_order = {"safe": 0, "approach": 1, "grasp": 2, "retreat": 3, "place_safe": 4}
    parts = stage.split("/")
    round_no = 0
    if parts and parts[0].startswith("round_"):
        try:
            round_no = int(parts[0].split("_", 1)[1])
        except ValueError:
            round_no = 0
    stage_name = parts[-1]
    return round_no, stage_order.get(stage_name, 99), stage


def stage_label(stage: str) -> str:
    mapping = {
        "safe": "安全位",
        "approach": "预抓取",
        "grasp": "抓取",
        "retreat": "后退",
        "place_safe": "放置后安全位",
    }
    parts = stage.split("/")
    round_label = parts[0].replace("round_", "第") + "轮" if parts else stage
    name = mapping.get(parts[-1], parts[-1]) if parts else stage
    return f"{round_label}\n{name}"



def json_cell(value: Any) -> str:
    if value is None:
        return ""
    return json.dumps(value, ensure_ascii=False)


def cost_value(cost: dict[str, Any] | None, key: str) -> float:
    if not isinstance(cost, dict):
        return math.nan
    value = cost.get(key)
    return float(value) if value is not None else math.nan

def to_dataframe(stages: list[dict[str, Any]]) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    ordered_stages = sorted({rec["stage"] for rec in stages}, key=stage_sort_key)
    stage_index = {stage: index + 1 for index, stage in enumerate(ordered_stages)}
    for rec in stages:
        strategy = rec.get("strategy", "")
        joint2_lever = rec.get("joint2_lever_length")
        joint3_lever = rec.get("joint3_lever_length")
        h_interval = rec.get("h_interval") if isinstance(rec.get("h_interval"), dict) else {}
        selected_cost = rec.get("selected_cost_breakdown") if isinstance(rec.get("selected_cost_breakdown"), dict) else {}
        rows.append(
            {
                "节点序号": stage_index[rec["stage"]],
                "流程节点": rec["stage"],
                "流程节点中文": stage_label(rec["stage"]),
                "策略": strategy,
                "策略中文": STRATEGY_LABELS.get(strategy, strategy),
                "是否成功": bool(rec.get("success", False)),
                "当前updown位置_m": float(rec.get("current_h", 0.0) or 0.0),
                "选中updown位置_m": float(rec.get("selected_h", rec.get("current_h", 0.0)) or 0.0),
                "本阶段updown运动量_m": float(rec.get("updown_delta", 0.0) or 0.0),
                "累计updown运动量_m": 0.0,
                "IK求解次数": int(rec.get("trial_count", rec.get("attempt_count", 0)) or 0),
                "候选总数": int(rec.get("candidate_count", rec.get("trial_count", rec.get("attempt_count", 0))) or 0),
                "合法候选数": int(rec.get("legal_count", rec.get("legal_candidate_count", 0)) or 0),
                "拒绝候选数": int(rec.get("rejected_candidate_count", 0) or 0),
                "近似超时次数": int(rec.get("timeout_like_count", 0) or 0),
                "本阶段wall耗时_ms": float(rec.get("wall_ms", rec.get("total_solve_ms", 0.0)) or 0.0),
                "IK累计求解耗时_ms": float(rec.get("sum_solve_ms", rec.get("total_solve_ms", 0.0)) or 0.0),
                "是否使用降级": rec.get("fallback_used", ""),
                "求解路径": rec.get("solver_path", ""),
                "选中代价": rec.get("selected_score", math.nan),
                "选中candidate_index": rec.get("selected_candidate_index", ""),
                "选中rank": rec.get("selected_rank", ""),
                "选中h索引": rec.get("selected_h_index", ""),
                "选中seed索引": rec.get("selected_seed_index", ""),
                "选中target_order": rec.get("selected_target_order", ""),
                "选中solver_path": rec.get("selected_solver_path", ""),
                "选中joint_delta": rec.get("selected_joint_delta", math.nan),
                "h区间下限_m": rec.get("h_interval_lower", h_interval.get("lower", math.nan)),
                "h区间上限_m": rec.get("h_interval_upper", h_interval.get("upper", math.nan)),
                "h区间可达": rec.get("h_interval_reachable", h_interval.get("reachable", "")),
                "h候选数": rec.get("h_candidate_count", len(rec.get("h_candidates", [])) if isinstance(rec.get("h_candidates"), list) else 0),
                "h候选列表_json": json_cell(rec.get("h_candidates")),
                "direct位置误差_m": rec.get("direct_pos_error", math.nan),
                "direct姿态误差_rad": rec.get("direct_ori_error", math.nan),
                "是否无碰撞": rec.get("collision_free", ""),
                "碰撞pair列表_json": json_cell(rec.get("collision_pairs")),
                "失败原因": rec.get("failure_reason", rec.get("rejection_reason", "")),
                "选中关节名_json": json_cell(rec.get("selected_joint_names")),
                "选中关节值_json": json_cell(rec.get("selected_joint_values")),
                "左joint2力臂长度_m": rec.get("left_joint2_lever_length", math.nan),
                "右joint2力臂长度_m": rec.get("right_joint2_lever_length", math.nan),
                "joint2力臂长度_m": float(joint2_lever) if joint2_lever is not None else math.nan,
                "左joint3力臂长度_m": rec.get("left_joint3_lever_length", math.nan),
                "右joint3力臂长度_m": rec.get("right_joint3_lever_length", math.nan),
                "joint3力臂长度_m": float(joint3_lever) if joint3_lever is not None else math.nan,
                "选中cost_updown_static_bonus": cost_value(selected_cost, "updown_static_bonus"),
                "选中cost_updown_small_motion_bonus": cost_value(selected_cost, "updown_small_motion_bonus"),
                "选中cost_updown_over_small_motion_penalty": cost_value(selected_cost, "updown_over_small_motion_penalty"),
                "选中cost_left_joint2_torque": cost_value(selected_cost, "left_joint2_torque"),
                "选中cost_right_joint2_torque": cost_value(selected_cost, "right_joint2_torque"),
                "选中cost_left_joint3_torque": cost_value(selected_cost, "left_joint3_torque"),
                "选中cost_right_joint3_torque": cost_value(selected_cost, "right_joint3_torque"),
                "选中cost_solve_ms": cost_value(selected_cost, "solve_ms"),
                "选中cost_total": cost_value(selected_cost, "total"),
            }
        )
    df = pd.DataFrame(rows)
    df = df.sort_values(["节点序号", "策略"], key=lambda col: col.map({s: i for i, s in enumerate(STRATEGY_ORDER)}) if col.name == "策略" else col)
    df["累计updown运动量_m"] = df.groupby("策略", sort=False)["本阶段updown运动量_m"].cumsum()
    return df


def candidates_dataframe(stages: list[dict[str, Any]]) -> pd.DataFrame:
    rows: list[dict[str, Any]] = []
    ordered_stages = sorted({rec["stage"] for rec in stages}, key=stage_sort_key)
    stage_index = {stage: index + 1 for index, stage in enumerate(ordered_stages)}
    for rec in stages:
        strategy = rec.get("strategy", "")
        for cand in rec.get("candidates", []) or []:
            if not isinstance(cand, dict):
                continue
            cost = cand.get("cost_breakdown") if isinstance(cand.get("cost_breakdown"), dict) else {}
            rows.append({
                "节点序号": stage_index[rec["stage"]],
                "流程节点": rec["stage"],
                "流程节点中文": stage_label(rec["stage"]),
                "策略": strategy,
                "策略中文": STRATEGY_LABELS.get(strategy, strategy),
                "candidate_index": cand.get("candidate_index", ""),
                "rank": cand.get("rank", ""),
                "是否选中": bool(cand.get("selected", False)),
                "是否合法": bool(cand.get("legal", False)),
                "是否无碰撞": cand.get("collision_free", ""),
                "是否左右绑定错误": cand.get("swapped", ""),
                "是否近似超时": cand.get("timeout_like", ""),
                "求解路径": cand.get("solver_path", ""),
                "target_order": cand.get("target_order", ""),
                "拒绝原因": cand.get("rejection_reason", ""),
                "h_m": cand.get("h", math.nan),
                "h_center_m": cand.get("h_center", math.nan),
                "h_index": cand.get("h_index", ""),
                "seed_index": cand.get("seed_index", ""),
                "score": cand.get("score", math.nan),
                "solve_ms": cand.get("solve_ms", math.nan),
                "direct位置误差_m": cand.get("direct_pos_error", math.nan),
                "direct姿态误差_rad": cand.get("direct_ori_error", math.nan),
                "swapped位置误差_m": cand.get("swapped_pos_error", math.nan),
                "updown运动量_m": cand.get("updown_delta", math.nan),
                "joint_delta": cand.get("joint_delta", math.nan),
                "collision_pairs_json": json_cell(cand.get("collision_pairs")),
                "joint_names_json": json_cell(cand.get("joint_names")),
                "joint_values_json": json_cell(cand.get("joint_values")),
                "full_joint_names_json": json_cell(cand.get("full_joint_names")),
                "full_joint_values_json": json_cell(cand.get("full_joint_values")),
                "左joint2力臂长度_m": cand.get("left_joint2_lever_length", math.nan),
                "右joint2力臂长度_m": cand.get("right_joint2_lever_length", math.nan),
                "joint2力臂长度_m": cand.get("joint2_lever_length", math.nan),
                "左joint3力臂长度_m": cand.get("left_joint3_lever_length", math.nan),
                "右joint3力臂长度_m": cand.get("right_joint3_lever_length", math.nan),
                "joint3力臂长度_m": cand.get("joint3_lever_length", math.nan),
                "cost_updown_static_bonus": cost_value(cost, "updown_static_bonus"),
                "cost_updown_small_motion_bonus": cost_value(cost, "updown_small_motion_bonus"),
                "cost_updown_over_small_motion_penalty": cost_value(cost, "updown_over_small_motion_penalty"),
                "cost_left_joint2_torque": cost_value(cost, "left_joint2_torque"),
                "cost_right_joint2_torque": cost_value(cost, "right_joint2_torque"),
                "cost_left_joint3_torque": cost_value(cost, "left_joint3_torque"),
                "cost_right_joint3_torque": cost_value(cost, "right_joint3_torque"),
                "cost_solve_ms": cost_value(cost, "solve_ms"),
                "cost_total": cost_value(cost, "total"),
            })
    return pd.DataFrame(rows)


def safe_filename(value: str) -> str:
    safe = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in value)
    return safe.strip("_") or "stage"


def write_candidate_stage_csvs(candidates: pd.DataFrame, output_dir: Path) -> None:
    split_dir = output_dir / "候选按任务拆分"
    split_dir.mkdir(parents=True, exist_ok=True)
    if candidates.empty:
        return
    sort_cols = [col for col in ["是否合法", "rank", "score", "candidate_index"] if col in candidates.columns]
    for stage, sub in candidates.groupby("流程节点", sort=False):
        ordered = sub.copy()
        if "是否合法" in ordered.columns:
            ordered["__legal_sort"] = ordered["是否合法"].map(lambda v: 0 if bool(v) else 1)
            sort_cols = ["__legal_sort"] + [col for col in ["rank", "score", "candidate_index"] if col in ordered.columns]
        ordered = ordered.sort_values(sort_cols, na_position="last") if sort_cols else ordered
        if "__legal_sort" in ordered.columns:
            ordered = ordered.drop(columns=["__legal_sort"])
        prefix = f"{int(ordered['节点序号'].iloc[0]):02d}" if "节点序号" in ordered.columns and not ordered.empty else "00"
        ordered.to_csv(split_dir / f"{prefix}_{safe_filename(str(stage))}_候选明细.csv", index=False, encoding="utf-8-sig")


def candidate_summary_dataframe(candidates: pd.DataFrame) -> pd.DataFrame:
    if candidates.empty:
        return pd.DataFrame(columns=["策略", "策略中文", "流程节点", "候选总数", "合法候选数", "非法候选数", "合法率_%", "近似超时次数", "最小score", "平均score", "平均solve_ms", "主要拒绝原因"])
    rows: list[dict[str, Any]] = []
    for (strategy, strategy_cn, stage), sub in candidates.groupby(["策略", "策略中文", "流程节点"], sort=False):
        total = len(sub)
        legal = int(sub["是否合法"].sum())
        rejected = total - legal
        reasons = sub.loc[~sub["是否合法"], "拒绝原因"].replace("", pd.NA).dropna()
        main_reason = reasons.mode().iloc[0] if not reasons.empty else ""
        rows.append({
            "策略": strategy,
            "策略中文": strategy_cn,
            "流程节点": stage,
            "流程节点中文": stage_label(stage),
            "候选总数": total,
            "合法候选数": legal,
            "非法候选数": rejected,
            "合法率_%": legal / total * 100.0 if total else math.nan,
            "近似超时次数": int(sub["是否近似超时"].sum()),
            "最小score": sub["score"].min(),
            "平均score": sub["score"].mean(),
            "平均solve_ms": sub["solve_ms"].mean(),
            "主要拒绝原因": main_reason,
        })
    return pd.DataFrame(rows)


def summary_dataframe(df: pd.DataFrame) -> pd.DataFrame:
    grouped = df.groupby(["策略", "策略中文"], sort=False).agg(
        流程节点数=("流程节点", "count"),
        成功节点数=("是否成功", "sum"),
        updown总运动量_m=("本阶段updown运动量_m", "sum"),
        wall总耗时_s=("本阶段wall耗时_ms", lambda s: s.sum() / 1000.0),
        IK总求解耗时_s=("IK累计求解耗时_ms", lambda s: s.sum() / 1000.0),
        IK求解总次数=("IK求解次数", "sum"),
        近似超时总次数=("近似超时次数", "sum"),
        平均合法候选数=("合法候选数", "mean"),
    ).reset_index()
    baseline_motion = grouped.loc[grouped["策略"] == "unlimited_bioik_until_collision_free", "updown总运动量_m"]
    lookup_motion = grouped.loc[grouped["策略"] == "lookup_like_fixed_h_with_fallback", "updown总运动量_m"]
    baseline_time = grouped.loc[grouped["策略"] == "unlimited_bioik_until_collision_free", "wall总耗时_s"]
    lookup_time = grouped.loc[grouped["策略"] == "lookup_like_fixed_h_with_fallback", "wall总耗时_s"]
    grouped["相对不限updown运动量_%"] = grouped["updown总运动量_m"] / float(baseline_motion.iloc[0]) * 100.0 if not baseline_motion.empty else math.nan
    grouped["相对lookup运动量_%"] = grouped["updown总运动量_m"] / float(lookup_motion.iloc[0]) * 100.0 if not lookup_motion.empty else math.nan
    grouped["相对不限updown耗时_%"] = grouped["wall总耗时_s"] / float(baseline_time.iloc[0]) * 100.0 if not baseline_time.empty else math.nan
    grouped["相对lookup耗时_%"] = grouped["wall总耗时_s"] / float(lookup_time.iloc[0]) * 100.0 if not lookup_time.empty else math.nan
    return grouped


def plot_lines(df: pd.DataFrame, y: str, title: str, ylabel: str, output: Path, marker: str = "o") -> None:
    fig, ax = plt.subplots(figsize=(14, 6))
    stages = df[["节点序号", "流程节点中文"]].drop_duplicates().sort_values("节点序号")
    for strategy in STRATEGY_ORDER:
        sub = df[df["策略"] == strategy].sort_values("节点序号")
        if sub.empty:
            continue
        if sub[y].isna().all():
            continue
        ax.plot(sub["节点序号"], sub[y], marker=marker, linewidth=2, label=STRATEGY_LABELS.get(strategy, strategy))
    ax.set_title(title)
    if df[y].isna().all():
        ax.text(0.5, 0.5, "当前JSONL未记录选中关节角，无法从历史数据还原该曲线\n后续重新运行实验会自动保存 joint2/joint3 力臂字段",
                transform=ax.transAxes, ha="center", va="center", fontsize=13)
    ax.set_xlabel("流程节点")
    ax.set_ylabel(ylabel)
    ax.set_xticks(stages["节点序号"].tolist())
    ax.set_xticklabels(stages["流程节点中文"].tolist(), rotation=35, ha="right")
    ax.grid(True, alpha=0.3)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend()
    fig.tight_layout()
    fig.savefig(output.with_suffix(".png"), dpi=180)
    fig.savefig(output.with_suffix(".svg"))
    plt.close(fig)


def plot_bar(summary: pd.DataFrame, y: str, title: str, ylabel: str, output: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 5))
    labels = summary["策略中文"].tolist()
    values = summary[y].tolist()
    bars = ax.bar(labels, values)
    ax.set_title(title)
    ax.set_xlabel("策略")
    ax.set_ylabel(ylabel)
    ax.grid(True, axis="y", alpha=0.3)
    for bar, value in zip(bars, values):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(), f"{value:.3g}", ha="center", va="bottom")
    fig.tight_layout()
    fig.savefig(output.with_suffix(".png"), dpi=180)
    fig.savefig(output.with_suffix(".svg"))
    plt.close(fig)


def write_markdown(summary: pd.DataFrame, df: pd.DataFrame, candidate_summary: pd.DataFrame, output: Path, source: Path, metadata: dict[str, Any]) -> None:
    with output.open("w", encoding="utf-8") as f:
        f.write(f"# Updown 求解策略对比实验汇总\n\n")
        f.write(f"数据源：`{source}`\n\n")
        f.write(f"schema_version：`{metadata.get('schema_version', '')}`\n\n")
        f.write(f"候选明细：`{metadata.get('candidate_records_count', 0)}` 条\n\n")
        f.write("## 总览\n\n")
        f.write(summary.drop(columns=["策略"]).to_markdown(index=False, floatfmt=".3f"))
        f.write("\n\n## 每阶段明细\n\n")
        cols = [
            "流程节点中文",
            "策略中文",
            "是否成功",
            "选中updown位置_m",
            "本阶段updown运动量_m",
            "累计updown运动量_m",
            "IK求解次数",
            "近似超时次数",
            "本阶段wall耗时_ms",
            "IK累计求解耗时_ms",
            "joint2力臂长度_m",
            "joint3力臂长度_m",
            "求解路径",
        ]
        f.write(df[cols].to_markdown(index=False, floatfmt=".3f"))
        f.write("\n")
        if not candidate_summary.empty:
            f.write("\n\n## 候选解汇总\n\n")
            f.write(candidate_summary.to_markdown(index=False, floatfmt=".3f"))
            f.write("\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="生成 Updown solver 对比实验中文图表")
    parser.add_argument("jsonl", type=Path, help="updown_solver_comparison JSONL")
    parser.add_argument("--output-dir", type=Path, default=None, help="图表输出目录")
    args = parser.parse_args()

    charts_enabled = load_matplotlib()
    if charts_enabled:
        configure_fonts()
    header, stages, _ = load_records(args.jsonl)
    df = to_dataframe(stages)
    summary = summary_dataframe(df)
    candidates = candidates_dataframe(stages)
    candidate_summary = candidate_summary_dataframe(candidates)

    output_dir = args.output_dir or args.jsonl.parent / "charts"
    output_dir.mkdir(parents=True, exist_ok=True)

    df.to_csv(output_dir / "每阶段明细.csv", index=False, encoding="utf-8-sig")
    summary.to_csv(output_dir / "策略汇总.csv", index=False, encoding="utf-8-sig")
    candidates.to_csv(output_dir / "候选明细.csv", index=False, encoding="utf-8-sig")
    candidate_summary.to_csv(output_dir / "候选汇总.csv", index=False, encoding="utf-8-sig")
    write_candidate_stage_csvs(candidates, output_dir)
    metadata = {
        "source": str(args.jsonl),
        "output_dir": str(output_dir),
        "header": header,
        "schema_version": header.get("schema_version", 1),
        "enabled_strategies": header.get("enabled_strategies", []),
        "experiment_h_search_mode": header.get("experiment_h_search_mode", ""),
        "stage_records": len(stages),
        "candidate_records_available": not candidates.empty,
        "candidate_records_count": len(candidates),
        "charts_enabled": charts_enabled,
        "charts_note": "joint2/joint3 lever charts use joint2_lever_length/joint3_lever_length, which are max(left,right) when generated by current benchmark.",
    }
    write_markdown(summary, df, candidate_summary, output_dir / "实验汇总.md", args.jsonl, metadata)

    if charts_enabled:
        plot_lines(df, "本阶段updown运动量_m", "全过程每阶段 updown 运动量", "updown运动量 / m", output_dir / "01_全过程updown运动量")
        plot_lines(df, "选中updown位置_m", "全过程 updown 实际位置", "updown位置 / m", output_dir / "02_全过程updown位置")
        plot_lines(df, "joint2力臂长度_m", "全过程 joint2 力臂长度", "力臂长度 / m", output_dir / "03_全过程joint2力臂长度")
        plot_lines(df, "joint3力臂长度_m", "全过程 joint3 力臂长度", "力臂长度 / m", output_dir / "04_全过程joint3力臂长度")
        plot_lines(df, "IK求解次数", "全过程单次任务 IK 求解次数", "IK求解次数 / 次", output_dir / "05_全过程单次任务IK求解次数")
        plot_lines(df, "IK累计求解耗时_ms", "全过程单次任务 IK 求解总耗时", "IK求解总耗时 / ms", output_dir / "06_全过程单次任务IK求解总耗时")
        plot_lines(df, "本阶段wall耗时_ms", "全过程单次任务 wall 耗时", "wall耗时 / ms", output_dir / "07_全过程单次任务wall耗时")
        plot_lines(df, "累计updown运动量_m", "全过程累计 updown 运动量", "累计updown运动量 / m", output_dir / "08_全过程累计updown运动量")
        plot_bar(summary, "updown总运动量_m", "三种策略 updown 总运动量", "updown总运动量 / m", output_dir / "09_策略updown总运动量")
        plot_bar(summary, "wall总耗时_s", "三种策略 wall 总耗时", "wall总耗时 / s", output_dir / "10_策略wall总耗时")

    (output_dir / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"saved charts/tables to: {output_dir}")


if __name__ == "__main__":
    main()
