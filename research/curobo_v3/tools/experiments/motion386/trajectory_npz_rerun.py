"""Record an executed, timed trajectory NPZ as a real URDF/FK Rerun recording."""
import argparse
from pathlib import Path
import sys

import numpy as np
import rerun as rr
import rerun.blueprint as rrb


PHASE_COLORS = {
    "approach": [75, 137, 220], "contact_approach": [85, 188, 216], "attach": [232, 174, 68],
    "extract": [105, 187, 116], "transport": [74, 177, 126], "base_reposition": [152, 160, 173],
    "conveyor_place": [187, 112, 181], "release": [226, 111, 88], "return_home": [72, 173, 181],
    "base_return": [152, 160, 173], "home": [152, 160, 173],
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--selector", default="B2 group smooth")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[5]
    rerun_package = repo_root / "ros2_ws/src/alfa_robot_rerun"
    sys.path.insert(0, str(rerun_package))
    from alfa_robot_rerun.visualize_rerun import UrdfRobot, log_robot_state, log_robot_static_model

    document = np.load(args.trajectory)
    frames = np.asarray(document["frames"], dtype=np.float64)
    phases = np.asarray(document["phases"]).astype(str)
    payload = np.asarray(document["payload"], dtype=bool)
    times = np.asarray(document["time_from_start_s"], dtype=np.float64)
    names = np.asarray(document["joint_names"]).astype(str).tolist()
    if frames.ndim != 2 or frames.shape[1] != len(names) or len(frames) != len(phases) or len(frames) != len(payload):
        raise ValueError("trajectory arrays have inconsistent shapes")
    if len(times) != len(frames) or not np.isfinite(frames).all() or not np.isfinite(times).all():
        raise ValueError("trajectory must contain finite timing for every frame")
    if np.any(np.diff(times) < 0.):
        raise ValueError("trajectory time must be monotonic")

    robot = UrdfRobot(args.urdf.read_text())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rr.init("motion386_b2_executed_trajectory", spawn=False)
    rr.save(str(args.output))
    rr.send_blueprint(rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/world", contents=["/world/**"], name="B2 实际规划轨迹",
                              eye_controls=rrb.EyeControls3D(position=[-3.2, 3.2, 2.5],
                                                            look_target=[0., 0., .9], eye_up=[0., 0., 1.])),
            rrb.TextDocumentView(origin="/status", name="轨迹状态"), column_shares=[.78, .22]),
        rrb.TimePanel(timeline="trajectory_frame", expanded=True, fps=40, play_state="Playing"),
        collapse_panels=True))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/floor", rr.Boxes3D(centers=[[0., 0., -.025]], half_sizes=[[1.7, 1.7, .025]],
                                     colors=[[82, 91, 103, 50]]), static=True)
    log_robot_static_model(robot, "world/robot")

    prior_phase = None
    for index, (frame, phase, loaded, elapsed) in enumerate(zip(frames, phases, payload, times)):
        rr.set_time("trajectory_frame", sequence=index)
        log_robot_state(robot, dict(zip(names, frame)), "world/robot")
        if phase != prior_phase or index % 80 == 0:
            color = PHASE_COLORS.get(phase, [180, 180, 180])
            rr.log("world/phase_marker", rr.Points3D([[0., 0., 2.15]], colors=[color], radii=[.04], labels=[phase]))
            rr.log("status", rr.TextDocument(
                f"## {args.selector}\n\n"
                f"**任务：** `{args.task}`\n\n"
                f"**阶段：** `{phase}`\n\n"
                f"**负载：** `{'attached' if loaded else 'empty'}`\n\n"
                f"**时间：** {elapsed:.2f} / {times[-1]:.2f} s\n\n"
                f"**帧：** {index + 1} / {len(frames)}\n\n"
                "这是 B2 严格单组选组后由原规划器生成、通过阶段碰撞与定时校核的轨迹。",
                media_type=rr.MediaType.MARKDOWN))
        prior_phase = phase
    rr.get_global_data_recording().flush()
    print({"rrd": str(args.output.resolve()), "frames": len(frames), "duration_s": float(times[-1]),
           "phases": {phase: int((phases == phase).sum()) for phase in sorted(set(phases))}})


if __name__ == "__main__":
    main()
