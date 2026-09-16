import json
from types import SimpleNamespace
from unittest.mock import patch

from alfa_robot_rerun.sequence_timeline import SequenceTimeline
from alfa_robot_rerun import v3_single_arm_box_extract_viewer as module


def _segment(index, begin):
    frames = [
        dict(stage=f"box_{index}_{offset}", joints=[float(begin + offset)],
             box_attached=False, scene_index=index)
        for offset in range(2)
    ]
    return dict(
        kind="segment", sequence=True, task_id="task", publisher_id="publisher",
        generation=1, segment_index=index, frame_begin=begin, frame_end=begin + 2,
        joint_names=["joint"], scenes=[{}] * (index + 1), success=True,
        frames=frames, diagnostic_frames=[], trajectory_sample_period=0.05,
    )


def test_final_without_boxes_updates_summary_without_rewriting_frames():
    timeline = SequenceTimeline()
    segments = [_segment(0, 0), _segment(1, 2)]
    assert timeline.select(segments[0])
    for segment in segments:
        assert len(timeline.append(segment)) == 1

    original_frames = list(timeline.frames)
    final = dict(
        segments[0], kind="result", frame_begin=0, frame_end=4, segment_count=2,
        scenes=[{}, {}], frames=segments[0]["frames"] + segments[1]["frames"],
        completed_count=2, failed_box_id=-1, distance_demo=True,
        metrics={"ik_calls": 7}, status="整墙完成",
    )

    viewer = module.V3SingleArmBoxExtractViewer.__new__(module.V3SingleArmBoxExtractViewer)
    viewer.sequence_timeline = timeline
    viewer.sequence_started = 0.0
    viewer.global_frame = 5
    viewer.generation = 1
    viewer.frames = [SimpleNamespace(stage="last")]
    viewer.last_frame = viewer.frames[0]
    summaries = []
    set_times = []
    logger = SimpleNamespace(info=lambda _: None, warning=lambda _: None,
                             error=lambda message: (_ for _ in ()).throw(AssertionError(message)))

    def set_time(timeline_name, **value):
        set_times.append((timeline_name, value))

    with patch.object(module.rr, "set_time", side_effect=set_time), \
         patch.object(module.rr, "get_global_data_recording", return_value=None), \
         patch.object(viewer, "log_summary", side_effect=lambda status, planning: summaries.append((status, planning))), \
         patch.object(viewer, "write_trajectory") as write_trajectory, \
         patch.object(module.V3SingleArmBoxExtractViewer, "get_logger", return_value=logger):
        viewer.on_task(SimpleNamespace(data=json.dumps(final)))
        viewer.on_task(SimpleNamespace(data=json.dumps(final)))

    assert timeline.frames == original_frames
    assert set_times == [("task_frame", {"sequence": 5})]
    assert summaries == [("整墙完成", False)]
    assert viewer.metrics == {"ik_calls": 7}
    assert viewer.frames[0].stage == "last" and viewer.last_frame is viewer.frames[0]
    write_trajectory.assert_not_called()


def test_time_mock_accepts_sequence_and_duration_keywords():
    viewer = module.V3SingleArmBoxExtractViewer.__new__(module.V3SingleArmBoxExtractViewer)
    viewer.frames = [module.PlaybackFrame("done", (0.0,), False, execution_time_s=1.25)]
    viewer.frame_index = 0
    viewer.global_frame = 3
    viewer.generation = 1
    viewer.joint_names = ("joint",)
    viewer.scenes = []
    viewer.scene_index = -1
    viewer.diagnostic = {}
    viewer.last_frame = None
    calls = []

    def set_time(timeline_name, **value):
        calls.append((timeline_name, value))

    with patch.object(module.rr, "set_time", side_effect=set_time), \
         patch.object(module.rr, "log"), patch.object(viewer, "log_boxes"):
        viewer.log_frame({})

    assert calls == [
        ("task_frame", {"sequence": 3}),
        ("execution_time", {"duration": 1.25}),
    ]
