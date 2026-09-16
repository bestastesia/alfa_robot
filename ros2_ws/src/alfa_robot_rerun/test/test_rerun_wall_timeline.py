from datetime import timedelta
from unittest.mock import patch

import numpy as np
import rerun as rr
from rerun.experimental import RrdReader

from alfa_robot_rerun.v3_single_arm_box_extract_viewer import PlaybackFrame, V3SingleArmBoxExtractViewer


def test_execution_time_is_numeric_in_real_rrd(tmp_path):
    recording = tmp_path / "execution_time.rrd"
    rr.init("rerun_wall_timeline_test")
    rr.save(recording)

    viewer = V3SingleArmBoxExtractViewer.__new__(V3SingleArmBoxExtractViewer)
    viewer.frames = [
        PlaybackFrame("start", (0.0,), False, execution_time_s=0.25),
        PlaybackFrame("finish", (1.0,), False, execution_time_s=1.5),
    ]
    viewer.joint_names = ("joint",)
    viewer.robot = SimpleRobot()
    viewer.global_frame = 1
    viewer.frame_index = 0
    viewer.generation = 1
    viewer.scenes = []
    viewer.scene_index = -1
    viewer.diagnostic = {}
    viewer.stream_segment = False
    viewer.last_frame = None

    with patch.object(viewer, "log_boxes"), patch.object(viewer, "log_summary"):
        viewer.write_trajectory()
    rr.get_global_data_recording().disconnect()

    values = {}
    for chunk in RrdReader(recording).store().stream():
        if chunk.is_static or "execution_time" not in chunk.timeline_names:
            continue
        batch = chunk.to_record_batch()
        durations = batch.column("execution_time").to_pylist()
        assert all(isinstance(value, timedelta) for value in durations)
        values.setdefault(str(chunk.entity_path), []).extend(value.total_seconds() for value in durations)

    assert values["/world/current_stage"] == [0.25, 1.5]
    assert values["/world/robot/link"] == [0.25, 1.5]


class SimpleRobot:
    links = ("link",)

    @staticmethod
    def fk(_):
        return {"link": np.eye(4)}
