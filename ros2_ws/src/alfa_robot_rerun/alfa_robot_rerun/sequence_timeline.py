"""Checked box segments, conflict detection, and execution-time concatenation."""
import math

from .demo_failure import replay_frames


def payload_execution_times(payload, frames, start_s=0.0):
    """Return frame times and the next segment start, never using receipt time."""
    if not math.isfinite(start_s) or start_s < 0.0:
        raise ValueError('invalid execution timeline start')
    if not frames:
        return [], start_s
    if payload.get('timing_valid', False):
        try:
            local = [float(frame['time_from_start_s']) for frame in frames]
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError('timed trajectory missing time_from_start_s') from error
        if (not all(math.isfinite(value) and value >= 0.0 for value in local)
                or any(right < left for left, right in zip(local, local[1:]))):
            raise ValueError('invalid timed trajectory frame times')
        duration = float(payload.get('execution_duration_s', local[-1]))
        if not math.isfinite(duration) or not math.isclose(
                duration, local[-1], rel_tol=0.0, abs_tol=1e-9):
            raise ValueError('execution duration does not match final frame time')
        return [start_s + value for value in local], start_s + duration
    step = float(payload.get('trajectory_sample_period', 0.05))
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError('invalid nominal sample period')
    return [start_s + index * step for index in range(len(frames))], start_s + (len(frames) - 1) * step


def sequence_execution_times(payload, frames):
    """Build a full execution timeline from authoritative per-segment records."""
    records = payload.get('boxes', [])
    if not records:
        return payload_execution_times(payload, frames)[0]
    output = [None] * len(frames)
    cursor = 0.0
    expected = 0
    failed_record = None
    for record in records:
        begin, end = record.get('frame_begin'), record.get('frame_end')
        if (type(begin) is not int or type(end) is not int or begin != expected
                or not begin <= end <= len(frames)):
            raise ValueError('invalid sequence timing range')
        if begin == end:
            if record.get('success', False) or failed_record is not None:
                raise ValueError('invalid sequence timing range')
            failed_record = record
            continue
        options = dict(payload, **record)
        times, cursor = payload_execution_times(options, frames[begin:end], cursor)
        output[begin:end] = times
        expected = end
    if expected < len(frames) and failed_record is not None and not payload.get('success', False):
        options = dict(payload, **failed_record)
        options['timing_valid'] = False
        times, cursor = payload_execution_times(options, frames[expected:], cursor)
        output[expected:] = times
        expected = len(frames)
    if expected != len(frames):
        raise ValueError('sequence timing ranges do not cover frames')
    return output


class SequenceTimeline:
    def __init__(self):
        self.task_id = None
        self.publisher_id = None
        self.generation = -1
        self.retired = set()
        self.retired_publishers = set()
        self.frames = []
        self.execution_times_s = []
        self.execution_cursor_s = 0.0
        self.scenes = []
        self.pending = {}
        self.segments = {}
        self.joint_names = None
        self.final_count = None

    def select(self, payload):
        task_id = payload['task_id']
        if (not isinstance(task_id, str) or not task_id
                or not isinstance(payload['publisher_id'], str) or not payload['publisher_id']
                or type(payload['generation']) is not int or payload['generation'] < 0):
            raise ValueError('invalid task identity')
        if task_id == self.task_id:
            return True
        if task_id in self.retired or payload['publisher_id'] in self.retired_publishers or (payload['publisher_id'] == self.publisher_id
                                     and payload['generation'] < self.generation):
            return False
        if self.task_id is not None:
            self.retired.add(self.task_id)
        if self.publisher_id is not None and self.publisher_id != payload['publisher_id']:
            self.retired_publishers.add(self.publisher_id)
        self.task_id = task_id
        self.publisher_id = payload['publisher_id']
        self.generation = payload['generation']
        self.frames = []
        self.execution_times_s = []
        self.execution_cursor_s = 0.0
        self.scenes = []
        self.pending = {}
        self.segments = {}
        self.joint_names = None
        self.final_count = None
        return True

    def append(self, payload):
        """Return only new contiguous suffixes; never paint scene state on receipt."""
        if payload.get('task_id') != self.task_id or self.task_id is None:
            raise ValueError('segment without selected task identity')
        frames = [{k: v for k, v in f.items() if k != 'diagnostic_only'}
                  for f in replay_frames(payload)]
        begin, end = payload['frame_begin'], payload['frame_end']
        if not (type(begin) is int and type(end) is int
                and 0 <= begin < end and end - begin == len(frames)):
            raise ValueError('invalid segment frame range')
        scenes = payload['scenes']
        if not scenes or any(not 0 <= f.get('scene_index', 0) < len(scenes) for f in frames):
            raise ValueError('invalid segment scene index')
        if self.joint_names is not None and self.joint_names != payload['joint_names']:
            raise ValueError('joint names changed within task')
        overlap = min(len(self.scenes), len(scenes))
        if self.scenes[:overlap] != scenes[:overlap]:
            raise ValueError('scene snapshot conflict')
        if self.final_count is not None and end > self.final_count:
            raise ValueError('segment extends beyond final result')
        overlap = min(end, len(self.frames))
        if begin < overlap and frames[:overlap - begin] != self.frames[begin:overlap]:
            raise ValueError('trajectory conflicts with written prefix')
        for pending in self.pending.values():
            a, b = max(begin, pending['frame_begin']), min(end, pending['frame_end'])
            if a < b and frames[a-begin:b-begin] != pending['frames'][a-pending['frame_begin']:b-pending['frame_begin']]:
                raise ValueError('trajectory conflicts with pending segment')
        final = payload['kind'] == 'result'
        if final:
            if begin != 0 or end < len(self.frames) or any(p['frame_end'] > end for p in self.pending.values()):
                raise ValueError('final result truncates sequence')
            if payload['segment_count'] != max(f.get('scene_index', 0) for f in frames) + 1:
                raise ValueError('final segment count mismatch')
            final_times = sequence_execution_times(payload, frames) if payload.get('boxes') else None
            if final_times is not None and self.execution_times_s and any(
                    abs(a - b) > 1e-9 for a, b in zip(final_times, self.execution_times_s)):
                raise ValueError('execution timeline conflicts with written prefix')
        else:
            index = payload['segment_index']
            if type(index) is not int or not 0 <= index < len(scenes):
                raise ValueError('invalid segment id')
            signature = (begin, end, payload['success'], payload.get('diagnostic'))
            if index in self.segments and self.segments[index] != signature:
                raise ValueError('duplicate segment id has conflicting metadata')
            if any(f.get('scene_index', 0) != index for f in frames):
                raise ValueError('segment id does not match scene')
            self.segments[index] = signature
            final_times = None
        self.joint_names = payload['joint_names']
        if len(scenes) > len(self.scenes):
            self.scenes = scenes
        item = dict(payload, frames=frames, diagnostic_frames=[], _execution_times_s=final_times)
        if final:
            self.final_count = end
            self.pending.clear()  # validated full snapshot repairs every missing range
        if end > len(self.frames):
            self.pending[begin] = item
        ready = []
        for start in sorted(self.pending):
            item = self.pending[start]
            if start > len(self.frames):
                break
            offset = max(0, len(self.frames) - start)
            suffix = item['frames'][offset:]
            if suffix:
                full_times = item['_execution_times_s']
                if full_times is None:
                    if start < len(self.execution_times_s):
                        local, _ = payload_execution_times(item, item['frames'], 0.0)
                        base = self.execution_times_s[start] - local[0]
                    else:
                        base = self.execution_cursor_s
                    full_times, next_cursor = payload_execution_times(item, item['frames'], base)
                    self.execution_cursor_s = max(self.execution_cursor_s, next_cursor)
                else:
                    self.execution_cursor_s = max(self.execution_cursor_s, full_times[-1])
                suffix_times = full_times[offset:]
                ready.append(dict(item, kind='result', frames=suffix,
                                  frame_begin=len(self.frames), stream_segment=True,
                                  execution_times_s=suffix_times))
                self.frames.extend(suffix)
                self.execution_times_s.extend(suffix_times)
            del self.pending[start]
        return ready
