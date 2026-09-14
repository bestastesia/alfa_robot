"""Checked box segments: contiguous append, conflict detection and final-snapshot repair."""
from .demo_failure import replay_frames


class SequenceTimeline:
    def __init__(self):
        self.task_id = None
        self.publisher_id = None
        self.generation = -1
        self.retired = set()
        self.retired_publishers = set()
        self.frames = []
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
        self.joint_names = payload['joint_names']
        if len(scenes) > len(self.scenes):
            self.scenes = scenes
        item = dict(payload, frames=frames, diagnostic_frames=[])
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
            suffix = item['frames'][max(0, len(self.frames) - start):]
            if suffix:
                ready.append(dict(item, kind='result', frames=suffix,
                                  frame_begin=len(self.frames), stream_segment=True))
                self.frames.extend(suffix)
            del self.pending[start]
        return ready
