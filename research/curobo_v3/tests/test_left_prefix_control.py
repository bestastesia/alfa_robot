import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import MagicMock,patch
from urllib.error import HTTPError
from io import BytesIO

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/ipc_left_prefix'))
from model import JOINT_NAMES,MOTOR_SIGNS,export_prefix,motor_positions,read_library,select_path,time_parameterize,compact_path,validate_timing
from server import State
from console_lease import ConsoleLease


class PrefixControlTest(unittest.TestCase):
    def test_expired_console_session_reauthenticates_once_before_acquire(self):
        lease=ConsoleLease.__new__(ConsoleLease)
        lease.url='http://127.0.0.1:18121'
        response=MagicMock()
        response.__enter__.return_value=BytesIO(b'{"ok":true}')
        lease.opener=MagicMock()
        lease.opener.open.side_effect=[HTTPError(lease.url,401,'expired',{},BytesIO(b'{}')),response]
        with patch.object(lease,'refresh_login') as refresh:
            self.assertTrue(lease.acquire()['ok'])
            refresh.assert_called_once()
        self.assertEqual(lease.opener.open.call_count,2)

    def test_controller_inactive_is_distinct_from_query_stale(self):
        state=State(dict(tasks=dict(sample=self.record())),'left7_joint_replay')
        state.positions=[0.]*7;state.received=time.monotonic();state.stamp_age=0.
        self.assertIn('查询过期',state.reason())
        state.controller_received=time.monotonic()
        self.assertIn('未激活',state.reason())

    def record(self):
        return dict(model_positions=[[0.]*7,[.01,.02,.03,0.,0.,.04,0.],[.02,.03,.04,0.,0.,.05,0.]],
                    phases=['home','approach','contact_approach'])

    def test_export_stops_before_attachment_and_forbids_following_segments(self):
        names=['base_x','base_y','base_yaw','updown']+JOINT_NAMES+[f'right_joint{i}' for i in range(1,8)]
        phases=['home','approach','contact_approach','attach','extract','transport','release','return_home']
        frames=[[0.]*18 for _ in phases]
        for index in [1,2,3]:frames[index][4]=.01*min(index,2)
        for index in range(4,8):frames[index][4]=99.
        source=dict(success=True,contact_roll_policy='corrected',joint_names=names,phases=phases,frames=frames,task=dict(left=18,right=22))
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'source.json';path.write_text(json.dumps(source));record=export_prefix(path)
        self.assertEqual(len(record['model_positions']),3)
        self.assertEqual(record['model_positions'][-1][0],.02)
        self.assertEqual(record['phases'][-1],'contact_approach')

    def test_named_mapping_and_return_reverse_only_recorded_prefix(self):
        rows=self.record()['model_positions']
        self.assertEqual(motor_positions([1,2,3,4,5,6,7]),[1,2,3,4,5,6,7])
        backward=select_path(self.record(),motor_positions(rows[-1]),'home')
        self.assertEqual(backward[-1],motor_positions(rows[0]))
        self.assertEqual(backward[1:],list(reversed([motor_positions(row) for row in rows])))
        with self.assertRaises(ValueError):select_path(self.record(),[1.]*7,'forward')
        with self.assertRaises(ValueError):select_path(self.record(),[1.]*7,'home')
        with self.assertRaises(ValueError):select_path(self.record(),[0.]*7,'extract')

    def test_timing_respects_limits_and_rest_at_vertices(self):
        points=time_parameterize(self.record()['model_positions'],speed_deg_s=5.)
        self.assertTrue(all(len(point['positions'])==7 for point in points))
        self.assertTrue(all(end['time_s']>start['time_s'] for start,end in zip(points,points[1:])))
        self.assertLessEqual(max(abs(value) for point in points for value in point['velocities']),math.radians(5.)+1e-8)
        self.assertLessEqual(max(abs(value) for point in points for value in point['accelerations']),math.radians(10.)+1e-8)
        self.assertEqual(points[-1]['velocities'],[0.]*7)
        for start,end in zip(points,points[1:]):
            jerk=max(abs(b-a)/(end['time_s']-start['time_s']) for a,b in zip(start['accelerations'],end['accelerations']))
            self.assertLessEqual(jerk,math.radians(50.)+1e-6)

    def test_compaction_preserves_corners_and_reversal(self):
        rows=[[value,0.,0.,0.,0.,0.,0.] for value in [0.,.1,.2,.3]]
        rows+=[[.3,value,0.,0.,0.,0.,0.] for value in [.1,.2,.1,0.]]
        compact=compact_path(rows)
        self.assertEqual(compact,[rows[0],rows[3],rows[5],rows[-1]])
        self.assertLess(len(time_parameterize(rows)),len(rows))

    def test_actual_quintic_interpolation_peaks_and_runtime_hard_limits(self):
        points=time_parameterize(self.record()['model_positions'],speed_deg_s=15.)
        for start,end in zip(points,points[1:]):
            duration=end['time_s']-start['time_s']
            for index in range(1001):
                fraction=index/1000
                for origin,target in zip(start['positions'],end['positions']):
                    delta=target-origin
                    velocity=delta*(30*fraction**2-60*fraction**3+30*fraction**4)/duration
                    acceleration=delta*(60*fraction-180*fraction**2+120*fraction**3)/duration**2
                    jerk=delta*(60-360*fraction+360*fraction**2)/duration**3
                    self.assertLessEqual(abs(velocity),math.radians(15)+1e-9)
                    self.assertLessEqual(abs(acceleration),math.radians(30)+1e-9)
                    self.assertLessEqual(abs(jerk),math.radians(150)+1e-9)
        with self.assertRaises(ValueError):time_parameterize(self.record()['model_positions'],speed_deg_s=16.)
        with self.assertRaises(ValueError):validate_timing(points,max_duration_s=.1)
        with self.assertRaises(ValueError):validate_timing(points,max_points=1)

    def test_all_six_recordings_fit_15_degree_and_60_second_caps(self):
        library=read_library(Path(__file__).resolve().parents[1]/'generated/left_prefix_control_20261018/prefixes.json')
        for record in library['tasks'].values():
            rows=record['model_positions'];points=time_parameterize(rows,speed_deg_s=15.)
            self.assertLessEqual(len(points),200)
            self.assertLessEqual(points[-1]['time_s'],60.)
            for point in rows:
                errors=[]
                for first,last in zip(points,points[1:]):
                    start,end=first['positions'],last['positions']
                    delta=[b-a for a,b in zip(start,end)]
                    length=sum(value*value for value in delta)
                    fraction=sum((value-origin)*change for value,origin,change in zip(point,start,delta))/length if length>1e-20 else 0.
                    fraction=max(0.,min(1.,fraction))
                    errors.append(max(abs(value-origin-fraction*change) for value,origin,change in zip(point,start,delta)))
                self.assertLessEqual(min(errors),2.1e-7)

    def test_invalid_or_post_contact_library_is_rejected(self):
        data=dict(joint_names=JOINT_NAMES,motor_signs=MOTOR_SIGNS,tasks=dict(sample=self.record()))
        data['tasks']['sample']['phases'][1]='extract'
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'library.json';path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):read_library(path)

    def test_scope_and_feedback_gate_block_actual_publish(self):
        state=State(dict(tasks=dict(sample=self.record())),'pending')
        sent=[];state.publisher=lambda points:sent.append(points)
        with self.assertRaises(ValueError):state.send('sample','forward',5.)
        state.scope='left7_joint_replay'
        with self.assertRaises(ValueError):state.send('sample','forward',5.)
        self.assertEqual(sent,[])

    def test_ready_sends_only_seven_axis_prefix_and_locks_until_arrival(self):
        state=State(dict(tasks=dict(sample=self.record())),'left7_joint_replay')
        now=time.monotonic();state.positions=[0.]*7;state.received=now;state.stamp_age=0.
        state.samples.extend([(now-.25,[0.]*7),(now,[0.]*7)])
        state.controller=True;state.controller_received=now;state.subscribers=1
        state.enable_state='ENABLED';state.enable_received=now
        state.lease_client=object();state.console_status={'lease':'you'};state.console_received=now
        state.motor_bounds=[(-math.pi,math.pi)]*7
        sent=[];state.publisher=lambda points:sent.append(points)
        result=state.send('sample','forward',5.)
        self.assertEqual(result['joint_names'],JOINT_NAMES)
        self.assertEqual(sent[0][-1]['positions'],motor_positions(self.record()['model_positions'][-1]))
        with self.assertRaises(ValueError):state.send('sample','home',5.)
        self.assertEqual(len(sent),1)


if __name__=='__main__':
    unittest.main()
