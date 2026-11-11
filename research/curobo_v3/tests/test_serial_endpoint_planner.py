import sys
import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from v3_serial_endpoint_planner import nearest_endpoint,SerialEndpointPlanner
from v3_playback import playback_frame_interval


def test_terminal_selection_only_uses_predecessor_distance():
    previous=np.zeros(15)
    rows=np.zeros((3,15))
    rows[0,1]=.4
    rows[1,0]=.1
    rows[2,1]=.1
    chosen,index,distance=nearest_endpoint(rows,previous)
    assert index==2
    assert np.allclose(chosen,rows[2])
    assert np.isclose(distance,.1)


def test_teleport_does_not_slow_all_other_motion_frames():
    result=dict(joint_names=['left_joint1'],frames=[[0],[.01],[2.],[2.01]],phases=['home','approach','extraction_teleport','transport'])
    assert np.isclose(playback_frame_interval(result),.01/np.deg2rad(30))


def test_single_serial_result_contains_fields_read_by_playback():
    planner=SerialEndpointPlanner.__new__(SerialEndpointPlanner)
    planner.configure_task=lambda boxes:None
    planner.checker_resource_events=[]
    fixture=dict(task={'right':7},single_arm=True,success=True,
        transport_endpoint_envelope=dict(radius_m=.98,limit_m=1.05,includes_idle_arm=True,includes_payload=True))
    with patch('v3_serial_endpoint_planner.ConveyorCyclePlanner.full_cycle',return_value=fixture):
        result=planner.full_cycle({'right':7})
    assert result['active_side']=='right'
    assert result['transport_endpoint_envelope']['passed'] is True
    source=Path(__file__).resolve().parents[1]/'tools/v3_visualized_task_demo.py'
    tree=ast.parse(source.read_text())
    block=next(node for node in ast.walk(tree) if isinstance(node,ast.If) and ast.unparse(node.test)=="result.get('single_arm')")
    module=ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[]))
    status=SimpleNamespace(content='')
    exec(compile(module,str(source),'exec'),{'result':result,'status':status})
    assert '右手工作' in status.content
    assert '通过' in status.content
