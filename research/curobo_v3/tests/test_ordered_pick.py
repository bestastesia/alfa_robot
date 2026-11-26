"""CPU contracts for real phase attachment/world state and unchanged ordinary geometry load behavior."""
import ast
import copy
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from curobo_core.box_pick import BoxPick
from curobo_core.ordered_pick import ordered_stages,OrderedStage,cartesian_check


class OrderedPickTest(unittest.TestCase):
    def picks(self):
        return {s:BoxPick(size,(1.1680520510673524,0.,z),s,side_min_height_m=.79,carry_joint_home=True)
                for s,size,z in [('left','40x40x30',.5),('right','46.5x25.5x58.2',1.1)]}

    def test_eight_phase_payload_state_never_drops_pending_or_carried_box(self):
        stages=ordered_stages(self.picks())
        self.assertEqual(len(stages),8)
        self.assertEqual([s.payload_sides for s in stages],[(),(),('right',),('right',),('right',),('right',),('left','right'),('left','right')])
        self.assertEqual([s.pending_sides for s in stages],[('left','right'),('left','right'),('left',),('left',),('left',),('left',),(),()])
        self.assertEqual(stages[3].final_joint_sides,('right',))
        self.assertEqual(stages[-1].final_joint_sides,('left','right'))
        self.assertEqual(dict(stages[6].cartesian_displacements),{'left':(-.3,0.,0.),'right':(0.,0.,0.)})
        for stage in stages:
            self.assertEqual(set(stage.pending_sides)|set(stage.payload_sides),{'left','right'})
            self.assertFalse(set(stage.pending_sides)&set(stage.payload_sides))

    def test_contract_rejects_phantom_or_missing_payload_and_wrong_contact_phase(self):
        for kwargs in [dict(payload_sides=('right',),pending_sides=()),
                       dict(payload_sides=('right',),pending_sides=('left','right')),
                       dict(payload_sides=(),pending_sides=('left','right'),source_sides=('right',)),
                       dict(payload_sides=('right',),pending_sides=('left',),incoming_side='right')]:
            with self.assertRaises(ValueError):OrderedStage('test','approach',**kwargs)
        with self.assertRaises(ValueError):OrderedStage('../unsafe','contact',(),('left','right'))
        with self.assertRaises(ValueError):
            OrderedStage('extract','extract',('left','right'),(),cartesian_displacements=(('left',(-.3,0.,0.)),))

    def test_cartesian_moving_and_held_box_share_real_pose_tolerances(self):
        start=np.eye(4);start[:3,3]=[1.1,0.,.5]
        end=start.copy();end[0,3]-=.3
        poses=np.repeat(start[None],9,axis=0);poses[:,0,3]=np.linspace(start[0,3],end[0,3],9)
        self.assertTrue(cartesian_check(poses,start,end)['valid'])
        poses[4,1,3]+=.0021
        self.assertFalse(cartesian_check(poses,start,end)['valid'])
        held=np.repeat(start[None],3,axis=0)
        self.assertTrue(cartesian_check(held,start,start)['valid'])
        held[-1,2,3]+=.0021
        self.assertFalse(cartesian_check(held,start,start)['valid'])

    def test_geometry_explicit_payload_loads_carried_upper_during_lower_approach(self):
        path=ROOT/'tools/v3_fast_three_layers.py'
        cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='FastThreeLayerPlanner')
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='trajopt_path')
        begin=next(i for i,n in enumerate(fn.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='explicit_payload' for t in n.targets))
        end=next(i for i,n in enumerate(fn.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='managers' for t in n.targets))
        code=compile(ast.Module(body=fn.body[begin:end],type_ignores=[]),str(path),'exec')
        actual=(NS(parent_link='right_tool0'),)
        planner=NS(robot={'empty':True},box_fit={},task_attachments=lambda tasks:actual)
        for phase,source,loaded in [('approach',{'task':{},'active_payload_sides':['right']},True),
                                    ('approach',{'task':{}},False),('transport',{'task':{}},True),
                                    ('approach',{'task':{},'active_payload_sides':[]},False)]:
            calls=[]
            env=dict(self=NS(planner=planner),phase=phase,source=source,copy=copy,
                     loaded_robot=lambda *args,**kw:calls.append(kw['attachments']) or {'loaded':True})
            exec(code,env)
            self.assertEqual(bool(calls),loaded)
        with self.assertRaises(ValueError):
            exec(code,dict(self=NS(planner=planner),phase='approach',source={'task':{},'active_payload_sides':['left']},copy=copy))


if __name__=='__main__':unittest.main()


def test_ordered_runtime_callbacks_have_no_missing_global_names():
    # GPU callbacks are not all executed by CPU mocks; catch the actual missed-import failure without initializing CUDA.
    import builtins,symtable
    for relative in ('tools/v3_ordered_layout.py','tools/curobo_core/ordered_pick.py','tools/v3_two_box_timing.py','tools/v3_fast_three_layers.py'):
        path=ROOT/relative;table=symtable.symtable(path.read_text(),str(path),'exec')
        defined={s.get_name() for s in table.get_symbols() if s.is_assigned() or s.is_imported() or s.is_namespace()}
        defined|=set(vars(builtins))|{'__file__','__name__','__package__','__spec__','__loader__','__builtins__'}
        def visit(scope):
            for symbol in scope.get_symbols():
                if symbol.is_global() and symbol.is_referenced():
                    assert symbol.get_name() in defined,(relative,scope.get_name(),symbol.get_name())
            for child in scope.get_children():visit(child)
        visit(table)


def test_rejected_native_piece_recovery_keeps_a_serializable_failure(tmp_path):
    import json
    file=ROOT/'tools/v3_ordered_layout.py'
    method=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='_time_native_stage')
    failed=dict(success=False,phase='transport',trajectory_optimization={})
    result=dict(success=False,stages=[failed])
    planner=NS(set_stage=lambda *args:None,snapshot=NS(to_dict=lambda:{}))
    namespace=dict(np=np,Path=Path,replace=replace,time_pick=lambda *args,**kw:copy.deepcopy(result),atomic_json=lambda *args:None)
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(file),'exec'),namespace)
    picks=OrderedPickTest().picks();spec=ordered_stages(picks)[3]
    actual=namespace['_time_native_stage'](NS(planner=planner),dict(frames=np.zeros((2,15)).tolist()),spec,
        tmp_path/'native',None,{},{},{},{},lambda *args:None)
    assert not actual['success']
    assert not actual['stages'][0]['ordered_representation_recovery']['success']
    assert json.dumps(actual)


def test_default_dispatch_adds_ordered_recovery_without_relocating_boxes(tmp_path):
    from dataclasses import asdict
    from unittest.mock import patch
    import json
    file=ROOT/'tools/v3_box_layout_planner.py'
    cls=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='BoxLayoutPlanner')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='plan_layout')
    from curobo_core.layout_contract import validate_pair
    namespace=dict(np=np,Path=Path,json=json,replace=replace,asdict=asdict,validate_pair=validate_pair)
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(file),'exec'),namespace)
    picks=OrderedPickTest().picks();called=[]
    def ordered(actual,*args,**kwargs):
        called.append(actual)
        return dict(success=True,workflow='ordered_upper_first',task={s:asdict(p) for s,p in actual.items()})
    owner=NS(_plan_layout=lambda *args,**kw:dict(success=False,failure='synchronous contact is blocked'))
    with patch.dict(sys.modules,{'v3_ordered_layout':NS(plan_ordered_pick=ordered)}):
        result=namespace['plan_layout'](owner,picks,tmp_path)
    assert result['success'] and result['workflow']=='ordered_upper_first' and len(called)==1
    assert {s:p.center_m for s,p in called[0].items()}=={s:p.center_m for s,p in picks.items()}
    assert result['grasp_trials'][-1]['mode']=='ordered_upper_first'
    assert Path(result['grasp_trials'][-1]['result_file']).exists()


def test_replay_attachment_schedule_keeps_lower_original_until_its_contact():
    from dataclasses import asdict
    from curobo_core.ordered_pick import replay_payload_sides,replay_box_poses
    picks=OrderedPickTest().picks();picks['right']=replace(picks['right'],side_face='y-',suction_roll_deg=90.)
    record=dict(workflow='ordered_upper_first',task={s:asdict(p) for s,p in picks.items()})
    assert replay_payload_sides(record,'upper_contact')==()
    assert replay_payload_sides(record,'upper_extract')==('right',)
    assert replay_payload_sides(record,'lower_approach')==('right',)
    assert replay_payload_sides(record,'lower_contact')==('right',)
    assert replay_payload_sides(record,'lower_extract')==('left','right')
    transforms={'left_tool0':np.eye(4),'right_tool0':np.eye(4)}
    transforms['left_tool0'][:3,3]=[3.,4.,5.];transforms['right_tool0'][:3,3]=[.8,-.6,1.2]
    robot=NS(base_link='base',get_transform=lambda name,base:transforms[name])
    actual=replay_box_poses(robot,record,'lower_contact')
    np.testing.assert_allclose(actual['left'][:3,3],picks['left'].center_m)
    np.testing.assert_allclose(actual['right'],transforms['right_tool0']@picks['right'].tool_to_box())
    actual=replay_box_poses(robot,record,'lower_extract')
    np.testing.assert_allclose(actual['left'],transforms['left_tool0']@picks['left'].tool_to_box())
    with __import__('pytest').raises(ValueError):replay_payload_sides(record,'unknown')
