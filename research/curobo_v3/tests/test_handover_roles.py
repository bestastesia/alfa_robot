"""CPU regression for real role/ID helpers, scenes, caps and bounded seven-axis retreat."""
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import yourdfpy

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from curobo_core.handover_roles import HandoverRoles,PHYSICAL_IDS,NAMES,arm_slice,select_roles,admitted_precontacts
from curobo_core.box_pick import BoxPick,diverse_contact_order
from curobo_core.layout_contract import carry_configuration
from curobo_core.handover import STAGES,_contract,_boxes
from curobo_core.scene import SceneSnapshot,RobotState,SceneObject,AttachedObject,Pose
from curobo_core.adapter import matrix_pose
from curobo_core.suction_grasp import contact_cap_spheres
from experiments.box_layout_20261119.transfer_fresh.plan_fresh import donor_retreat


def test_handover_roles():
    before={n for n in sys.modules if n.split('.')[0] in ('torch','curobo')}
    picks={s:BoxPick(size,(1.168052051,-.85,z),s,side_min_height_m=.79,carry_joint_home=True)
        for s,size,z in [('left','40x40x30',.8),('right','46.5x25.5x58.2',1.4)]}
    negative=select_roles(picks)
    assert negative==HandoverRoles('right','left','right')
    assert negative.owner_to_physical_id=={'right':'wall_box_00','left':'wall_box_01'}
    positive={s:replace(p,center_m=(p.center_m[0],.85,p.center_m[2])) for s,p in picks.items()}
    assert select_roles(positive)==HandoverRoles()
    assert [p.extraction_distance_m for p in picks.values()]==[.3,.255]
    assert {s:len(contact_cap_spheres(s)) for s in ('left','right')}=={'left':10,'right':11}
    robot=yourdfpy.URDF.load(ROOT/'generated/fast_runtime_assets/fixed.urdf',load_meshes=False)
    q=np.asarray([carry_configuration(-.4)[n] for n in NAMES]);robot.update_cfg(dict(zip(NAMES,q)))
    tools={s:robot.get_transform(s+'_tool0',robot.base_link).copy() for s in ('left','right')}
    for role in (negative,HandoverRoles()):
        specs=[]
        for index,name in enumerate(STAGES):
            parents=role.parents(index);objects=[];attached=[]
            for identity,pick in picks.items():
                physical_id=PHYSICAL_IDS[identity];C=np.eye(4);C[:3,3]=pick.center_m
                if physical_id in parents:
                    parent=parents[physical_id];side=parent.removesuffix('_tool0')
                    attached.append(AttachedObject(physical_id,pick.dimensions_m,parent,matrix_pose(np.linalg.inv(tools[side])@C)))
                else:objects.append(SceneObject(physical_id,pick.dimensions_m,Pose(pick.center_m)))
            scene=SceneSnapshot('cpu-role-fixture',RobotState(tuple(NAMES),tuple(q),0),tuple(objects),tuple(attached))
            spec=dict(stage_id=name,scene=scene.to_dict());specs.append(spec)
            boxes=_boxes(robot,spec,q[None])
            for identity,pick in picks.items():
                assert np.allclose(boxes[PHYSICAL_IDS[identity]][0,:3,3],pick.center_m,atol=1e-12)
        record=dict(workflow='handover_same_side',success=True,joint_names=NAMES,
            timing=dict(success=True,stages=[dict(stage_id=s,success=True,physical_success=True) for s in STAGES]),
            stage_interface=dict(joint_names=NAMES,stages=specs,support_transfer=dict(
                physical_id=PHYSICAL_IDS[role.source],parent_before=role.giver+'_tool0',parent_after=role.recipient+'_tool0',
                mass_kg=10.,stationary_only=True,alpha_key='support_alpha')))
        assert _contract(record)[0]==specs  # Schema fixtures only, not a native-success certificate.
    for side in ('left','right'):
        rows,detail=donor_retreat(robot,q,side)
        assert detail['success'] and rows.shape==(36,15)
        columns=arm_slice(side);names=NAMES[columns]
        lo=np.array([robot.joint_map[n].limit.lower for n in names]);hi=np.array([robot.joint_map[n].limit.upper for n in names])
        assert np.all(rows[:,columns]>=lo+.001-1e-6) and np.all(rows[:,columns]<=hi-.001+1e-6)
        idle=[i for i in range(15) if i not in range(columns.start,columns.stop)]
        assert np.array_equal(rows[:,idle],np.repeat(rows[:1,idle],len(rows),axis=0))
        for column in range(columns.start,columns.stop):
            near=q.copy();near[column]=robot.joint_map[NAMES[column]].limit.upper-.0005
            rejected,why=donor_retreat(robot,near,side)
            assert rejected is None and why['failure']=='donor start lacks1mrad native spline joint margin'
    # A bounded pair pool may repeat the same giver while only the held arm varies.
    # Spend the same four trials across actual giver elbow families, using the shared helper.
    pairs=np.repeat(q[None],6,axis=0);pairs[:4,11]=1.;pairs[4:,11]=-1.
    pairs[:4,1]+=np.arange(4)*.01
    order=diverse_contact_order(pairs,'right')
    assert order[:4].tolist()==[0,4,1,5]
    assert np.array_equal(pairs[order][0,8:],pairs[0,8:])
    assert np.sign(pairs[order[:4],11]).tolist()==[1.,-1.,1.,-1.]
    # Selecting by left would miss this right-giver distinction entirely.
    assert diverse_contact_order(pairs,'left').tolist()==list(range(6))
    assert 'contacts[diverse_contact_order(contacts,giver)]' in (ROOT/'tools/experiments/box_layout_20261119/transfer_suffix/probe_suffix.py').read_text()
    assert before=={n for n in sys.modules if n.split('.')[0] in ('torch','curobo')}


def test_precontact_admission():
    # Repeated giver7 with different held-arm configurations remain distinct physical candidates.
    rows=np.zeros((8,15),dtype=np.float32);rows[:,1]=np.arange(8)*.01
    rows[4:,8]=.1;rows[6:,8]=.2
    checked=[]
    def check(index,q):
        checked.append(index)
        return np.stack((q,q)) if index in (5,6,7,3) else None
    admitted=list(admitted_precontacts(rows,'right',check,limit=2))
    assert checked==[0,4,6,1,5] and [i for i,q,path in admitted]==[6,5]
    assert all(np.array_equal(path[0],q) for i,q,path in admitted)
    # Rejected raw contacts do not consume a full trial; the finite pool is not expanded.
    checked=[]
    assert list(admitted_precontacts(rows,'right',lambda i,q:checked.append(i)))==[]
    assert sorted(checked)==list(range(8))
    def expired(index,q):raise TimeoutError('same task deadline')
    try:list(admitted_precontacts(rows,'right',expired))
    except TimeoutError:pass
    else:raise AssertionError('admission swallowed the task deadline')


if __name__=='__main__':
    sys.dont_write_bytecode=True
    test_handover_roles()
    test_precontact_admission()
    print('CPU role regression passed: both directions, stable IDs, caps10/11, real URDF7-axis retreat with1mrad margin; no CUDA')
