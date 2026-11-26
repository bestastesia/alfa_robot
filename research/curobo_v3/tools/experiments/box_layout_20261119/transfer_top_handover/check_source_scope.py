"""CPU contract regression only: transport may finish clearing extraction residuals."""
from pathlib import Path
import json
import sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[4];OUT=ROOT/'generated/two_box_layout/joint_contract_20261124_top_handover'
sys.path.insert(0,str(ROOT/'tools'))
from curobo_core.box_pick import BoxPick,source_slot_observation,source_slot_process_checks

if __name__=='__main__':
    request=json.loads((OUT/'y85_both_top20_high70_request.json').read_text());pick=BoxPick(**dict(next(b for b in request['boxes'] if b['side']=='right'),side='left'))
    source=np.eye(4);source[:3,3]=pick.center_m
    extracted=source.copy();extracted[0,3]-=pick.extraction_distance_m;extracted[:3,:3]=Rotation.from_rotvec([0.,.001,0.]).as_matrix()
    transported=source.copy();transported[0,3]-=pick.extraction_distance_m+.01
    extract=np.stack([source,extracted]);transport=np.stack([extracted,transported]);joined=np.stack([source,extracted,transported])
    checks={name:source_slot_process_checks({'left':source_slot_observation(pick,rows)}) for name,rows in [('extract_only',extract),('transport',transport),('combined',joined)]}
    assert not checks['extract_only']['valid'] and checks['transport']['valid'] and checks['combined']['valid']
    assert checks['extract_only']['source_slot_observation']['left']['deeper_than_initial_m']==0
    result=dict(valid=True,checks=checks,scope='Synthetic rigid-box pose counterexample of process-gate scope only, not a robot path or a replacement for the failed run. Same1um no-deepen/no-return policy applied unchanged to the whole extraction+transport prefix.')
    (OUT/'source_scope_cpu.json').write_text(json.dumps(result,indent=2)+'\n');print('Same source policy: extract-only rejects; transport and complete prefix pass')
