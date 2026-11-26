"""One caller-owned gpuq job: raw top-profile request -> existing fresh/native pipeline."""
from pathlib import Path
import argparse
import hashlib
import json
import sys
ROOT=Path(__file__).resolve().parents[4]
OUT=ROOT/'generated/two_box_layout/joint_contract_20261124_top_handover'
NATIVE=ROOT/'tools/experiments/box_layout_20261119/transfer_fresh_native'
sys.path.insert(0,str(NATIVE))
import plan_native

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',default='y85_top70_high130');p.add_argument('--attempt',default='attempt_01');p.add_argument('--core-sha256',required=True);args=p.parse_args()
    assert Path(args.case).name==args.case and Path(args.attempt).name==args.attempt
    core=ROOT/'tools/curobo_core/handover.py';actual=hashlib.sha256(core.read_bytes()).hexdigest()
    assert actual==args.core_sha256,('core not frozen',actual)
    document=json.loads((OUT/(args.case+'_request.json')).read_text())
    plan_native.OUT=OUT
    result=plan_native.plan_document(document,OUT/args.attempt,max_contacts=16)
    print('TOP_NATIVE_RESULT',result['success'],'wall',result['wall_seconds'],'motion',result['timing'].get('motion_seconds'),flush=True)
    sys.exit(0 if result['success'] else 1)
