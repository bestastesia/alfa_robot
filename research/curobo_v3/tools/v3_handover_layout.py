"""Fresh one-box handover recovery for the joint-defined two-box task."""
from pathlib import Path

from curobo_core.layout_contract import validate_pair


def plan_handover_pick(picks, output, max_contacts=16, progress=lambda event: None):
    contract = validate_pair(picks)
    # Preserve the physical request; grasp, intermediate poses and lift remain planner choices.
    document = dict(frame_id='world', front_distance_m=contract['front_distance_m'], boxes=[
        dict(size=pick.size, side=side, center_m=list(pick.center_m),
             side_min_height_m=pick.side_min_height_m,
             **({'swap_length_height': True} if pick.swap_length_height else {}))
        for side, pick in picks.items()])
    from experiments.box_layout_20261119.transfer_fresh_native.plan_native import plan_document
    return plan_document(document, Path(output), progress, max_contacts=max_contacts)
