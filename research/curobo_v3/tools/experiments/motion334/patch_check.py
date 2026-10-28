#!/usr/bin/env python3
"""MOTION-334 step 3: MOTION-326's CPU patch replica and rejected-frame audit over the whole cycle.

patch_eval.py (check-cpu) and flag_audit.py cover approach, contact_approach, extract and transport.
This wrapper adds conveyor_place (payload attached) and return_home (no payload) and otherwise runs
them unchanged. Both evaluate the attach snapshot with the base joints of each frame minus the base
advance, which also places the base correctly in the conveyor phases. Like MOTION-326's replica, it
leaves out the planner's own check of the task box before contact (TargetContactValidity), so for
approach to transport the GPU check (patch_eval.py check) is the reference.

  python patch_check.py patch-eval check-cpu --set ... [--patch patch.json] [--cuboids cuboids.json] --output ...
  python patch_check.py flag-audit --set ... --checked ... [--patch ...] [--cuboids ...] --output ...
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from experiments.motion326 import flag_audit, patch_eval

patch_eval.PHASES = {"approach": False, "contact_approach": False, "extract": False, "transport": True,
                     "conveyor_place": True, "return_home": False}
flag_audit.PAYLOAD_PHASES = {"transport", "conveyor_place"}

if __name__ == "__main__":
    tool = sys.argv.pop(1)
    {"patch-eval": patch_eval.main, "flag-audit": flag_audit.main}[tool]()
