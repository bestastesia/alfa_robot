#!/usr/bin/env python3
"""Export controller-ready arm segments from a timed full-wall result."""
import argparse
import json
from pathlib import Path

from curobo_core.trajectory import controller_segments
from v3_wall_ik_benchmark import ACTIVE_JOINTS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--allow-missing-torque", action="store_true")
    args = parser.parse_args()
    result = controller_segments(
        json.loads(args.input.read_text()), ACTIVE_JOINTS,
        require_torque=not args.allow_missing_torque)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False) + "\n")
    print(json.dumps({"controller_ready": result["controller_ready"],
                      "segments": len(result["segments"]), "output": str(args.output)},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
