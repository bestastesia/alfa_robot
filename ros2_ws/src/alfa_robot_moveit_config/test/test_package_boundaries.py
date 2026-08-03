#!/usr/bin/env python3
"""Guard production packages from depending on experiment source paths."""

from __future__ import annotations

import argparse
from pathlib import Path


FORBIDDEN = (
    "scripts/ik_benchmark",
    "IK_BENCHMARK_ROOT",
    '"ik_benchmark/',
    "ik_benchmark::",
)


def source_files(root: Path):
    source_root = root / "ros2_ws/src"
    package_roots = [
        path
        for path in source_root.iterdir()
        if path.is_dir()
    ]
    suffixes = {".cpp", ".hpp", ".h", ".py", ".xml", ".txt"}
    for package_root in package_roots:
        for path in package_root.rglob("*"):
            if not path.is_file() or path.suffix not in suffixes:
                continue
            if "test" in path.parts or "__pycache__" in path.parts:
                continue
            yield path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo_root", type=Path)
    args = parser.parse_args()

    violations = []
    for path in source_files(args.repo_root.resolve()):
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern in FORBIDDEN:
            if pattern in text:
                violations.append(f"{path}: forbidden dependency {pattern!r}")

    if violations:
        print("\n".join(violations))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
