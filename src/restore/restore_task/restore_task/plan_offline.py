"""Diff and plan two snapshots saved by perception, without a robot (docs/task_architecture.md, 12).

    python -m restore_task.plan_offline data/snapshots/reference data/snapshots/<current>
    ros2 run restore_task plan_offline data/snapshots/reference data/snapshots/<current>
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from restore_perception.core.config import load_objects

from .core import scene as sc
from .core.config import config_dir, load_config, objects_file
from .core.diff import diff
from .core.order import plan


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("reference", type=Path)
    parser.add_argument("current", type=Path)
    parser.add_argument("--config", type=Path, default=config_dir() / "task.yaml")
    parser.add_argument("--objects", type=Path, default=objects_file())
    args = parser.parse_args(argv)
    cfg, objects = load_config(args.config), load_objects(args.objects)
    reference, current = sc.from_disk(args.reference), sc.from_disk(args.current)
    d = diff(reference, current, objects, cfg)
    p = plan(d, current, cfg)
    d.refused.extend(p.refused)

    print(f"{len(d.tasks)} moved, {len(d.in_place)} in place, {len(d.refused)} refused, "
          f"{len(d.missing)} missing, {len(d.new)} new")
    for i, step in enumerate(p.steps, 1):
        t = step.task
        flags = f" [{', '.join(t.flags)}]" if t.flags else ""
        print(f"{i}. {t.label} ({t.object_id}): {'park at' if step.park else 'home'} "
              f"({step.target.x:+.3f}, {step.target.y:+.3f}), moved {1000 * t.dxy:.0f} mm, "
              f"turn {math.degrees(t.dyaw):+.0f} deg, grasp {t.primitive}{flags}")
    for r in d.refused:
        print(f"refused: {r.label} ({r.object_id}): {r.reason}")
    for ref_id in d.missing:
        print(f"missing: {ref_id}")
    for cur_id in d.new:
        print(f"new: {cur_id}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
