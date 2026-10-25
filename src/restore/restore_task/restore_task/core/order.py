"""Section 6: in which order the tasks run, and where an object is parked when two block each other."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from .diff import Refusal
from .poses import Placement


class Grid:
    """Occupancy raster of the table in world xy, for footprint overlaps."""

    def __init__(self, low, high, step):
        self.low, self.step = np.asarray(low, float), float(step)
        self.shape = tuple((np.ceil((np.asarray(high) - self.low) / step).astype(int) + 1)[::-1])  # rows, cols

    def mask(self, polygon_xy, grow=0.0):
        mask = np.zeros(self.shape, np.uint8)
        polygon_xy = np.asarray(polygon_xy, float).reshape(-1, 2)
        if len(polygon_xy) == 0:
            return mask.astype(bool)
        cells = np.round((polygon_xy - self.low) / self.step).astype(np.int32)
        if len(cells) >= 3:
            cv2.fillPoly(mask, [cv2.convexHull(cells.reshape(-1, 1, 2))], 1)
        else:
            mask[np.clip(cells[:, 1], 0, self.shape[0] - 1), np.clip(cells[:, 0], 0, self.shape[1] - 1)] = 1
        radius = int(math.ceil(grow / self.step))
        if radius > 0:
            mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1,) * 2))
        return mask.astype(bool)


@dataclass
class Step:
    task: object  # diff.Task
    target: Placement  # the home, or a parking spot
    park: bool = False


@dataclass
class Plan:
    steps: list = field(default_factory=list)
    refused: list = field(default_factory=list)  # Refusal: blocked or no parking spot
    dependencies: dict = field(default_factory=dict)  # object id -> ids that must move first


def _parking_spot(task, grid, occupied, scene, cfg, area):
    """Nearest free place for the object's current footprint, keeping its yaw."""
    step = cfg["parking_step_m"]
    xs = np.arange(area["min"][0], area["max"][0] + 1e-9, step)
    ys = np.arange(area["min"][1], area["max"][1] + 1e-9, step)
    candidates = sorted(((float(np.hypot(x - task.current.x, y - task.current.y)), x, y) for x in xs for y in ys))
    footprint = task.current_footprint - task.current.xy
    for _, x, y in candidates:
        mask = grid.mask(footprint + [x, y], cfg["home_clearance_m"])
        if not (mask & occupied).any():
            z = scene.table_z(x, y) + float(task.size[2]) / 2.0
            return Placement(x, y, z, task.current.yaw), mask
    return None, None


def plan(diff, scene, config, hand_xy=(0.0, 0.0)):
    """Order the tasks of a Diff on the current Scene. Only the first step is run before re-planning."""
    cfg, area = config["order"], config["reach_area"]
    grid = Grid(np.array(area["min"]) - 0.25, np.array(area["max"]) + 0.25, cfg["grid_m"])
    objects = scene.by_id()
    tasks = {t.object_id: t for t in diff.tasks}
    current = {oid: grid.mask(np.asarray(o.footprint, float)[:, :2]) for oid, o in objects.items()}
    homes = {oid: grid.mask(t.home_footprint, cfg["home_clearance_m"]) for oid, t in tasks.items()}
    result = Plan()

    deps = {oid: set() for oid in tasks}
    blocked = {}
    for oid, task in tasks.items():
        home_support = diff.current_of(task.home_support) if task.home_support != "table" else None
        for other, mask in current.items():
            if other == oid or other == home_support or not (homes[oid] & mask).any():
                continue
            if other in tasks:
                deps[oid].add(other)  # it sits on this home: it moves first
            else:
                blocked.setdefault(oid, []).append(other)
        for other, o in objects.items():  # something resting on this object moves first
            if o.support_id == oid and other != oid:
                if other in tasks:
                    deps[oid].add(other)
                else:
                    blocked.setdefault(oid, []).append(other)
        if home_support in tasks:  # its home rests on another object: that one goes home first
            deps[oid].add(home_support)
    result.dependencies = {k: sorted(v) for k, v in deps.items()}

    for oid, others in blocked.items():
        result.refused.append(Refusal(oid, tasks[oid].reference_id, tasks[oid].label,
                                      f"blocked by {', '.join(sorted(set(others)))}, which stays where it is"))
    remaining = [oid for oid in tasks if oid not in blocked]
    reserved = np.zeros(grid.shape, bool)  # every home, and spots already chosen for parking
    for mask in homes.values():
        reserved |= mask
    parked = set()
    position = np.asarray(hand_xy, float)
    while remaining:
        ready = [oid for oid in remaining if not (deps[oid] & set(remaining))]
        if ready:
            oid = min(ready, key=lambda k: np.linalg.norm(tasks[k].current.xy - position))
            result.steps.append(Step(tasks[oid], tasks[oid].home))
            remaining.remove(oid)
            position = tasks[oid].home.xy
            continue
        # A cycle: park the nearest object that has not been parked yet.
        options = [oid for oid in remaining if oid not in parked]
        if not options:
            for oid in remaining:
                result.refused.append(Refusal(oid, tasks[oid].reference_id, tasks[oid].label,
                                              "waits on objects that cannot be cleared"))
            break
        oid = min(options, key=lambda k: np.linalg.norm(tasks[k].current.xy - position))
        occupied = reserved.copy()
        for other, mask in current.items():  # every object but this one, where it is now
            if other != oid:
                occupied |= mask
        spot, mask = _parking_spot(tasks[oid], grid, occupied, scene, cfg, area)
        if spot is None:
            result.refused.append(Refusal(oid, tasks[oid].reference_id, tasks[oid].label, "no free parking spot"))
            remaining.remove(oid)
            continue
        result.steps.append(Step(tasks[oid], spot, park=True))
        parked.add(oid)
        reserved |= mask
        current[oid] = mask
        for other in remaining:  # once parked it blocks no home
            deps[other].discard(oid)
        position = spot.xy
    return result
