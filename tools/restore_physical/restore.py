"""Restore the tidy scene with the real LARA5 + Inspire hand.

    restore.py reference          arm to the observe pose, record the tidy scene
    restore.py run [--dry]        find the moved objects and put each one back, then check
    restore.py observe            arm to the observe pose only

Perception: the fixed D435 over HTTP from the Jetson (scene.py). Every arm motion goes through the
safety filter, which also gets every object except the one in hand as a box.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

import grasp as gr
import jetson
import match as mt
import motion as mo
import robot as rb
import scene as sc
import side as sd

RUN = Path("/run_data")
REFERENCE = RUN / "reference.json"
# Arm folded up beside the base: out of the fixed camera's view of the sheet.
OBSERVE = [-1.0654, -0.4808, 1.4492, 3.3252, 0.1777, -1.6418]
SHEET = {"min": [0.0, -0.29], "max": [0.47, 0.28]}   # where objects may be put (parking)
BOX_MARGIN = 0.005      # around other objects, on top of the safety filter's 20 mm minimum distance [m]
# Wrist D435 + holder keep-out, hand frame (the URDF has no collision shape for them). Measured by
# where the can touched it on 2026-10-27: the body reaches ~5.5 cm along the fingers.
CAMERA_BOX = np.array([[x, y, z] for x, y, z in np.array(np.meshgrid(*gr.settings()["camera_box"])).reshape(3, -1).T])
FINGERS_CLOSED = 1.3
THUMB_YAW, THUMB_PITCH = 1.16, 0.6
MISSED = 1.22           # fingers that close past this found nothing
LOG = []


def log(msg):
    line = f"[{time.strftime('%H:%M:%S')}] {msg}"
    print(line, flush=True)
    LOG.append(line)


# ---------- perception ----------

def capture(scene, name, shots=3, need=2):
    """Objects seen in at least `need` of `shots` captures, positions averaged."""
    runs = [scene.capture(f"{name}_{i}")[0] for i in range(shots)]
    groups = []
    for run in runs:
        for o in run["objects"]:
            for g in groups:
                if np.hypot(g[0]["position"][0] - o["position"][0], g[0]["position"][1] - o["position"][1]) < 0.03:
                    g.append(o)
                    break
            else:
                groups.append([o])
    kept = []
    for g in groups:
        if len(g) < min(need, shots):
            continue
        o = dict(g[0])
        o["position"] = np.mean([s["position"] for s in g], axis=0).tolist()
        o["top_z"] = float(np.mean([s["top_z"] for s in g]))
        o["top_radius"] = float(np.mean([s["top_radius"] for s in g]))
        o["size"] = np.mean([s["size"] for s in g], axis=0).tolist()
        o["hist"] = np.mean([s["hist"] for s in g], axis=0).tolist()
        o["sat"] = float(np.mean([s["sat"] for s in g]))
        o["val"] = float(np.mean([s["val"] for s in g]))
        kept.append(o)
    kept.sort(key=lambda o: (round(o["position"][0], 2), o["position"][1]))
    for i, o in enumerate(kept):
        o["name"] = f"o{i}"
    return {"table_height": runs[-1]["table_height"], "table_plane": runs[-1]["table_plane"], "objects": kept}


def describe(o):
    p = o["position"]
    return f"{o.get('name', o['id'])} at ({p[0]:.3f}, {p[1]:.3f}) h{1000 * o['size'][2]:.0f} r{1000 * o['top_radius']:.0f}"


def table_z(scene_result, xy):
    a, b, c, d = scene_result["table_plane"]
    return -(d + a * xy[0] + b * xy[1]) / c


# ---------- geometry checks ----------

def object_box(o, margin=BOX_MARGIN):
    r = o["top_radius"] + margin
    h = o["size"][2]
    bottom = o["top_z"] - h
    return (o.get("name", o["id"]), [o["position"][0], o["position"][1], bottom + (h + margin) / 2.0],
            [2 * r, 2 * r, h + margin])


def camera_clear(T_hand, table, others):
    """The wrist D435 hangs ~9 cm out of the palm and has no collision shape in the URDF."""
    pts = (T_hand[:3, :3] @ CAMERA_BOX.T).T + T_hand[:3, 3]
    if pts[:, 2].min() < table + 0.01:
        return False
    for o in others:
        _, c, s = object_box(o, 0.015)
        lo, hi = np.array(c) - np.array(s) / 2, np.array(c) + np.array(s) / 2
        box_lo, box_hi = pts.min(axis=0), pts.max(axis=0)
        # The descent sweeps the camera down to this pose: overlap in xy and lower than the box top.
        if np.all(box_hi[:2] >= lo[:2]) and np.all(box_lo[:2] <= hi[:2]) and box_lo[2] <= hi[2]:
            return False
    return True


# ---------- robot steps ----------

class Restorer:
    def __init__(self, dry=False):
        self.dry = dry
        self.cfg = gr.settings()
        self.scene = sc.Scene()
        self.r = rb.start("restore_task")
        self.T_flange_hand = self.r.frame_pose("right_hand_base_link", "lara5_flange")
        self.side = sd.SideGrasp(self.r, self.T_flange_hand)

    def close(self):
        self.r.close()

    def check_robot(self):
        state = rb.machine_state()
        if state.get("collision") or state.get("error"):
            raise rb.MotionError(f"the arm reports collision={state.get('collision')} error={state.get('error')}: "
                                 "check the scene, then clear it (neura/robot_state reset)")

    def observe(self):
        q = self.r.arm_joints()
        if np.max(np.abs(q - OBSERVE)) < 0.02:
            return
        T = self.r.frame_pose("lara5_flange", "world")
        if T[2, 3] < 1.3:  # get clear of the objects first
            up = T.copy()
            up[2, 3] = min(T[2, 3] + 0.15, 1.45)
            try:
                self.r.move_pose(up, duration=0.0)
            except rb.MotionError as exc:
                log(f"straight up refused ({exc}); joint move to observe directly")
        self.r.move_joints(OBSERVE, max_speed=0.35)

    def hand_pose_for(self, top, yaw, clearance, radius):
        return gr.hand_pose(top, yaw, clearance, radius, self.cfg)

    def solve(self, top, clearance, radius, others, table, lines=(-1.0,)):
        """Hover pose `clearance` above the object top, accepted only if the straight lines from it
        (lines: multiples of the clearance along world z, -1 = down to the grasp) can be planned."""
        def check(yaw):
            return camera_clear(self.hand_pose_for(top, yaw, 0.0, radius), table, others)

        def validate(q, T_flange):
            for k in lines:
                T = T_flange.copy()
                T[2, 3] += k * clearance
                plan = self.r.plan_pose(T, start=q)
                if not plan.success:
                    return False
            return True
        return mo.solve_hand(self.r, top, clearance, self.T_flange_hand, self.r.arm_joints(), radius=radius,
                             check=check, validate=validate)

    def pick_and_place(self, obj, home_xy, others, scene_result):
        """Side grasp with the wrist-depth correction (side.py), carry above the others, put down."""
        name = obj["name"]
        table_here = table_z(scene_result, obj["position"])
        table_home = table_z(scene_result, home_xy)
        lift = max(self.side.cfg["lift_m"], max([o["size"][2] for o in others] + [0.0]) + 0.04)
        self.r.set_scene_boxes([object_box(o) for o in others])
        pick = self.side.plan(obj, table_here, others, log=log)
        if pick is None:
            return "refused: no collision-free side approach"
        place = self.side.plan_place(home_xy, obj, table_home, others, lift, log=log)
        if place is None:
            return "refused: no collision-free way to put it at its home"
        log(f"{name}: approach heading {math.degrees(pick[1] - gr.radial_yaw(obj['position'])):+.0f}, "
            f"place heading {math.degrees(place[1] - gr.radial_yaw(home_xy)):+.0f}, carry {lift:.2f} m")
        if self.dry:
            return "dry run"
        outcome = self.side.execute(obj, table_here, *pick, close=True, keep=True, log=log, lift=lift)
        if outcome == "replan":
            xy, radius = self.side.refined
            obj = dict(obj, position=[float(xy[0]), float(xy[1]), obj["position"][2]], top_radius=float(radius))
            pick = self.side.plan(obj, table_here, others, log=log)
            if pick is None:
                return "refused: no collision-free side approach to the corrected position"
            log(f"{name}: new approach heading {math.degrees(pick[1] - gr.radial_yaw(obj['position'])):+.0f}")
            outcome = self.side.execute(obj, table_here, *pick, close=True, keep=True, log=log, lift=lift,
                                        refine=False)
        if outcome != "holding":
            return outcome
        return self.side.place(home_xy, obj, *place, log=log)

    def release(self):
        self.side.release()


# ---------- planning ----------

def free_at(xy, radius, objects, ignore=(), gap=0.02):
    return all(np.hypot(o["position"][0] - xy[0], o["position"][1] - xy[1]) > o["top_radius"] + radius + gap
               for o in objects if o["name"] not in ignore)


def parking_spot(obj, objects, homes):
    best = None
    for x in np.linspace(SHEET["min"][0], SHEET["max"][0], 10):
        for y in np.linspace(SHEET["min"][1], SHEET["max"][1], 12):
            xy = (x, y)
            if not free_at(xy, obj["top_radius"], objects, ignore=(obj["name"],), gap=0.05):
                continue
            if any(np.hypot(xy[0] - hx, xy[1] - hy) < obj["top_radius"] + hr + 0.05 for hx, hy, hr in homes):
                continue
            d = np.hypot(x - obj["position"][0], y - obj["position"][1])
            if best is None or d < best[0]:
                best = (d, xy)
    return None if best is None else best[1]


def cmd_reference(args):
    t = Restorer()
    try:
        t.observe()
        ref = capture(t.scene, "reference", shots=3)
    finally:
        t.close()
    REFERENCE.write_text(json.dumps(ref, indent=1))
    log(f"reference: {len(ref['objects'])} objects")
    for o in ref["objects"]:
        log("  " + describe(o))


def cmd_run(args):
    ref = json.loads(REFERENCE.read_text())
    t = Restorer(dry=args.dry)
    report = {}
    try:
        t.check_robot()
        for round_ in range(args.rounds):
            t.observe()
            cur = capture(t.scene, f"round{round_}")
            pairs, missing, new = mt.match(ref["objects"], cur["objects"])
            todo = [(r, c) for r, c, _ in pairs if mt.displacement(r, c) > args.tolerance]
            log(f"round {round_}: {len(cur['objects'])} objects, {len(pairs)} matched, {len(todo)} moved, "
                f"{len(missing)} missing, {len(new)} new")
            for r_, c, k in pairs:
                log(f"  {describe(c)} is reference {r_['name']} (cost {k:.2f}), off by "
                    f"{1000 * mt.displacement(r_, c):.0f} mm")
            for m in missing:
                report[m["name"]] = "missing"
            if not todo:
                break
            objects = cur["objects"]
            homes = [(r_["position"][0], r_["position"][1], r_["top_radius"]) for r_, _ in todo]
            progress, blocked = False, []
            for r_, c in sorted(todo, key=lambda rc: mt.displacement(*rc)):
                home = r_["position"][:2]
                if not free_at(home, c["top_radius"], objects, ignore=(c["name"],)):
                    blockers = [o for o in objects if o["name"] != c["name"] and not free_at(home, c["top_radius"], [o])]
                    log(f"{c['name']}: home taken by {[b['name'] for b in blockers]}; later")
                    blocked.append((r_, c))
                    continue
                others = [o for o in objects if o["name"] != c["name"]]
                # The reference capture is the clean one (3 shots, object well inside the sheet): an
                # object shuffled to an edge or behind another reads shorter. Plan with the taller.
                if r_["size"][2] > c["size"][2]:
                    c = dict(c, size=c["size"][:2] + [r_["size"][2]],
                             top_z=c["top_z"] + r_["size"][2] - c["size"][2])
                log(f"{c['name']} -> home ({home[0]:.3f}, {home[1]:.3f})")
                try:
                    t.check_robot()
                    outcome = t.pick_and_place(c, home, others, cur)
                except rb.MotionError as exc:
                    outcome = f"failed: {exc}"
                log(f"{c['name']}: {outcome}")
                report[r_["name"]] = outcome
                if outcome == "placed":
                    for o in objects:
                        if o["name"] == c["name"]:
                            o["position"] = [home[0], home[1], o["position"][2]]
                    progress = True
                elif outcome.startswith("failed"):
                    raise rb.MotionError(outcome)
            if not progress and blocked and not args.dry:
                # Every remaining home is taken: park one of the waiting objects somewhere free.
                r_, c = blocked[0]
                spot = parking_spot(c, objects, homes)
                if spot is None:
                    log("stuck: no free parking spot")
                    break
                log(f"parking {c['name']} at ({spot[0]:.3f}, {spot[1]:.3f})")
                t.pick_and_place(c, spot, [o for o in objects if o["name"] != c["name"]], cur)
            if args.dry or not progress and not blocked:
                break
        t.observe()
        final = capture(t.scene, "final")
        pairs, missing, new = mt.match(ref["objects"], final["objects"])
        log("final check:")
        for r_, c, _ in pairs:
            d = mt.displacement(r_, c)
            log(f"  {r_['name']}: {1000 * d:.0f} mm from home {'OK' if d <= args.tolerance else 'NOT restored'}")
        for m in missing:
            log(f"  {m['name']}: not seen")
    finally:
        t.r.set_scene_boxes([])
        t.close()
        (RUN / "restore_log.txt").write_text("\n".join(LOG))
        (RUN / "restore_report.json").write_text(json.dumps(report, indent=1))


def cmd_move(args):
    """Pick the object nearest (x, y) and put it at (tx, ty): a grasp test without a reference."""
    t = Restorer(dry=args.dry)
    try:
        t.check_robot()
        t.observe()
        cur = capture(t.scene, "move")
        obj = min(cur["objects"], key=lambda o: np.hypot(o["position"][0] - args.x, o["position"][1] - args.y))
        others = [o for o in cur["objects"] if o is not obj]
        log(f"moving {describe(obj)} to ({args.tx:.3f}, {args.ty:.3f})")
        log(t.pick_and_place(obj, (args.tx, args.ty), others, cur))
        t.observe()
    finally:
        t.r.set_scene_boxes([])
        t.close()


def brightness():
    """Mean grey level, and contrast (std): a covered lens gives a flat image even when the camera's
    auto-exposure brightens it."""
    f = jetson.capture(jetson.FIXED, frames=1)
    g = f.color.mean(axis=2)
    return float(g.mean()), float(g.std())


def keepalive(t):
    """The arm stops following after a long idle with no fault shown: a 0.1 deg wrist nudge and back,
    which also checks that it still follows."""
    q = t.r.arm_joints()
    q1 = q.copy()
    q1[5] += 0.002
    t.r.move_joints(q1, duration=1.0, tol=0.0015)
    t.r.move_joints(q, duration=1.0, tol=0.0015)


def wait_for(t, covered, level, poll=0.5, keep_every=120.0):
    """Block until the fixed camera is covered (dark) or uncovered and steady again."""
    last_keep = time.monotonic()
    steady = 0
    mean0, std0 = level
    while True:
        mean, std = brightness()
        dark = mean < 0.35 * mean0 or std < 0.35 * std0
        if covered and dark:
            return mean, std
        if not covered:
            steady = steady + 1 if (mean > 0.7 * mean0 and std > 0.7 * std0) else 0
            if steady >= 4:
                return mean, std
        if time.monotonic() - last_keep > keep_every:
            keepalive(t)
            last_keep = time.monotonic()
        time.sleep(poll)


def cmd_demo(args):
    """Record the order, wait until the camera is covered, then uncovered, then restore everything."""
    t = Restorer()
    try:
        t.check_robot()
        t.observe()
        keepalive(t)
        ref = capture(t.scene, "reference", shots=3)
        REFERENCE.write_text(json.dumps(ref, indent=1))
        log(f"reference: {len(ref['objects'])} objects")
        for o in ref["objects"]:
            log("  " + describe(o))
        level = brightness()
        log(f"COVER THE CAMERA NOW and shuffle the objects (image mean {level[0]:.0f}, contrast {level[1]:.0f})")
        wait_for(t, True, level)
        log("camera covered: shuffle the objects, then uncover the camera")
        wait_for(t, False, level)
        log("camera uncovered: restoring")
        time.sleep(2.0)
    finally:
        t.close()
    cmd_run(args)


def cmd_observe(args):
    t = Restorer()
    try:
        t.observe()
    finally:
        t.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("reference")
    run = sub.add_parser("run")
    run.add_argument("--dry", action="store_true")
    run.add_argument("--rounds", type=int, default=3)
    run.add_argument("--tolerance", type=float, default=0.025)
    sub.add_parser("observe")
    demo = sub.add_parser("demo")
    demo.add_argument("--rounds", type=int, default=3)
    demo.add_argument("--tolerance", type=float, default=0.025)
    demo.add_argument("--dry", action="store_true")
    mv = sub.add_parser("move")
    for k in ("x", "y", "tx", "ty"):
        mv.add_argument(k, type=float)
    mv.add_argument("--dry", action="store_true")
    a = ap.parse_args()
    try:
        {"reference": cmd_reference, "run": cmd_run, "observe": cmd_observe, "move": cmd_move,
         "demo": cmd_demo}[a.cmd](a)
    except BaseException:
        import traceback
        traceback.print_exc()
    finally:
        # rclpy's executor thread sometimes keeps the process alive after shutdown.
        sys.stdout.flush()
        os._exit(0)
