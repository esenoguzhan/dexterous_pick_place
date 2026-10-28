"""Side grasp of upright objects, with the last approach corrected by the wrist camera's depth.

Hand frame (+z fingers, +y out of the palm, +x little finger, thumb at -x). For a side grasp the
fingers point along the approach direction d (horizontal), the palm faces the object sideways and
the thumb points up (hand +x = world down), so the open, abducted thumb passes over the object and
the wrist camera (palm side, near the wrist) looks along d straight at it:

    pre-approach  hand `back` metres behind the grasp along -d, palm `palm_gap` further off the object
    wrist look    the D435 on the wrist sees the object ~25 cm ahead; its depth refines the axis
    approach      straight along d to the grasp pose, then sideways onto the object
    close         thumb, fingers together; lift straight up

side.py test X Y [--go] [--close] [--keep]
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

import cv2
import numpy as np
import yaml

import grasp as gr
import ik
import jetson
import motion as mo
import robot as rb
import scene as sc
from restore_perception.core.extrinsics import load_extrinsics

CONFIG = Path(__file__).resolve().parent / "config" / "side.yaml"


def settings():
    return yaml.safe_load(CONFIG.read_text())


def hand_rotation(yaw):
    """Fingers along (cos yaw, sin yaw, 0), thumb up, palm facing left of the approach."""
    d = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    x = np.array([0.0, 0.0, -1.0])
    y = np.cross(d, x)
    return np.column_stack([x, y, d])


def hand_pose(centre_xy, yaw, origin_z, radius, cfg, back=0.0, off=0.0):
    """T_world_hand with the object axis at (0, palm_gap + radius + off, grip_z) in the hand frame,
    moved `back` metres against the approach direction."""
    R = hand_rotation(yaw)
    p = np.array([0.0, cfg["palm_gap_m"] + radius + off, cfg["grip_z"] + back])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array([centre_xy[0], centre_xy[1], origin_z]) - R @ p
    return T


def origin_height(obj_height, table, cfg):
    """Hand origin height: high enough for the wrist camera below it, low enough that the middle
    finger (hand x = -0.01, 1 cm above the origin) still wraps the object."""
    lo = table + cfg["min_origin_height_m"]
    hi = table + obj_height - cfg["top_wrap_m"]
    # Tall objects are held near their top: the whole arm then stays higher, which keeps the forearm
    # off the table and over the other objects when it reaches far out.
    want = max(cfg["origin_height_m"], obj_height - cfg["grip_below_top_m"])
    return max(lo, min(hi, table + want)), hi >= lo


def wrist_refine(r, prior_xy, top_z, table, radius, cfg, save=None):
    """Object axis from the wrist camera's depth: points near the prior, above the table and below
    the top, circle-fitted. Returns (xy, radius, n points) or None."""
    cam = load_extrinsics(Path(__file__).resolve().parent / "config" / "extrinsics.yaml")[jetson.WRIST]
    frame = jetson.capture(jetson.WRIST, frames=5)
    T = r.frame_pose("lara5_flange", "world") @ cam.T_parent_cam
    K = frame.intrinsics.matrix()
    v, u = np.nonzero(frame.depth > 0)
    z = frame.depth[v, u]
    p = np.c_[(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z]
    w = p @ T[:3, :3].T + T[:3, 3]
    near = (np.hypot(w[:, 0] - prior_xy[0], w[:, 1] - prior_xy[1]) < radius + cfg["refine_window_m"]) & \
           (w[:, 2] > table + 0.015) & (w[:, 2] < top_z - 0.005)
    pts = w[near]
    if save:
        img = frame.color[..., ::-1].copy()
        img[v[near], u[near]] = (0, 255, 0)
        cv2.imwrite(save, img)
    fit = sc.fit_axis(pts[:, :2], np.random.default_rng(0)) if len(pts) > 50 else None
    if fit is None or abs(fit[1] - radius) > 0.012 or fit[2] < 0.3:
        return None, len(pts)
    return (fit[0], fit[1], len(pts)), len(pts)


class SideGrasp:
    def __init__(self, r, T_flange_hand, cfg=None):
        self.r, self.T_flange_hand = r, T_flange_hand
        self.cfg = cfg or settings()

    def flange(self, T_hand):
        return gr.flange_pose(T_hand, self.T_flange_hand)

    def camera_clear(self, T_hand, others, margin=0.005):
        """The wrist camera box (grasp.yaml) at this hand pose must stay out of the other objects."""
        box = np.array(np.meshgrid(*gr.settings()["camera_box"])).reshape(3, -1).T
        pts = box @ T_hand[:3, :3].T + T_hand[:3, 3]
        lo_c, hi_c = pts.min(axis=0), pts.max(axis=0)
        for o in others:
            r = o["top_radius"] + margin
            lo = np.array([o["position"][0] - r, o["position"][1] - r, o["top_z"] - o["size"][2]])
            hi = np.array([o["position"][0] + r, o["position"][1] + r, o["top_z"] + margin])
            if np.all(hi_c >= lo) and np.all(lo_c <= hi):
                return False
        return True

    def plan(self, obj, table, others=(), log=print):
        """(q_high, yaw, origin_z) for the waypoint above the pre-approach, trying approach directions
        around radial. Every segment (joint move up there, down to the pre-approach, slide in, lift) is
        checked by the safety filter first."""
        cfg = self.cfg
        xy = np.array(obj["position"][:2])
        oz, ok = origin_height(obj["size"][2], table, cfg)
        if not ok:
            return None
        radial = gr.radial_yaw(xy)
        current = self.r.arm_joints()
        best = None
        for off in cfg["headings_deg"]:
            yaw = radial + math.radians(off)
            if not all(self.camera_clear(hand_pose(xy, yaw, oz, obj["top_radius"], cfg, back=b, off=o), others)
                       for b, o in ((cfg["back_m"], cfg["side_off_m"]), (0.0, cfg["side_off_m"]), (0.0, 0.0))):
                continue
            T_pre = self.flange(hand_pose(xy, yaw, oz, obj["top_radius"], cfg, back=cfg["back_m"], off=cfg["side_off_m"]))
            T_fin = self.flange(hand_pose(xy, yaw, oz, obj["top_radius"], cfg))
            T_high = T_pre.copy()
            T_high[2, 3] += cfg["high_m"]
            for seed in [current] + mo.load_seeds()[:6]:
                q, ep, er = ik.solve(self.r, T_high, seed)
                if not (ep < 2e-3 and er < 0.01):
                    continue
                q = mo.wrap_near(q, current)
                down = self.r.plan_pose(T_pre, start=q)
                if not down.success:
                    continue
                q_pre = np.array(down.trajectory.points[-1].positions)
                slide = self.r.plan_pose(T_fin, start=q_pre)
                if not slide.success:
                    continue
                q_fin = np.array(slide.trajectory.points[-1].positions)
                T_up = T_fin.copy()
                T_up[2, 3] += cfg["lift_m"]
                if not self.r.plan_pose(T_up, start=q_fin).success:
                    continue
                valid, msg = self.r.check_joints(current, q)
                if not valid:
                    log(f"heading {off:+.0f}: joint move refused ({msg[:80]})")
                    continue
                cost = float(np.sum(np.abs(q - current)))
                if best is None or cost < best[3]:
                    best = (q, yaw, oz, cost)
                break
            if best is not None and best[3] < 2.0:
                break
        if best is not None:
            mo.save_seed(best[0])
        return None if best is None else best[:3]

    def execute(self, obj, table, q_high, yaw, oz, close=True, keep=False, log=print, refine=True, lift=None):
        cfg, r = self.cfg, self.r
        radius = obj["top_radius"]
        xy = np.array(obj["position"][:2])
        r.hand(cfg["open"], 1.0)
        r.move_joints(q_high, max_speed=0.3)
        r.hand(cfg["preshape"], 1.5)
        r.move_pose(self.flange(hand_pose(xy, yaw, oz, radius, cfg, back=cfg["back_m"], off=cfg["side_off_m"])), speed=0.05)
        self.refined = None
        if refine:
            try:
                got, n = wrist_refine(r, xy, obj["top_z"], table, radius, cfg, save="/run_data/wrist_refine.jpg")
            except Exception as exc:  # the wrist camera on USB 2 drops out now and then
                got, n = None, f"camera error {str(exc)[:60]}"
            if got is not None:
                shift = got[0] - xy
                log(f"wrist depth: axis {np.round(got[0], 3).tolist()} r{1000 * got[1]:.0f} from {n} points, "
                    f"{1000 * np.linalg.norm(shift):.0f} mm from the fixed camera's")
                if np.linalg.norm(shift) < cfg["max_refine_shift_m"]:
                    xy, radius = got[0], got[1]
                    self.refined = (xy, radius)
                else:
                    log("wrist correction too large; keeping the fixed camera's position")
            else:
                log(f"wrist depth: no usable fit ({n} points); keeping the fixed camera's position")
        T_pre = self.flange(hand_pose(xy, yaw, oz, radius, cfg, back=cfg["back_m"], off=cfg["side_off_m"]))
        T_near = self.flange(hand_pose(xy, yaw, oz, radius, cfg, off=cfg["side_off_m"]))
        T_fin = self.flange(hand_pose(xy, yaw, oz, radius, cfg))
        if self.refined is not None:
            # The correction moved the lines that were checked at planning: check them again from here.
            q = r.arm_joints()
            for T in (T_pre, T_near, T_fin):
                plan = r.plan_pose(T, start=q)
                if not plan.success:
                    log(f"corrected approach refused ({plan.message[:90]}); re-planning")
                    return "replan"
                q = np.array(plan.trajectory.points[-1].positions)
        for label, T, v in (("pre-approach", T_pre, 0.03), ("slide in", T_near, 0.03), ("onto the object", T_fin, 0.02)):
            log(f"{label}")
            r.move_pose(T, speed=v)
        if not close:
            return "at grasp pose"
        hj = r.hand(cfg["close"], cfg["close_s"])
        log(f"closed at {np.round(hj, 2).tolist()}")
        fingers = hj[1:5]
        if np.min(fingers) > cfg["missed_rad"]:
            self.release()
            r.hand(cfg["preshape"], 1.0)
            r.move_pose(T_near, speed=0.03)
            r.move_pose(T_pre, speed=0.04)
            return "missed: the fingers closed on nothing"
        T_up = T_fin.copy()
        T_up[2, 3] += lift or cfg["lift_m"]
        r.move_pose(T_up, speed=cfg["lift_speed_m_s"])
        hj = r.hand_joints()
        if np.min(hj[1:5]) > cfg["missed_rad"]:
            return "dropped while lifting"
        if keep:
            return "holding"
        return "lifted"

    def plan_place(self, home_xy, obj, table, others=(), lift=None, log=print):
        """(q_high, yaw, origin_z, lift) to put the held object at home_xy, or None. Checked: the joint
        move to the carry pose above home, the descent, the slide back out and the lift after it."""
        cfg = self.cfg
        lift = lift or cfg["lift_m"]
        radius = obj["top_radius"]
        oz = origin_height(obj["size"][2], table, cfg)[0] + cfg["place_drop_m"]
        radial = gr.radial_yaw(home_xy)
        current = self.r.arm_joints()
        dy, dz = cfg["held_shift_m"]
        for off in cfg["headings_deg"]:
            yaw = radial + math.radians(off)
            if not all(self.camera_clear(hand_pose(home_xy, yaw, oz, radius, cfg, back=b, off=o), others)
                       for b, o in ((0.0, 0.0), (0.0, cfg["side_off_m"]), (cfg["back_m"], cfg["side_off_m"]))):
                continue
            T_fin = self.flange(hand_pose(home_xy, yaw, oz, radius, cfg, back=dz, off=dy))
            T_high = T_fin.copy()
            T_high[2, 3] += lift
            why = "no IK for the carry pose"
            for seed in [current] + mo.load_seeds()[:6]:
                q, ep, er = ik.solve(self.r, T_high, seed)
                if not (ep < 2e-3 and er < 0.01):
                    continue
                q = mo.wrap_near(q, current)
                down = self.r.plan_pose(T_fin, start=q)
                if not down.success:
                    why = "descent: " + down.message[:80]
                    continue
                q_fin = np.array(down.trajectory.points[-1].positions)
                T_out = self.flange(hand_pose(home_xy, yaw, oz, radius, cfg, back=dz, off=dy + cfg["side_off_m"]))
                out = self.r.plan_pose(T_out, start=q_fin)
                if not out.success:
                    why = "off the object: " + out.message[:80]
                    continue
                T_out_up = T_out.copy()
                T_out_up[2, 3] += max(lift, cfg["high_m"])
                up = self.r.plan_pose(T_out_up, start=np.array(out.trajectory.points[-1].positions))
                if not up.success:
                    why = "up: " + up.message[:80]
                    continue
                return q, yaw, oz, lift
            log(f"place heading {off:+.0f}: {why}")
        return None

    def place(self, home_xy, obj, q_high, yaw, oz, lift, log=print):
        """From holding (lifted after the pick): carry, lower, release, slide out, go up."""
        cfg, r = self.cfg, self.r
        radius = obj["top_radius"]
        dy, dz = cfg["held_shift_m"]
        T_fin = self.flange(hand_pose(home_xy, yaw, oz, radius, cfg, back=dz, off=dy))
        T_high = T_fin.copy()
        T_high[2, 3] += lift
        try:
            r.move_pose(T_high, speed=0.05)
        except rb.MotionError as exc:
            log(f"straight carry refused ({str(exc)[:80]}); joint move")
            r.move_joints(q_high, max_speed=0.25)
        r.move_pose(T_fin, speed=0.03)
        self.release()
        # Straight up first: the open fingers are beside the object and the open thumb above it, so
        # nothing catches; a low slide back along the fingers swept the arm towards the table.
        T_near = self.flange(hand_pose(home_xy, yaw, oz, radius, cfg, back=dz, off=dy + cfg["side_off_m"]))
        r.move_pose(T_near, speed=0.02)
        T_up = T_near.copy()
        T_up[2, 3] += max(lift, cfg["high_m"])
        r.move_pose(T_up, speed=0.04)
        return "placed"

    def release(self):
        hj = self.r.hand_joints()
        self.r.hand([hj[0], hj[1], hj[2], hj[3], hj[4], 0.0], 0.8)   # thumb first
        self.r.hand([0.0, hj[1], hj[2], hj[3], hj[4], 0.0], 0.8)
        self.r.hand(self.cfg["open"], 1.2)


if __name__ == "__main__":
    import restore as rs
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["test", "grip", "putdown"])
    ap.add_argument("x", type=float, nargs="?", default=0.0)
    ap.add_argument("y", type=float, nargs="?", default=0.0)
    ap.add_argument("--lift", type=float, default=0.03)
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--close", action="store_true")
    ap.add_argument("--keep", action="store_true")
    ap.add_argument("--no-refine", action="store_true")
    a = ap.parse_args()
    t = rs.Restorer()
    if a.cmd in ("grip", "putdown"):
        # Step by step from where the hand is: grip = close and lift `--lift`; putdown = lower by
        # `--lift`, release, slide back out along the fingers and away from the palm, go up.
        sg, r = t.side, t.r
        try:
            T = r.frame_pose("lara5_flange", "world")
            if a.cmd == "grip":
                hj = r.hand(sg.cfg["close"], sg.cfg["close_s"])
                print("closed at", np.round(hj, 2).tolist())
                T[2, 3] += a.lift
                r.move_pose(T, speed=sg.cfg["lift_speed_m_s"])
                print("lifted; hand", np.round(r.hand_joints(), 2).tolist())
            else:
                T[2, 3] -= a.lift
                r.move_pose(T, speed=sg.cfg["lift_speed_m_s"])
                sg.release()
                H = r.frame_pose("right_hand_base_link", "world")
                out = H.copy()
                out[:3, 3] -= sg.cfg["side_off_m"] * H[:3, 1]
                r.move_pose(sg.flange(out), speed=0.02)
                out[:3, 3] -= sg.cfg["back_m"] * H[:3, 2]
                r.move_pose(sg.flange(out), speed=0.04)
                out[2, 3] += sg.cfg["high_m"]
                r.move_pose(sg.flange(out), speed=0.06)
                print("released and clear")
        finally:
            t.close()
        raise SystemExit
    try:
        t.observe()
        cur = rs.capture(t.scene, "side")
        obj = min(cur["objects"], key=lambda o: np.hypot(o["position"][0] - a.x, o["position"][1] - a.y))
        others = [o for o in cur["objects"] if o is not obj]
        table = rs.table_z(cur, obj["position"])
        print("object", rs.describe(obj))
        t.r.set_scene_boxes([rs.object_box(o) for o in others])
        sg = SideGrasp(t.r, t.T_flange_hand)
        plan = sg.plan(obj, table, others)
        if plan is None:
            raise SystemExit("no side approach found")
        print("approach heading", round(math.degrees(plan[1] - gr.radial_yaw(obj["position"])), 1),
              "origin height above table", round(plan[2] - table, 3))
        if a.go:
            print(sg.execute(obj, table, *plan, close=a.close, keep=a.keep, refine=not a.no_refine))
    except BaseException:
        import traceback
        traceback.print_exc()
    finally:
        t.close()
        os._exit(0)  # rclpy's executor thread sometimes keeps the process alive
