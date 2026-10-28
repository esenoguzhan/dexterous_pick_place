"""place_check.py X Y H R: why a side-grasp place at (X, Y) fails, per heading and segment (no motion)."""
import math
import sys

import numpy as np

import grasp as gr
import ik
import motion as mo
import restore as rs
import side as sd

x, y, h, radius = (float(v) for v in sys.argv[1:5])
t = rs.Restorer()
try:
    cur = rs.capture(t.scene, "pc", shots=1, need=1)
    table = rs.table_z(cur, (x, y))
    obj = {"position": [x, y, table + h / 2], "size": [0.06, 0.06, h], "top_radius": radius, "top_z": table + h}
    others = [o for o in cur["objects"] if np.hypot(o["position"][0] - x, o["position"][1] - y) > 0.05]
    t.r.set_scene_boxes([rs.object_box(o) for o in others])
    sg, cfg = t.side, t.side.cfg
    oz = sd.origin_height(h, table, cfg)[0] + cfg["place_drop_m"]
    for off in cfg["headings_deg"]:
        yaw = gr.radial_yaw((x, y)) + math.radians(off)
        cam = all(sg.camera_clear(sd.hand_pose((x, y), yaw, oz, radius, cfg, back=b, off=o), others)
                  for b, o in ((0.0, 0.0), (0.0, cfg["side_off_m"])))
        T_fin = sg.flange(sd.hand_pose((x, y), yaw, oz, radius, cfg))
        q, ep, er = None, 1, 1
        for seed in [t.r.arm_joints()] + mo.load_seeds()[:8]:
            q, ep, er = ik.solve(t.r, T_fin, seed)
            if ep < 2e-3 and er < 0.01:
                break
        fin_ok = ep < 2e-3 and er < 0.01
        msg = ""
        if not fin_ok:
            import kin
            Tq = kin.fk(q)
            try:
                ik.fk(t.r, q)
                why = "service ok"
            except Exception as exc:
                why = str(exc)[:110]
            msg = f"local error {1000 * np.linalg.norm(Tq[:3, 3] - T_fin[:3, 3]):.1f} mm; {why}"
        if fin_ok:
            T_high = T_fin.copy()
            T_high[2, 3] += 0.2
            plan = t.r.plan_pose(T_high, start=q)
            msg = "lift ok" if plan.success else "lift: " + plan.message[:90]
        print(f"heading {off:+.0f}: camera clear {cam}, grasp-pose IK {'ok' if fin_ok else f'{1000*ep:.0f} mm off'}; {msg}")
finally:
    t.r.set_scene_boxes([])
    t.close()
    import os
    os._exit(0)
