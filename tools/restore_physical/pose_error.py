"""pose_error.py X Y: open hand over an empty spot at hover and at grasp height; real vs model hand each time."""
import sys

import numpy as np

import measure_hand as mh
import restore as rs

x, y = float(sys.argv[1]), float(sys.argv[2])
t = rs.Restorer()
try:
    t.observe()
    cur = rs.capture(t.scene, "pe")
    t.r.set_scene_boxes([rs.object_box(o) for o in cur["objects"]])
    table = rs.table_z(cur, (x, y))
    top = np.array([x, y, table + 0.113])
    pick = t.solve(top, t.cfg["hover_clearance_m"], 0.037, cur["objects"], table)
    t.r.move_joints(pick[0], max_speed=0.35)
    print("HOVER"); mh.measure(t.r, t.scene, "_hover")
    T = pick[2].copy()
    T[2, 3] -= t.cfg["hover_clearance_m"]
    t.r.move_pose(T, duration=4.0)
    print("GRASP HEIGHT"); mh.measure(t.r, t.scene, "_grasp")
    t.r.move_pose(pick[2], duration=3.0)
finally:
    t.close()
