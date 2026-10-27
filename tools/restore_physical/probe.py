"""probe.py: capture the fixed camera, print world xyz of given pixels and the objects found."""
import sys

import numpy as np

import scene as sc

s = sc.Scene()
result, frame, snap = s.capture("probe")
T = s.cam.world_pose()
print("table height", round(result["table_height"], 4), "plane", np.round(result["table_plane"], 4).tolist())
for arg in sys.argv[1:]:
    u, v = (int(x) for x in arg.split(","))
    p = sc.pixel_to_world(frame, T, u, v)
    print(f"pixel ({u},{v}) ->", None if p is None else np.round(p, 3).tolist())
for o in result["objects"]:
    print(o["id"], "xyz", np.round(o["position"], 3).tolist(), "box", np.round(o["box_position"][:2], 3).tolist(), "top_r", round(o["top_radius"], 3), "top_centre", np.round(o["top_centre"], 3).tolist(), "fit", o["axis_fit"], "size", np.round(o["size"], 3).tolist(),
          "yaw", round(np.degrees(o["yaw"]), 1), "pts", o["points"], "sat", round(o["sat"], 2), "val", round(o["val"], 2))
print("warnings", result["warnings"])
